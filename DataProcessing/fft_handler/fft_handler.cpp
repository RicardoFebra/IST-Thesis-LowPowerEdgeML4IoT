#include "fft_handler.h"

FFT_handler::FFT_handler(int raw_data_sample_nr, int windows_nr, int *raw_data,float *fft_amps)
{// Constructor
	this->_fft_amps = fft_amps;
	this->_raw_data_sample_nr = raw_data_sample_nr;
	this->_raw_data = raw_data;
	this->_windows_nr = windows_nr;
	this->_window_sample_nr = int(this->_raw_data_sample_nr/this->_windows_nr);

}

void FFT_handler::Windowing(uint8_t window)
{// Weighing factors are computed once before multiple use of FFT
// The weighing function is symetric; half the weighs are recorded
	
	int aux_raw_data[this->_window_sample_nr]={};
	for (int j=0;j<this->_windows_nr;j++){

		float samplesMinusOne = (float(this->_window_sample_nr) - 1.0);
		for (int i=0;i<this->_window_sample_nr;i++){aux_raw_data[i]=this->_raw_data[i+j*this->_window_sample_nr];}

		for (int i = 0; i < (this->_window_sample_nr >> 1); i++) {
			float indexMinusOne = float(i);
			float ratio = (indexMinusOne / samplesMinusOne);
			float weighingFactor = 1.0;
			// Compute and record weighting factor
			switch (window) {
			case FFT_WIN_TYPE_RECTANGLE: // rectangle (box car)
				weighingFactor = 1.0;
				break;
			case FFT_WIN_TYPE_HAMMING: // hamming
				weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
				break;
			case FFT_WIN_TYPE_HANN: // hann
				weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
				break;
			case FFT_WIN_TYPE_TRIANGLE: // triangle (Bartlett)
				#if defined(ESP8266) || defined(ESP32)
				weighingFactor = 1.0 - ((2.0 * fabs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
				#else
				weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
				#endif
				break;
			case FFT_WIN_TYPE_NUTTALL: // nuttall
				weighingFactor = 0.355768 - (0.487396 * (cos(twoPi * ratio))) + (0.144232 * (cos(fourPi * ratio))) - (0.012604 * (cos(sixPi * ratio)));
				break;
			case FFT_WIN_TYPE_BLACKMAN: // blackman
				weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
				break;
			case FFT_WIN_TYPE_BLACKMAN_NUTTALL: // blackman nuttall
				weighingFactor = 0.3635819 - (0.4891775 * (cos(twoPi * ratio))) + (0.1365995 * (cos(fourPi * ratio))) - (0.0106411 * (cos(sixPi * ratio)));
				break;
			case FFT_WIN_TYPE_BLACKMAN_HARRIS: // blackman harris
				weighingFactor = 0.35875 - (0.48829 * (cos(twoPi * ratio))) + (0.14128 * (cos(fourPi * ratio))) - (0.01168 * (cos(sixPi * ratio)));
				break;
			case FFT_WIN_TYPE_FLT_TOP: // flat top
				weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
				break;
			case FFT_WIN_TYPE_WELCH: // welch
				weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
				break;
			}
			aux_raw_data[i] = static_cast<int>(aux_raw_data[i]*weighingFactor);
			aux_raw_data[this->_raw_data_sample_nr - (i + 1)] =static_cast<int>(aux_raw_data[this->_raw_data_sample_nr - (i + 1)]*weighingFactor);

		for (int i=0;i<this->_window_sample_nr;i++){this->_raw_data[i+j*this->_window_sample_nr] = aux_raw_data[i];}
		
		}
	}
}

void FFT_handler::set_fft_output_units(uint8_t units){
	this->_fft_output_units = units;
}

void FFT_handler::set_fft_input_units(uint8_t units){
	this->_fft_input_units = units;
}


void FFT_handler::FFT(byte convert_units,byte normalise_with_window_sample_nr, byte normalise_for_half_window)
{	

	for (int i=0;i<(this->_window_sample_nr>>1);i++){(this->_fft_amps)[i]=0;}
	
	int aux_raw_data[this->_window_sample_nr]={};
	float fft_amps_aux[this->_window_sample_nr>>1]={};

	/* ----------------- FFT ----------------- */
	
	for (int j=0;j<this->_windows_nr;j++){
		for (int i=0;i<this->_window_sample_nr;i++){aux_raw_data[i]=this->_raw_data[i+j*this->_window_sample_nr];}
		aproxFFT_func(aux_raw_data,(this->_window_sample_nr),fft_amps_aux);

		
		/* ----------------- Normalise and convert units ----------------- */
		int window_sample_nr_norm = 1;
		if(normalise_with_window_sample_nr){window_sample_nr_norm = this->_window_sample_nr;}
		int half_window_norm = 1;
		if(normalise_for_half_window){half_window_norm = 2;}
		float coef = 1;
		if (convert_units){
			switch (this->_fft_input_units){
				case FFT_INPUT_NONE:
					coef = 1;
					break;
				case FFT_INPUT_MILI:
					coef = 0.001;
					break;
				case FFT_INPUT_MICRO:
					coef = 0.000001;
					break;
				default:
					break;
			}
		}
		switch (this->_fft_output_units){
			case FFT_OUTPUT_NONE:
				for (int i=0;i<(this->_window_sample_nr>>1);i++){
					float aux_fft_amps = fft_amps_aux[i]/window_sample_nr_norm;
					aux_fft_amps = aux_fft_amps*half_window_norm;
					(this->_fft_amps)[i]+=(aux_fft_amps*coef);
				}
				break;
			case FFT_OUTPUT_DB:
				for (int i=0;i<(this->_window_sample_nr>>1);i++){
					float aux_fft_amps = fft_amps_aux[i]/window_sample_nr_norm;
					aux_fft_amps = aux_fft_amps*half_window_norm;
					(this->_fft_amps)[i]+=(20*log10((aux_fft_amps*coef)));
				}
				break;
			default:
				for (int i=0;i<(this->_window_sample_nr>>1);i++){
					float aux_fft_amps = fft_amps_aux[i]/window_sample_nr_norm;
					aux_fft_amps = aux_fft_amps*half_window_norm;	
					(this->_fft_amps)[i]+=(aux_fft_amps*coef);
				}
				break;
		}		
	}
}