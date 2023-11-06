#include "peakfinder1_peak.h"

PeakFinder1::PeakFinder1(float samplingFrequency,int sample_nr,int nr_peaks,float *peaks_value,float *peaks_amps,float *signal_amps,float *abcissa){
	this->_samplingFrequency = samplingFrequency;
	this->_sample_nr = sample_nr;
	this->_nr_peaks = nr_peaks;
	this->_peaks_value = peaks_value;
	this->_peaks_amps = peaks_amps;
	this->_signal_amps = signal_amps;
	this->_abcissa = abcissa;
}

void PeakFinder1::get_peaks_made_algorithm1(float peak_amp_threshold,float value_between_peaks){

	for (int aux_i=0;aux_i<(this->_nr_peaks);aux_i++){
		this->_peaks_value[aux_i]=-2;
		this->_peaks_amps[aux_i]=-2;
	}

	int flag_peak=0;
	float amplitude;

	for (int j=0;j<(this->_nr_peaks);j++){
		amplitude=0;

		if (flag_peak==2){this->_peaks_value[j]=-1;this->_peaks_amps[j]=-1;continue;}
		// It is only possible to iterate until the penultimate element of the array becaue the last element is not compared with the next one
		for(int i=0;i<(this->_sample_nr>>1)-1;i++){
			// if the frequency has a peak
			if (amplitude<this->_signal_amps[i] && this->_signal_amps[i]>this->_signal_amps[i+1]){ 
				// compare the peak with the previous peaks
				for (int aux_i=0;aux_i<(this->_nr_peaks);aux_i++){
					// if the peak is already in the list of peaks
					if (this->_peaks_value[aux_i]==this->_abcissa[i]){flag_peak=0;break;}
					// if the peak is too close to the previous peak
					else if(abs(this->_abcissa[i]-this->_peaks_value[aux_i])<value_between_peaks && this->_peaks_value[aux_i]>=0){flag_peak=0;break;}
					// if the peak is too low compared to the previous peak: 
					flag_peak=1;
				}
				if (flag_peak==1){this->_peaks_value[j]=this->_abcissa[i];this->_peaks_amps[j]=this->_signal_amps[i];amplitude=this->_signal_amps[i];}
				if (flag_peak==2){this->_peaks_value[j]=-1;this->_peaks_amps[j]=-1;break;}
			}
		}
		if(j>0){if((peak_amp_threshold)>(this->_peaks_amps[j]/(this->_peaks_amps[j-1])) && this->_peaks_amps[j-1]>0 && this->_peaks_value[j-1]>=0){flag_peak=2;break;}}
	}
}