#include "peak_handler.h"

PeakHandler::PeakHandler(float samplingFrequency, int nr_freq_peaks, int signal_sample_nr, float *peaks_value, float *peaks_amps, float *signal_amplitude, float *signal_abcissa){
    this->_samplingFrequency = samplingFrequency;
    this->_nr_peaks = nr_freq_peaks;
    this->_peaks_value = peaks_value;
    this->_peaks_amps = peaks_amps;
    this->_signal_amplitude = signal_amplitude;
    this->_signal_abcissa = signal_abcissa;
    this->_signal_sample_nr = signal_sample_nr;
}

void PeakHandler::PEAKS(){
    this->get_peaks_zscore();
}

/*
void PeakHandler::get_peaks_made_algorithm1(float peak_amp_threshold,float value_between_peaks){

    PeakFinder1 peakfinder1(this->_samplingFrequency,this->_signal_sample_nr,this->_nr_peaks,this->_peaks_value,this->_peaks_amps,this->_signal_amplitude,this->_signal_abcissa);
    peakfinder1.get_peaks_made_algorithm1(peak_amp_threshold,value_between_peaks);
}
*/

void PeakHandler::get_peaks_zscore(){

    ZScorePeak peakDetection;
	peakDetection.begin(int(this->_signal_sample_nr/4), 5, 0.8);  // sets the lag, threshold and influence

    uint16_t zscore_peak_vector[this->_signal_sample_nr]={};
    double zscore_filtered_vector[this->_signal_sample_nr]={};
    
    #ifdef PEAK_HANDLER_INFO
        Serial.println("Zscore algorithm done");
    #endif
    for (int i = 0; i<(this->_signal_sample_nr); i++) {
        peakDetection.add(this->_signal_amplitude[i]);
        zscore_peak_vector[i] = peakDetection.getPeak();
        zscore_filtered_vector[i] = peakDetection.getFilt();
    #ifdef PEAK_HANDLER_DEBUG
        Serial.print(this->_signal_abcissa[i]);
        Serial.print(" ");
        Serial.print(this->_signal_amplitude[i]);
        Serial.print(" ");
        Serial.print(zscore_peak_vector[i]);
        Serial.print(" ");
        Serial.println(zscore_filtered_vector[i]);
    #endif
    }

    uint16_t zscore_peak_counter = 0;
    // determine the peaks found by the zscore algorithm
    for (int i = 0; i<(this->_signal_sample_nr); i++) {
        if (zscore_peak_vector[i]==1){
            zscore_peak_counter++;
        }
    }

    float zscore_peaks_value[zscore_peak_counter]={};
    float zscore_peaks_amps[zscore_peak_counter]={};

    #ifdef PEAK_HANDLER_INFO
        Serial.println("Find peaks from zscore algorithm");
        Serial.println("[APP] Free memory: " + String(esp_get_free_heap_size()) + " bytes");
    #endif
    int zscore_peak_counter_aux = 0;
    for (int i = 0; i<(this->_signal_sample_nr); i++) {
        int aux_j = 0;
        if (i==0 and zscore_peak_vector[i]==1){
            int max_amp = 0;
            int max_idx = 0;
            for (int j=i; j<(this->_signal_sample_nr-1); j++){
                if (this->_signal_amplitude[j]>max_amp){
                    max_amp = this->_signal_amplitude[j];
                    max_idx = j;
                }
                if ((zscore_peak_vector[j]==1 and zscore_peak_vector[j+1]==0)){
                    zscore_peaks_value[zscore_peak_counter_aux]=this->_signal_abcissa[max_idx];
                    zscore_peaks_amps[zscore_peak_counter_aux]=this->_signal_amplitude[max_idx];
                    #ifdef PEAK_HANDLER_DEBUG
                        Serial.print("peak found: ");Serial.print(zscore_peaks_value[zscore_peak_counter_aux]);Serial.print(" ");Serial.print(zscore_peaks_amps[zscore_peak_counter_aux]);Serial.print(" ");Serial.print("Index: ");Serial.println(max_idx);
                    #endif
                    zscore_peak_counter_aux++;      
                    aux_j = j;            
                    break;
                }
            }
            i = aux_j;
        }else if ((zscore_peak_vector[i]==1 and zscore_peak_vector[i-1]==0)){
            int max_amp = 0;
            int max_idx = 0;
            for (int j=i; j<(this->_signal_sample_nr-1); j++){
                if (this->_signal_amplitude[j]>max_amp){
                    max_amp = this->_signal_amplitude[j];
                    max_idx = j;
                }
                if ((zscore_peak_vector[j]==1 and zscore_peak_vector[j+1]==0)){
                    zscore_peaks_value[zscore_peak_counter_aux]=this->_signal_abcissa[max_idx];
                    zscore_peaks_amps[zscore_peak_counter_aux]=this->_signal_amplitude[max_idx];
                    #ifdef PEAK_HANDLER_DEBUG
                        Serial.print("peak found: ");Serial.print(zscore_peaks_value[zscore_peak_counter_aux]);Serial.print(" ");Serial.print(zscore_peaks_amps[zscore_peak_counter_aux]);Serial.print(" ");Serial.print("Index: ");Serial.println(max_idx);
                    #endif
                    zscore_peak_counter_aux++;  
                    aux_j = j;                
                    break;
                }
            }
            i = aux_j;
        }
    }
    for(int i = zscore_peak_counter_aux; i< zscore_peak_counter; i++){
        zscore_peaks_value[i]=-1;
        zscore_peaks_amps[i]=-1;
    }

    #ifdef PEAK_HANDLER_DEBUG
        for (int i = 0; i<zscore_peak_counter;i++){
            Serial.print(zscore_peaks_value[i]);Serial.print(" ");Serial.println(zscore_peaks_amps[i]);
        }
    #endif

    #ifdef PEAK_HANDLER_INFO
        Serial.println("Sort peaks from zscore algorithm");
    #endif

    // select only the highest peaks and only the first nr_peaks peaks
    for (int i = 0; i<this->_nr_peaks;i++){
        if (i<zscore_peak_counter_aux){
            float max_peak_amp = 0;
            int max_peak_idx = 0;
            for (int j = 0; j<zscore_peak_counter_aux;j++){
                if (zscore_peaks_amps[j]>max_peak_amp){
                    max_peak_amp = zscore_peaks_amps[j];
                    max_peak_idx = j;
                }
            }
            this->_peaks_value[i] = zscore_peaks_value[max_peak_idx];
            this->_peaks_amps[i] = zscore_peaks_amps[max_peak_idx];
            zscore_peaks_amps[max_peak_idx] = 0;
        }else{
            this->_peaks_value[i] = -1;
            this->_peaks_amps[i] = -1;
        }

    }
}