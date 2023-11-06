#include <Arduino.h>
#include "zscore_peak.h"

//#define PEAK_HANDLER_DEBUG
#ifdef PEAK_HANDLER_DEBUG
    #define PEAK_HANDLER_INFO
#endif

/* Peak finder routines */
#define ROUTINE_ZSCORE 0x00
#define ROUTINE_PEAKFINDER1 0x01

/* Objective of Peaks*/
#define OBJECTIVE_PRINT_PEAKS 0x00
#define OBJECTIVE_DETERMINE_PEAKS 0x01
#define OBJECTIVE_PRINT_PEAKS_IN_INPUT_SIGNAL 0x02
#define OBJECTIVE_SEND_PEAKS_LORA 0x03

class PeakHandler{

    public: 
        PeakHandler(float samplingFrequency, int nr_freq_peaks, int signal_samples_nr, float *peaks_value, float *peaks_amps, float *signal_amplitude, float *signal_abcissa);
        void PEAKS();
        void get_peaks_zscore();
        void get_peaks_made_algorithm1(float peak_amp_threshold,float value_between_peaks);
    private:
        float _samplingFrequency;
        int _signal_sample_nr;
        int _nr_peaks;
        float *_peaks_value;
        float *_peaks_amps;
        float *_signal_amplitude;
        float *_signal_abcissa;
        float _signal_amplitude_threshold;
};