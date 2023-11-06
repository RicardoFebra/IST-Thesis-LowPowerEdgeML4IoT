
class PeakFinder1{
    public:
        PeakFinder1(float samplingFrequency,int sample_nr,int nr_peaks,float *peaks_value,float *peaks_amps,float *signal_amps,float *abcissa);
        void get_peaks_made_algorithm1(float peak_amp_threshold,float value_between_peaks);
    private:
        float _samplingFrequency;
        int _sample_nr;
        int _nr_peaks;
        float *_peaks_value;
        float *_peaks_amps;
        float *_signal_amps;
        float *_abcissa;
};