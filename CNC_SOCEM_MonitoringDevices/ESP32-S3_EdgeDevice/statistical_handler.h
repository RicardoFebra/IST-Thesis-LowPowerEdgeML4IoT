#include <Arduino.h>
#include <math.h>

#define StatisticalMean 0x00
#define StatisticalStdDev 0x01
#define StatisticalKurtosis 0x02
#define StatisticalSkewness 0x03
#define StatisticalRootMeanSquare 0x04


class StatisticalHandler{
    private:
        float *_kurtosis_amps;
        int _windows_nr;
        int _raw_data_sample_nr;
        int *_raw_data;
        int _window_sample_nr;
        

    public:
        StatisticalHandler(int raw_data_sample_nr, int windows_nr, int *raw_data);
        ~StatisticalHandler();
        float MakeAnalysis(byte analysis_type);
        float Mean(int *raw_data, int sample_nr);
        float StdDev(int *raw_data, int sample_nr);
        float Kurtosis(int *raw_data, int sample_nr);
        float Skewness(int *raw_data, int sample_nr);
        float RootMeanSquare(int *raw_data, int sample_nr);
        float MaxAbsoluteValue(int *raw_data, int sample_nr);

};