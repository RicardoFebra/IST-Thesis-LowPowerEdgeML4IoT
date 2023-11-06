#include "statistical_handler.h"

/**
 * @brief Construct a new Statistical Handler:: Statistical Handler object
 * 
 * @param raw_data_sample_nr 
 * @param windows_nr 
 * @param raw_data 
 */
StatisticalHandler::StatisticalHandler(int raw_data_sample_nr, int windows_nr, int *raw_data)
{// Constructor
    this->_raw_data_sample_nr = raw_data_sample_nr;
    this->_raw_data = raw_data;
    this->_windows_nr = windows_nr;
    this->_window_sample_nr = int(this->_raw_data_sample_nr/this->_windows_nr);

}

StatisticalHandler::~StatisticalHandler()
{// Destructor
}

float StatisticalHandler::MakeAnalysis(byte analysis_method){
    
  float result = 0;
    
	int aux_raw_data[this->_window_sample_nr]={};

	for (int j=0;j<this->_windows_nr;j++){
		for (int i=0;i<this->_window_sample_nr;i++){aux_raw_data[i]=this->_raw_data[i+j*this->_window_sample_nr];}
    switch (analysis_method){
        case StatisticalMean:
            result += this->Mean(aux_raw_data, this->_window_sample_nr);
            break;
        case StatisticalStdDev:
            result += this->StdDev(aux_raw_data, this->_window_sample_nr);
            break;
        case StatisticalKurtosis:
            result += this->Kurtosis(aux_raw_data, this->_window_sample_nr);
            break;
        case StatisticalSkewness:
            result += this->Skewness(aux_raw_data, this->_window_sample_nr);
            break;
        case StatisticalRootMeanSquare:
            result += (this->RootMeanSquare(aux_raw_data, this->_window_sample_nr));
            break;
        default:
            return 0;
            break;
        }
    }
    result = result/this->_windows_nr;
    
    return result;
}

float StatisticalHandler::Mean(int *raw_data, int sample_nr){
    float mean = 0;
    for (int i=0;i<sample_nr;i++){mean += raw_data[i];}
    mean = mean/sample_nr;

    return mean;
}

float StatisticalHandler::StdDev(int *raw_data, int sample_nr){
    float mean = 0;
    float std_dev = 0;

    mean = this->Mean(raw_data, sample_nr);

    for (int i=0;i<sample_nr;i++){std_dev += (raw_data[i]-mean)*(raw_data[i]-mean);}
    std_dev = std_dev/(sample_nr-1);
    std_dev = sqrt(std_dev);

    return std_dev;
}

float StatisticalHandler::Kurtosis(int *raw_data, int sample_nr){
    float mean = 0;
    float std_dev = 0;
    float kurtosis = 0;

    mean = this->Mean(raw_data, sample_nr);
    std_dev = this->StdDev(raw_data, sample_nr);
    
    kurtosis = 0;
    for (int i=0;i<sample_nr;i++){
        kurtosis += (raw_data[i]-mean)*(raw_data[i]-mean)*(raw_data[i]-mean)*(raw_data[i]-mean);
    }

    kurtosis = kurtosis/(sample_nr*std_dev*std_dev*std_dev*std_dev);

    return kurtosis;
}

float StatisticalHandler::Skewness(int *raw_data, int sample_nr){
    float mean = 0;
    float std_dev = 0;
    float skewness = 0;

    mean = this->Mean(raw_data, sample_nr);
    std_dev = this->StdDev(raw_data, sample_nr);
    
    skewness = 0;
    for (int i=0;i<sample_nr;i++){
        skewness += (raw_data[i]-mean)*(raw_data[i]-mean)*(raw_data[i]-mean);
    }
    skewness = skewness/(sample_nr*std_dev*std_dev*std_dev);

    return skewness;

}

float StatisticalHandler::RootMeanSquare(int *raw_data, int sample_nr){
    float rms = 0;
    // divide raw_data according to max value
    float max_abs_value = this->MaxAbsoluteValue(raw_data, sample_nr);
    // continue with ints or floats, compromise in time vs precision
    float aux_raw_data[sample_nr];
		//int aux_raw_data[sample_nr];
    
    for (int i=0;i<sample_nr;i++){aux_raw_data[i] = raw_data[i]/(max_abs_value);}
    // calculate rms
    for (int i=0;i<sample_nr;i++){rms += aux_raw_data[i]*aux_raw_data[i];}
    rms = rms/sample_nr;
    rms = sqrt(rms);
    rms = rms*(max_abs_value);

    return rms;
}

float StatisticalHandler::MaxAbsoluteValue(int *raw_data, int sample_nr){
    float max_abs_value = 0;
    for (int i=0;i<sample_nr;i++){
        if (abs(raw_data[i])>max_abs_value){max_abs_value = abs(raw_data[i]);}
    }

    return max_abs_value;
}