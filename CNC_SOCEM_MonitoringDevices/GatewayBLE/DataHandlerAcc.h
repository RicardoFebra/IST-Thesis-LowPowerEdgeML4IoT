//------------------------------------------------------------------------------------------
//                              Define the desired tasks
/*------ACC------*/
//#define LIS3DH_acc
#ifndef LIS3DH_acc
const int acc_sampling_frequency = 400; //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
#endif
/*------Data------*/
#define RealTimeData
/*------Analysis------*/
#define DefineAnalysisVariables
//#define FD_Domain
//#define TD_StatAnalysis
//#define FD_FindPeaks
#ifdef DefineAnalysisVariables
  #define FD_DomainVariables
  #define TD_StatAnalysisVariables
  #define FD_FindPeaksVariables
#endif

//------------------------------------------------------------------------------------------
//                              Time Domain

const uint16_t acc_window_sample_nr = 1024; //This value MUST ALWAYS be a power of 2
const uint16_t acc_windows_nr = 1; //This value can be changed to increase the number of windows to make an average of

const int acc_sample_nr = acc_window_sample_nr*acc_windows_nr;

// Larger acc buffer
int acc_data_x_axis[acc_sample_nr];
int acc_data_y_axis[acc_sample_nr];
int acc_data_z_axis[acc_sample_nr];

int TD_wireless_channel_offset = 0;

//------------------------------------------------------------------------------------------
//                               ACC

byte received_data[6];

int acc_sample_count = 0;

volatile bool acc_fifo_wtm_reached = false;

void acc_fifo_wtm_interrupt(){
  acc_fifo_wtm_reached = true;
}

//------------------------------------------------------------------------------------------
//                              Frequency Domain - FFT

#ifdef FD_DomainVariables

const int acc_fft_sample_nr = acc_window_sample_nr>>1; 
// acc data fft
float acc_fft_x_axis[acc_fft_sample_nr]={};
float acc_fft_y_axis[acc_fft_sample_nr]={};
float acc_fft_z_axis[acc_fft_sample_nr]={};

float acc_freq[acc_fft_sample_nr]={};

#endif
#ifdef FD_Domain

#include "fft_handler.h"

FFT_handler acc_fft_handler_obj_x_axis(acc_sample_nr,acc_windows_nr,acc_data_x_axis,acc_fft_x_axis);
FFT_handler acc_fft_handler_obj_y_axis(acc_sample_nr,acc_windows_nr,acc_data_y_axis,acc_fft_y_axis);
FFT_handler acc_fft_handler_obj_z_axis(acc_sample_nr,acc_windows_nr,acc_data_z_axis,acc_fft_z_axis);

int FD_wireless_channel_offset = 20;
#endif

//------------------------------------------------------------------------------------------
//                              Frequency Domain - Peaks

#ifdef FD_FindPeaksVariables

const int acc_nr_freq_peaks = 4;

float FD_acc_data_peaks_freq_scalar = 1;
float FD_acc_data_peaks_amp_scalar = 10;

float FD_acc_data_peaks_freq_x_axis[acc_nr_freq_peaks]={};
float FD_acc_data_peaks_amps_x_axis[acc_nr_freq_peaks]={};

float FD_acc_data_peaks_freq_y_axis[acc_nr_freq_peaks]={};
float FD_acc_data_peaks_amps_y_axis[acc_nr_freq_peaks]={};

float FD_acc_data_peaks_freq_z_axis[acc_nr_freq_peaks]={};
float FD_acc_data_peaks_amps_z_axis[acc_nr_freq_peaks]={};

float acc_sampling_frequency_float = acc_sampling_frequency;

#endif
#ifdef FD_FindPeaks

// if the peak finder routine is zscore, the convert_units parameter from FFT_handler is advised to be set true or the FFT units to dB, otherwise it will not detect large peak differences
#include "peak_handler.h"

PeakHandler acc_peak_handler_obj_x_axis(acc_sampling_frequency_float,acc_nr_freq_peaks, acc_fft_sample_nr,FD_acc_data_peaks_freq_x_axis,FD_acc_data_peaks_amps_x_axis,acc_fft_x_axis,acc_freq);
PeakHandler acc_peak_handler_obj_y_axis(acc_sampling_frequency_float,acc_nr_freq_peaks, acc_fft_sample_nr,FD_acc_data_peaks_freq_y_axis,FD_acc_data_peaks_amps_y_axis,acc_fft_y_axis,acc_freq);
PeakHandler acc_peak_handler_obj_z_axis(acc_sampling_frequency_float,acc_nr_freq_peaks, acc_fft_sample_nr,FD_acc_data_peaks_freq_z_axis,FD_acc_data_peaks_amps_z_axis,acc_fft_z_axis,acc_freq);
#endif

//------------------------------------------------------------------------------------------
//                              Time Domain - Statistical Analysis

#ifdef TD_StatAnalysisVariables

float TD_acc_mean_x_axis = 0;
float TD_acc_std_dev_x_axis = 0;
float TD_acc_kurtosis_x_axis = 0;
float TD_acc_skewness_x_axis = 0;
float TD_acc_rms_x_axis = 0;


float TD_acc_mean_y_axis = 0;
float TD_acc_std_dev_y_axis = 0;
float TD_acc_kurtosis_y_axis = 0;
float TD_acc_skewness_y_axis = 0;
float TD_acc_rms_y_axis = 0;

float TD_acc_mean_z_axis = 0;
float TD_acc_std_dev_z_axis = 0;
float TD_acc_kurtosis_z_axis = 0;
float TD_acc_skewness_z_axis = 0;
float TD_acc_rms_z_axis = 0;

float TD_acc_mean_scalar = 100;
float TD_acc_std_dev_scalar = 100;
float TD_acc_kurtosis_scalar = 100;
float TD_acc_skewness_scalar = 100;
float TD_acc_rms_scalar = 100;


#endif
#ifdef TD_StatAnalysis

#include "statistical_handler.h"

StatisticalHandler TD_acc_statistical_handler_obj_x_axis(acc_sample_nr,acc_windows_nr,acc_data_x_axis);
StatisticalHandler TD_acc_statistical_handler_obj_y_axis(acc_sample_nr,acc_windows_nr,acc_data_y_axis);
StatisticalHandler TD_acc_statistical_handler_obj_z_axis(acc_sample_nr,acc_windows_nr,acc_data_z_axis);

#endif


//------------------------------------------------------------------------------------------
//                              Print Data Code Blocks
/*------FFT------*/
/*
for (int i=0;i<(acc_fft_sample_nr);i++){
  int peak_z = 0;
  int amp_z = 0;
  for (int j=0;j<acc_nr_freq_peaks;j++){
    if (acc_freq[i]==FD_acc_data_peaks_freq_z_axis[j]){
      peak_z = 1;
      amp_z = FD_acc_data_peaks_amps_z_axis[j];
      break;
    }
  }
  Serial.print(acc_freq[i]);
  Serial.print(" ");
  Serial.print(acc_data_x_axis[i]);
  Serial.print(" ");
  Serial.print(acc_data_y_axis[i]);
  Serial.print(" ");
  Serial.print(acc_data_z_axis[i]);
  Serial.print(" ");
  Serial.print(acc_fft_x_axis[i]);
  Serial.print(" ");
  Serial.print(acc_fft_y_axis[i]);
  Serial.print(" ");
  Serial.print(peak_z);
  Serial.print(" ");
  Serial.print(amp_z);
  Serial.print(" ");
  Serial.println(acc_fft_z_axis[i]);
}
}
*/