//------------------------------------------------------------------------------------------
//                              Define the desired tasks
/*------ACC------*/
//#define LIS3DH_acc
#ifndef LIS3DH_acc
const int acc_sampling_frequency = 400; //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
#endif
//#define MMA8451_acc
//#define LSM6DSOX_acc
//#define LSM6DSO_acc
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

#ifndef RealTimeData
//------------------------------------------------------------------------------------------
//                              FD - Lookup Tables 
int16_t FD_lookuptable_python_512[512]{
21,-37,-219,-72,-346,6,39,297,313,52,64,-396,-159,98,-74,379,97,-71,-87,-237,35,249,199,0,-136,-255,8,441,-120,-313,69,227,-151,-314,207,232,-95,-59,106,-90,0,160,-377,74,-47,31,438,39,201,-44,-160,-107,-373,-133,-350,68,-156,130,-192,199,-122,145,-184,-98,-120,-202,-38,-474,43,-419,88,-218,6,122,-94,250,13,158,194,-236,244,-172,-14,111,-96,224,-88,116,177,23,308,-139,164,-149,-42,-86,-191,-15,-198,71,-252,238,-282,286,-145,229,-185,265,-131,140,11,-47,67,-345,163,-238,105,274,-40,206,-107,-335,-1,127,-165,103,82,-109,-15,-30,-151,-142,59,50,187,22,-282,-140,188,191,88,6,-210,-34,114,61,311,-21,-218,-120,-328,22,268,261,245,83,4,-268,-136,-218,25,30,-63,201,83,274,62,-224,-114,-475,-10,131,322,346,-108,-38,-194,-231,262,71,215,5,-369,-182,183,204,-116,21,-97,-75,61,-111,61,104,13,-20,146,-164,-353,186,124,-50,217,-139,-93,180,-199,175,-41,-40,-43,-150,136,-242,206,-219,250,-225,89,-358,12,-304,-31,-229,-109,-147,-301,49,-208,104,19,74,320,-11,460,-154,140,123,-158,99,-147,-30,-5,-309,276,-171,255,-11,15,210,-216,167,-202,-55,-77,-157,54,-229,153,-311,131,-342,89,-350,68,-421,134,-341,220,-62,256,106,119,329,-217,-26,-178,-150,214,-60,-136,187,-140,104,345,-91,-308,2,249,-170,-107,226,291,-166,-385,105,77,147,157,-263,-155,119,110,72,43,-244,-22,-111,12,110,109,342,-21,-200,-309,-327,47,-33,291,146,135,114,-134,77,-271,-165,-167,-219,298,136,253,4,-274,-116,-324,147,249,50,94,-276,-299,52,133,272,58,-227,-151,133,109,-216,16,313,100,-337,-95,255,-157,-83,103,-19,83,256,-297,-76,-62,-378,274,-136,278,64,265,144,-8,79,-208,-32,-320,-24,-262,-47,-173,-3,-55,28,105,-73,247,-47,325,0,233,233,-95,182,-301,-27,-118,-413,-41,-354,67,16,10,222,-136,285,26,-26,275,-180,268,-297,92,-85,-108,-15,-307,38,-373,69,-249,71,-251,21,-130,72,-162,157,-75,72,22,103,143,-225,136,-243,-100,27,-212,338,234,-214,-34,-13,-122,-112,68,-147,53,221,32,-198,-128,-16,66,188,-74,-248,43,75,51,119,-184,19,28,-31,286,86,50,29,-243,-159,-294,66,205,153,443,-66,-26,-133,-302,2,-277,192,4,171,201,18,113,-284,-220,-183,-110,362,209,123,-169,-321,-95,55,116,249,-236,-370,57,127,128,-24,44,107,92,-92,
};

int16_t FD_lookupTable_python_1024[1024]={
-9,190,135,-26,-120,9,180,-10,-188,-5,223,107,-126,-353,55,155,214,-170,75,2,162,-240,-193,-97,308,3,-139,-384,74,-4,134,-186,54,123,250,-243,-202,-109,269,-84,-87,-206,189,-97,-98,-282,188,127,284,-280,-109,-136,227,-115,-27,-206,202,-78,-98,-369,219,154,260,-237,0,-108,200,-251,-98,-116,327,70,-98,-288,119,163,205,-134,104,75,160,-327,-187,-70,320,52,-59,-308,70,131,86,-240,43,194,195,-186,-235,-140,279,37,-67,-95,131,177,-19,-260,56,197,135,-118,-178,-160,246,52,-12,-101,183,235,39,-302,-73,144,278,-69,-209,-194,49,-79,-152,-32,172,205,123,-291,-161,65,274,45,-142,-153,151,44,-131,-295,84,256,217,-87,-214,22,162,22,-106,-93,163,180,-152,-370,1,133,218,-45,-131,13,205,-13,-181,-190,82,155,37,-301,-173,95,259,40,18,-25,245,-14,-89,-276,34,107,196,-273,-92,-97,236,25,-64,-34,206,109,117,-360,-78,27,361,-150,-51,-284,264,-46,50,-236,281,215,327,-216,-155,-182,268,-90,46,-255,107,-30,120,-328,-13,39,372,60,-57,-246,91,4,145,-236,-50,45,183,-130,-234,-103,266,218,117,-114,-68,97,70,-7,-173,17,138,198,-207,-198,-175,269,195,251,-242,-50,-50,265,-274,-173,-116,308,6,55,-346,57,11,369,-66,-54,-129,191,-62,-65,-263,-60,185,205,-99,-232,-188,96,261,58,-86,-118,134,11,-57,-393,-13,16,387,-36,-8,-278,181,41,292,-121,75,22,249,-66,-184,-312,57,294,241,-53,-269,-118,39,271,-40,-103,-139,221,8,-57,-347,-77,-24,412,-46,-90,-309,29,69,226,-68,-54,30,161,-11,-281,-339,-42,217,199,102,-228,-37,71,345,21,69,-140,187,106,107,-402,-274,-164,307,209,-17,-176,-126,111,119,136,-158,41,64,216,-121,-142,-344,71,118,368,-86,-93,-241,85,162,41,-142,-23,28,151,144,-317,-210,-228,293,108,178,-331,-12,-37,202,168,13,-111,47,189,9,-93,-449,17,11,396,11,-24,-393,9,47,302,-38,-95,-75,97,170,-33,-120,-338,79,106,334,-105,-84,-276,133,207,202,48,-169,47,-14,173,-172,-82,-302,176,119,291,-138,-103,-196,66,268,177,50,-200,-19,-48,141,-194,-57,-250,171,110,42,-90,-249,-85,160,340,47,78,-326,-11,-97,243,-65,-49,-96,-24,156,0,-26,-355,15,39,460,122,70,-399,-94,-33,147,97,-41,-69,-229,155,13,56,-340,-65,8,400,220,-41,-166,-247,95,57,335,-37,-89,-284,41,21,93,-38,-200,112,44,356,-79,66,-383,2,53,286,124,-228,-190,-339,201,130,325,-137,10,-59,179,123,19,-121,-330,104,73,307,-98,-125,-414,-72,53,220,296,-126,27,-161,197,10,132,-105,-29,-76,194,186,9,-181,-363,98,153,500,79,-116,-174,-91,173,174,236,-159,-10,-229,74,-37,26,-156,-197,56,32,329,-46,-15,-316,-21,84,318,244,-126,-187,-311,19,24,220,-101,-68,-101,88,181,94,62,-380,29,-54,316,109,4,-358,-248,13,5,296,0,117,-358,3,14,177,115,-80,-102,-107,218,101,81,-300,-195,-112,112,330,112,44,-307,-20,-72,304,236,112,-58,-258,53,-76,193,-121,-144,-221,106,185,82,47,-266,-102,-204,339,289,280,-96,-321,-156,-169,236,-13,134,-238,-72,-148,237,168,-34,-53,-236,284,91,299,-108,-154,-292,-96,193,143,304,-190,-261,-269,106,142,150,151,-106,-40,0,328,19,49,-222,-151,29,75,283,-163,-110,-436,128,106,348,182,-107,-157,-172,177,28,292,-49,-75,-144,-44,131,-28,-10,-258,-55,-72,265,219,97,-244,-329,89,15,429,15,-1,-355,-140,-85,90,101,-74,37,-163,121,73,101,-14,-126,-42,27,385,52,153,-338,-188,-225,124,225,61,59,-265,4,-110,251,-17,-21,-191,-65,166,62,140,-295,-225,-269,96,185,105,31,-256,-141,-149,345,194,274,-107,-89,-126,-8,177,0,-50,-252,77,139,290,-38,-216,-154,-105,255,175,287,-153,-208,-321,-41,159,101,65,-258,-70,-155,226,-41,81,-151,6,179,231,260,-171,-123,-366,78,75,279,-26,-209,-191,-184,238,31,203,-207,48,-44,218,156,-23,-170,-177,35,22,310,-133,-90,-356,-53,158,226,226,-17,-40,-141,170,71,143,-193,-166,-12,74,228,-133,-195,-372,64,89,405,-4,-129,-197,-133,125,96,229,-198,-89,-224,252,81,56,-162,-149,68,181,382,-26,-157,-332,81,138,264,101,-238,-173,-203,174,33,141,-258,53,0,270,72,-69,-168,-137,247,171,248,-198,-122,-259,170,64,204,-17,-115,48,115,288,-115,-31,-285,201,125,255,-107,-200,-375,-89,188,167,101,-249,27,-31,259,52,39,-258,29,238,202,-25,-344,-211,-124,311,37,150,-234,3,-104,177,132,-1,-175,79,162,13,18,-378,-149,-188,375,157,119,-260,-89,-25,244,235,-11,-90,-63,276,-118,-27,-402,-26,-64,355,98,-5,-283,-160,199,259,272,-69,8,-179,159,-63,-44,-354,60,42,292,-31,-176,-205,-88,236,143,69,-212,35,-64,141,-63,-12,-280,153,159,142,-225,-198,-153,209,356,93,-56,-292,131,0,130,-318,-74,
};
//------------------------------------------------------------------------------------------
//                              TD - Lookup Tables

#endif
//------------------------------------------------------------------------------------------
//                               ACC

byte received_data[6];

int acc_sample_count = 0;

#ifdef LIS3DH_acc
#include "SparkFunLIS3DH.h"
#define I2C_COM
#include <Wire.h>
LIS3DH lis3dh_obj(I2C_MODE, 0x18); //Default constructor is I2C, addr 0x18.
const int acc_sampling_frequency = 400; //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
const uint8_t acc_scale = 8;  //Max G force readable.  Can be: 2, 4, 8, 16
const uint8_t acc_fifo_wtm_val = 25;  
#endif
#ifdef MMA8451_acc
#include "MMA8451.h"
MMA8451 mma8451_obj = MMA8451(received_data);
const int acc_sampling_frequency = 800; //This value can be changed to the desired sampling frequency
const uint8_t acc_fifo_wtm_val = 30;
const uint8_t acc_scale = 8;
#endif
#ifdef LSM6DSOX_acc
#include "LSM6DSOXSensor.h"
#include <Wire.h>
LSM6DSOXSensor AccGyr(&Wire, LSM6DSOX_I2C_ADD_L);
const int acc_sampling_frequency = 1667; //This value can be changed to the desired sampling frequency
const int acc_fifo_wtm_val = 256;
#define INT1_pin 5
#endif
#ifdef LSM6DSO_acc
#include "LSM6DSO.h"
LSM6DSO lsm6dso_obj(received_data);
const int acc_sampling_frequency = 8000; //This value can be changed to the desired sampling frequency
const uint16_t acc_fifo_wtm_val = 500;
const int SS_pin = 10;
#endif

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