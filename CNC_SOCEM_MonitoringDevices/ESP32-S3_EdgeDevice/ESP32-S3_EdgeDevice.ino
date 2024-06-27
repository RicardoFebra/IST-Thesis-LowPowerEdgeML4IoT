#include <Arduino.h>

//------------------------------------------------------------------------------------------
//                              ESP32-S3 Init and LowPower

#include "ESP32-S3.h"

bool sleep_flag = true;
//------------------------------------------------------------------------------------------
//                               RTC

bool isClock = false;

//------------------------------------------------------------------------------------------
//                               BLE
#define ARDUINO_BLE
//#define ESP32_BLE
#include "DataHandlerBLE.h"

//------------------------------------------------------------------------------------------
//                              Data
#include "DataHandlerAcc.h"

#if !(USING_DEFAULT_ARDUINO_LOOP_STACK_SIZE)
uint16_t USER_CONFIG_ARDUINO_LOOP_STACK_SIZE = 32768;
#endif

SET_LOOP_TASK_STACK_SIZE(32 * 1024);

//uint8_t SS_pin = 10;
uint8_t SS_pin = 4;
uint8_t SCK_pin = 3;
uint8_t MISO_pin = 39;
uint8_t MOSI_pin = 38;

#define TWDT_TIMEOUT_MS         3000

const esp_task_wdt_config_t twdt_config = {
    .timeout_ms = TWDT_TIMEOUT_MS,
    .idle_core_mask = (1 << 2) - 1,    // Bitmask of all cores
    .trigger_panic = false,
};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  /*------------------Set WDT----------------------------*/
  // make sure we don't get killed for our long running tasks
  esp_task_wdt_init(&twdt_config);

  /*------------------Set BLE----------------------------*/
  // No need to start BLE here
  //ble_server_init(7);

  /*------------------Set SPI----------------------------*/
  SPI.begin();
  //SPI.begin(SCK_pin,MISO_pin, MOSI_pin, SS_pin);
  
  delay(1000); //relax...
  //Serial.println("Processor came out of reset.\n");

  /*------------------Set ACC----------------------------*/
  Serial.println("Start acc configuration");
  lsm6dso_obj.begin(SPI_com, SS_pin);
  lsm6dso_obj.set_mode_1(acc_fifo_wtm_val);

  /*------------------Set FFT----------------------------*/
  Serial.println("Start fft configuration");
  for (int i = 0; i < (acc_fft_sample_nr); i++) { acc_freq[i] = (acc_sampling_frequency * i) / (acc_window_sample_nr); };

  acc_fft_handler_obj_x_axis.set_fft_output_units(FFT_OUTPUT_NONE);
  acc_fft_handler_obj_x_axis.set_fft_input_units(FFT_INPUT_MILI);

  acc_fft_handler_obj_y_axis.set_fft_output_units(FFT_OUTPUT_NONE);
  acc_fft_handler_obj_y_axis.set_fft_input_units(FFT_INPUT_MILI);

  acc_fft_handler_obj_z_axis.set_fft_output_units(FFT_OUTPUT_NONE);
  acc_fft_handler_obj_z_axis.set_fft_input_units(FFT_INPUT_NONE);
}

void loop() {

  lsm6dso_obj.get_FIFO_STATUS();
  //Serial.print("fifo_full: ");Serial.println(lsm6dso_obj.check_FIFO_FULL());
  //Serial.print("fifo_ovr: ");Serial.println(lsm6dso_obj.check_FIFO_OVR());
  //Serial.print("fifo_wtm_status: "); Serial.println(lsm6dso_obj.check_FIFO_WTM());
  //Serial.print("fifo_wtm_unread_samples: "); Serial.println(lsm6dso_obj.check_FIFO_unread_samples());

  if (lsm6dso_obj.check_FIFO_WTM()) {
    //Serial.print("WTM Level reached! - Timestamp: ");Serial.println(millis());
    for (int i = 0; i < acc_fifo_wtm_val; i++) {
      int16_t accelerometer[3];
      lsm6dso_obj.get_FIFO_DATA_OUT(accelerometer);
      acc_data_x_axis[acc_sample_count] = accelerometer[0];
      acc_data_y_axis[acc_sample_count] = accelerometer[1];
      acc_data_z_axis[acc_sample_count++] = accelerometer[2];
      if (acc_sample_count == acc_sample_nr) {
        break;
      }
    }
  }
  if (acc_sample_count == acc_sample_nr) {
    acc_sample_count = 0;
    lsm6dso_obj.set_sleep_mode();

    /*------------------Start TD Stat Analysis----------------------------*/
    td_statistical_analysis();

    /*------------------Start FD Domain----------------------------*/
    Serial.println("Starting FFT");

    acc_fft_handler_obj_x_axis.Windowing(FFT_WIN_TYPE_HANN);
    acc_fft_handler_obj_x_axis.FFT(true, true, true);  // if peak finder routine is zscore, then the first parameter must be true
    acc_peak_handler_obj_x_axis.PEAKS();

    acc_fft_handler_obj_y_axis.Windowing(FFT_WIN_TYPE_HANN);
    acc_fft_handler_obj_y_axis.FFT(true, true, true);  // if peak finder routine is zscore, then the first parameter must be true
    acc_peak_handler_obj_y_axis.PEAKS();

    acc_fft_handler_obj_z_axis.Windowing(FFT_WIN_TYPE_HANN);
    acc_fft_handler_obj_z_axis.FFT(true, true, true);  // if peak finder routine is zscore, then the first parameter must be true
    acc_peak_handler_obj_z_axis.PEAKS();

    uint32_t frequency = 0;
    uint32_t amplitude = 0;
    Serial.println("Starting Freq Domain Analysis");
    for(uint8_t array_idx = 0;array_idx<acc_nr_freq_peaks;array_idx++) {
      int aux_array_idx = (array_idx*2);
      frequency = static_cast<uint32_t>(FD_acc_data_peaks_freq_x_axis[array_idx] + 0.5);
      amplitude = static_cast<uint32_t>((FD_acc_data_peaks_amps_x_axis[array_idx]*FD_acc_data_peaks_amp_scalar) + 0.5);
      //Serial.print("x-axis: ");Serial.print(frequency);Serial.print(" ");Serial.print(amplitude);Serial.print("; ");
      ble_encode_uint32(frequency,4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer);
      ble_encode_uint32(amplitude,4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer);
      frequency = static_cast<uint32_t>(FD_acc_data_peaks_freq_y_axis[array_idx] + 0.5);
      amplitude = static_cast<uint32_t>((FD_acc_data_peaks_amps_y_axis[array_idx]*FD_acc_data_peaks_amp_scalar) + 0.5);
      //Serial.print("y-axis: ");Serial.print(frequency);Serial.print(" ");Serial.print(amplitude);Serial.print("; ");
      ble_encode_uint32(frequency,4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer);
      ble_encode_uint32(amplitude,4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer);
      frequency = static_cast<uint32_t>(FD_acc_data_peaks_freq_z_axis[array_idx] + 0.5);
      amplitude = static_cast<uint32_t>((FD_acc_data_peaks_amps_z_axis[array_idx]*FD_acc_data_peaks_amp_scalar) + 0.5);
      //Serial.print("z-axis: ");Serial.print(frequency);Serial.print(" ");Serial.println(amplitude);
      ble_encode_uint32(frequency,4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer);
      ble_encode_uint32(amplitude,4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer);
    }
    ble_server_init(7);

    // End of computation start BLE
    ble_server_advertise_until_received();
    Serial.println("[BLE][Server] Advertising!");

    // End of transmission end BLE
    ble_server_end_advertise();

    TD_ACC1_Features_Carac_value_index=0;
    FD_ACC1_Peaks_Carac_value_index=0;

    /*------------------Set Sleep----------------------------*/
    if (sleep_flag){
      const int wakeup_time_sec = 3;
      esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);
      esp_deep_sleep_start();
    }
  }
}

void td_statistical_analysis(){

    int32_t aux_int32_var;
    Serial.println("Starting Time Domain Statistical Analysis");
    TD_acc_mean_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalMean)*TD_acc_mean_scalar;
    aux_int32_var = TD_acc_mean_x_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_mean_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalMean)*TD_acc_mean_scalar;
    aux_int32_var = TD_acc_mean_y_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_mean_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalMean)*TD_acc_mean_scalar;
    aux_int32_var = TD_acc_mean_z_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    //Serial.print("TD_acc_mean: ");Serial.print(TD_acc_mean_x_axis);Serial.print(" ");Serial.print(TD_acc_mean_y_axis);Serial.print(" ");Serial.println(TD_acc_mean_z_axis);

    TD_acc_std_dev_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalStdDev)*TD_acc_std_dev_scalar;
    aux_int32_var = TD_acc_std_dev_x_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_std_dev_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalStdDev)*TD_acc_std_dev_scalar;
    aux_int32_var = TD_acc_std_dev_y_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_std_dev_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalStdDev)*TD_acc_std_dev_scalar;
    aux_int32_var = TD_acc_std_dev_z_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    //Serial.print("TD_acc_std_dev: ");Serial.print(TD_acc_std_dev_x_axis);Serial.print(" ");Serial.print(TD_acc_std_dev_y_axis);Serial.print(" ");Serial.println(TD_acc_std_dev_z_axis);

    TD_acc_kurtosis_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalKurtosis)*TD_acc_kurtosis_scalar;
    aux_int32_var = TD_acc_kurtosis_x_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_kurtosis_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalKurtosis)*TD_acc_kurtosis_scalar;
    aux_int32_var = TD_acc_kurtosis_y_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_kurtosis_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalKurtosis)*TD_acc_kurtosis_scalar;
    aux_int32_var = TD_acc_kurtosis_z_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    //Serial.print("TD_acc_kurtosis: ");Serial.print(TD_acc_kurtosis_x_axis);Serial.print(" ");Serial.print(TD_acc_kurtosis_y_axis);Serial.print(" ");Serial.println(TD_acc_kurtosis_z_axis);

    TD_acc_skewness_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalSkewness)*TD_acc_skewness_scalar;
    aux_int32_var = TD_acc_skewness_x_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_skewness_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalSkewness)*TD_acc_skewness_scalar;
    aux_int32_var = TD_acc_skewness_y_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_skewness_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalSkewness)*TD_acc_skewness_scalar;
    aux_int32_var = TD_acc_skewness_z_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);    
    //Serial.print("TD_acc_skewness: ");Serial.print(TD_acc_skewness_x_axis);Serial.print(" ");Serial.print(TD_acc_skewness_y_axis);Serial.print(" ");Serial.println(TD_acc_skewness_z_axis);

    TD_acc_rms_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalRootMeanSquare)*TD_acc_rms_scalar;
    aux_int32_var = TD_acc_rms_x_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_rms_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalRootMeanSquare)*TD_acc_rms_scalar;
    aux_int32_var = TD_acc_rms_y_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    TD_acc_rms_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalRootMeanSquare)*TD_acc_rms_scalar;
    aux_int32_var = TD_acc_rms_z_axis;
    ble_encode_int32(aux_int32_var,4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer);
    //Serial.print("TD_acc_rms: ");Serial.print(TD_acc_rms_x_axis);Serial.print(" ");Serial.print(TD_acc_rms_y_axis);Serial.print(" ");Serial.println(TD_acc_rms_z_axis);
}