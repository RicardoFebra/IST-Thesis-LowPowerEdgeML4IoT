
#include "secrets.h"

//------------------------------------------------------------------------------------------
//                              ISTART LORA V2.0 - Init
#include "iStartLoRaV2.h"

//------------------------------------------------------------------------------------------
//                               RTC

#include <RTCZero.h>  //timer
//RTCZero rtc;
RTCZero rtc;  //Create an rtc object

void set_rtc_clock_interrupt() {
  byte h, mm, s = 0;
  //rtc.begin();
  rtc.setTime(h, mm, s);
  rtc.setAlarmTime(h, mm + 1, s);
  rtc.enableAlarm(rtc.MATCH_MMSS);
}

void set_first_rtc_clock_interrupt() {
  byte h, mm, s = 0;
  //rtc.begin();
  rtc.setTime(h, mm, s);
  rtc.setAlarmTime(h, mm, s + 10);
  rtc.enableAlarm(rtc.MATCH_MMSS);
}

volatile bool rtc_clock_alarm = false;  //variables shared between main code and ISR must be declared volatile to tell the compiler that their variable can change at any time

void rtc_clock_interrupt() {
  rtc_clock_alarm = true;
}

//------------------------------------------------------------------------------------------
//                               LoRa

#include <CayenneLPP.h>
CayenneLPP lpp(51);

#include "TinyLoRa.h"
TinyLoRa lora = TinyLoRa(RFM_DIO0, RFM_CS, RFM_RST);

//CONNECTION
#define ADC_AREF 2.33f

int lora_begin_flag = false;
int msg_count = 0;

void send_lora() {

  lora.frameCounter = msg_count;
  lora.sendData(lpp.getBuffer(), lpp.getSize(), lora.frameCounter);
  Serial.print("LoRa Packet: ");
  for (int i = 0; i < lpp.getSize(); i++) {
    Serial.print(lpp.getBuffer()[i]);
  }
  Serial.println();
  Serial.print("LoRa Packet Size: ");Serial.println(lpp.getSize());
  msg_count++;
  lpp.reset();
}

//------------------------------------------------------------------------------------------
//                              Data

#include "DataHandlerAcc.h"

//------------------------------------------------------------------------------------------
//                              Handlers
void interrupts_handler(bool aux_fifo_wtm, bool aux_rtc_clock, bool aux_sodaq_wdt) {
  acc_fifo_wtm_reached = aux_fifo_wtm;
  rtc_clock_alarm = aux_rtc_clock;
  sodaq_wdt_flag = aux_sodaq_wdt;
}
//------------------------------------------------------------------------------------------
//                              SETUP

void setup() {
  Serial.begin(256000);
  //while(!Serial);
  sodaq_wdt_safe_delay(20000);

  /*------------------Set Fuses----------------------------*/
  setup_BOD33();
  nvm_wait_states();
  //Find Reset Cause
  if (REG_PM_RCAUSE == PM_RCAUSE_SYST) {
    Serial.println("Reset requested by system");
    reset_cause = 1;
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_WDT) {
    Serial.println("Reset requested by Watchdog");
    reset_cause = 2;
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_EXT) {
    Serial.println("External reset requested");
    reset_cause = 3;
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_BOD33) {
    Serial.println("Reset brown out 3.3V");
    reset_cause = 4;
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_BOD12) {
    Serial.println("Reset brown out 1.2v");
    reset_cause = 5;
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_POR) {
    Serial.println("Normal power on reset");
    reset_cause = 0;
  }  //End find reset cause

  //stuff to allow falling interrupt
  // Set the XOSC32K to run in standby
  SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;

  /*------------------Set RTC----------------------------*/

  rtc.begin();
  rtc.attachInterrupt(rtc_clock_interrupt);
  set_first_rtc_clock_interrupt();

  /*------------------Set WDT----------------------------*/

  sodaq_wdt_enable(WDT_PERIOD_8X);  //ENABLE WDT

  /*------------------Set LoRa----------------------------*/

  pinMode(DIO1, INPUT);

  pinMode(BUZZER, OUTPUT);

  lora.setChannel(MULTI);
  lora.setDatarate(SF9BW125);
  if (!lora.begin()) {
    Serial.println("Failed");
    Serial.println("Check your radio");
    while (true) {
      delay(1000);
      Serial.println("Failed");
      Serial.println("Check your radio");
      delay(1000);
    }
  } else {
    Serial.println("INIT LORA SUCCESS");
  }

  lora.sleep();

  /*------------------Set ACC----------------------------*/
  Wire.begin();
  Wire.setClock(400000);

  mma8451_obj.set_mode_1(acc_fifo_wtm_val, acc_sampling_frequency, acc_scale);

  // (if external reset (press reset button) or normal power on or requested to flash and program memory) save offset values for acc
  if (reset_cause==3 || reset_cause==0 || reset_cause==1) {
    sodaq_wdt_safe_delay(5000);
    Serial.println("Set ACC offset");
    int16_t fifo_data[3];
    // the first value in the registers of the acc is trash, so the registers are read twice to set the offset value
    mma8451_obj.get_acc_data(fifo_data);
    mma8451_obj.get_acc_data(fifo_data);
    mma8451_obj.set_offset_value_with_current_vals();
    //mma8451_obj.set_gravity_compensation();
  }

  mma8451_obj.set_sleep_mode_1();

  pinMode(INT_BLACK_CABLE_MOLEX, INPUT_PULLUP);
  pinMode(BAT_2_SENSOR_POWER, OUTPUT);
  digitalWrite(BAT_2_SENSOR_POWER, HIGH);

  attachInterrupt(digitalPinToInterrupt(INT_BLACK_CABLE_MOLEX), acc_fifo_wtm_interrupt, RISING);

  // Configure EIC to use GCLK1 which uses XOSC32K
  // This has to be done after the first call to attachInterrupt()
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

  /*------------------Set FFT----------------------------*/

  for (int i = 0; i < (acc_fft_sample_nr); i++) { acc_freq[i] = (i * acc_sampling_frequency) / (acc_window_sample_nr); }

  acc_fft_handler_obj_x_axis.set_fft_output_units(FFT_OUTPUT_NONE);
  acc_fft_handler_obj_x_axis.set_fft_input_units(FFT_INPUT_MILI);

  acc_fft_handler_obj_y_axis.set_fft_output_units(FFT_OUTPUT_NONE);
  acc_fft_handler_obj_y_axis.set_fft_input_units(FFT_INPUT_MILI);

  acc_fft_handler_obj_z_axis.set_fft_output_units(FFT_OUTPUT_NONE);
  acc_fft_handler_obj_z_axis.set_fft_input_units(FFT_INPUT_MILI);

  sodaq_wdt_reset();  //Feed WDT
  sodaq_wdt_safe_delay(1000);
}

float b_volt = 0;

void check_bat_send_data(){
  b_volt = getBatteryVoltage()/ (1000.0);
  //lpp.addAnalogInput(0, b_volt);  // add battery voltage to payload
  if (b_volt <= 2.9) {
    Serial.println("Battery Low!");
    interrupts_handler(false, false, false);
    mma8451_obj.set_sleep_mode_1();
  } else {
    if (lora_begin_flag){
      send_lora();
    }
  }
}

int lookup_table_count = 0;
void loop() {

  if ((~acc_fifo_wtm_reached) && (~rtc_clock_alarm) && (~sodaq_wdt_flag)) {
    if (sleep_flag) {
      Serial.println("Sleep Mode!");
      sodaq_wdt_disable();
      lora.sleep();

      MCU_Sleep();
      sodaq_wdt_enable(WDT_PERIOD_8X);  //ENABLE WDT

      //sleep_flag = false;
    }
  }
  if (rtc_clock_alarm) {
    Serial.println("Alarm clock!");
    rtc_clock_alarm = false;
    mma8451_obj.set_mode_1(acc_fifo_wtm_val, acc_sampling_frequency, acc_scale);

    //sleep_flag = true;
  }
  if (acc_fifo_wtm_reached) {
    //Serial.println("FIFO wtm level reached!");
    acc_fifo_wtm_reached = false;

    for (uint8_t i = 0; i < acc_fifo_wtm_val; i++) {
      int16_t fifo_data[3];
      mma8451_obj.get_acc_data(fifo_data);
      int16_t aux_fifo_data_x_axis = (fifo_data[0]);
      int16_t aux_fifo_data_y_axis = (fifo_data[1]);
      int16_t aux_fifo_data_z_axis = (fifo_data[2]);

      acc_data_y_axis[acc_sample_count] = aux_fifo_data_y_axis;
      acc_data_z_axis[acc_sample_count] = aux_fifo_data_z_axis;
      acc_data_x_axis[acc_sample_count++] = aux_fifo_data_x_axis;
    }
    // Buffer is full - start analysis
    if (acc_sample_count == acc_sample_nr) {
      lookup_table_count = 0;
      if (!lora.begin()) {
        Serial.println("Failed");
        Serial.println("Check your radio");
        lora_begin_flag = false;
      }else{
        lora_begin_flag = true;
      }
      mma8451_obj.set_sleep_mode_1();
      acc_fifo_wtm_reached = false;
      acc_sample_count = 0;

      sodaq_wdt_reset();  //Feed WDT
      Serial.println("Starting Time Domain Statistical Analysis");
      TD_acc_mean_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalMean)*TD_acc_mean_scalar;
      TD_acc_mean_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalMean)*TD_acc_mean_scalar;
      TD_acc_mean_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalMean)*TD_acc_mean_scalar;
      lpp.addAccelerometer(0 + TD_wireless_channel_offset, TD_acc_mean_x_axis, TD_acc_mean_y_axis, TD_acc_mean_z_axis);
      TD_acc_std_dev_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalStdDev)*TD_acc_std_dev_scalar;
      TD_acc_std_dev_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalStdDev)*TD_acc_std_dev_scalar;
      TD_acc_std_dev_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalStdDev)*TD_acc_std_dev_scalar;
      lpp.addAccelerometer(1 + TD_wireless_channel_offset, TD_acc_std_dev_x_axis,TD_acc_std_dev_y_axis,TD_acc_std_dev_z_axis);
      TD_acc_kurtosis_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalKurtosis)*TD_acc_kurtosis_scalar;
      TD_acc_kurtosis_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalKurtosis)*TD_acc_kurtosis_scalar;
      TD_acc_kurtosis_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalKurtosis)*TD_acc_kurtosis_scalar;
      lpp.addAccelerometer(2 + TD_wireless_channel_offset, TD_acc_kurtosis_x_axis,TD_acc_kurtosis_y_axis,TD_acc_kurtosis_z_axis);
      TD_acc_skewness_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalSkewness)*TD_acc_skewness_scalar;
      TD_acc_skewness_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalSkewness)*TD_acc_skewness_scalar;
      TD_acc_skewness_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalSkewness)*TD_acc_skewness_scalar;
      lpp.addAccelerometer(3 + TD_wireless_channel_offset, TD_acc_skewness_x_axis,TD_acc_skewness_y_axis,TD_acc_skewness_z_axis);
      TD_acc_rms_x_axis = TD_acc_statistical_handler_obj_x_axis.MakeAnalysis(StatisticalRootMeanSquare)*TD_acc_rms_scalar;
      TD_acc_rms_y_axis = TD_acc_statistical_handler_obj_y_axis.MakeAnalysis(StatisticalRootMeanSquare)*TD_acc_rms_scalar;
      TD_acc_rms_z_axis = TD_acc_statistical_handler_obj_z_axis.MakeAnalysis(StatisticalRootMeanSquare)*TD_acc_rms_scalar;
      lpp.addAccelerometer(4 + TD_wireless_channel_offset, TD_acc_rms_x_axis,TD_acc_rms_y_axis,TD_acc_rms_z_axis);

      check_bat_send_data();
      sodaq_wdt_reset();  //Feed WDT
      Serial.println("Starting FFT");
      
      acc_fft_handler_obj_x_axis.Windowing(FFT_WIN_TYPE_HANN);
      acc_fft_handler_obj_x_axis.FFT(true,true,true);
      acc_peak_handler_obj_x_axis.PEAKS();

      acc_fft_handler_obj_y_axis.Windowing(FFT_WIN_TYPE_HANN);
      acc_fft_handler_obj_y_axis.FFT(true,true,true);
      acc_peak_handler_obj_y_axis.PEAKS();

      acc_fft_handler_obj_z_axis.Windowing(FFT_WIN_TYPE_HANN);
      acc_fft_handler_obj_z_axis.FFT(true,true,true);
      acc_peak_handler_obj_z_axis.PEAKS();

      Serial.println("Starting Freq Domain Analysis");
      uint32_t frequency = 0;
      uint32_t amplitude = 0;
      for(uint8_t array_idx = 0;array_idx<acc_nr_freq_peaks;array_idx++) {
        // FD_data_peaks_acc_freq has only positive values
        int aux_array_idx = (array_idx*2);
        frequency = static_cast<uint32_t>(FD_acc_data_peaks_freq_z_axis[array_idx] + 0.5);
        amplitude = static_cast<uint32_t>((FD_acc_data_peaks_amps_z_axis[array_idx]*FD_acc_data_peaks_amp_scalar) + 0.5);
        lpp.addDigitalOutput(aux_array_idx, frequency);
        lpp.addDigitalOutput((aux_array_idx+1), amplitude);
      }
      for(uint8_t array_idx = 0;array_idx<acc_nr_freq_peaks/2;array_idx++) {
        // FD_data_peaks_acc_freq has only positive values
        int aux_array_idx = (array_idx*2);
        frequency = static_cast<uint32_t>(FD_acc_data_peaks_freq_y_axis[array_idx] + 0.5);
        amplitude = static_cast<uint32_t>((FD_acc_data_peaks_amps_y_axis[array_idx]*FD_acc_data_peaks_amp_scalar) + 0.5);
        lpp.addLuminosity(aux_array_idx, frequency);
        lpp.addLuminosity((aux_array_idx+1), amplitude);
      }
      check_bat_send_data();
      for(uint8_t array_idx = acc_nr_freq_peaks/2;array_idx<acc_nr_freq_peaks;array_idx++) {
        // FD_data_peaks_acc_freq has only positive values
        int aux_array_idx = (array_idx*2);
        frequency = static_cast<uint32_t>(FD_acc_data_peaks_freq_y_axis[array_idx] + 0.5);
        amplitude = static_cast<uint32_t>((FD_acc_data_peaks_amps_y_axis[array_idx]*FD_acc_data_peaks_amp_scalar) + 0.5);
        lpp.addLuminosity(aux_array_idx, frequency);
        lpp.addLuminosity((aux_array_idx+1), amplitude);
      }
      for(uint8_t array_idx = 0;array_idx<acc_nr_freq_peaks;array_idx++) {
        // FD_data_peaks_acc_freq has only positive values
        int aux_array_idx = (array_idx*2);
        frequency = static_cast<uint32_t>(FD_acc_data_peaks_freq_x_axis[array_idx] + 0.5);
        amplitude = static_cast<uint32_t>((FD_acc_data_peaks_amps_x_axis[array_idx]*FD_acc_data_peaks_amp_scalar)+0.5);
        lpp.addDigitalInput(aux_array_idx, frequency);
        lpp.addDigitalInput((aux_array_idx+1), amplitude);
      }
      check_bat_send_data();

      set_rtc_clock_interrupt();
    }
    
    //sleep_flag = true;
  }
}