/*
Arduino Nano RP2040 template for mqtt projects
The intention is to create a rock solid if is possible.
Created from different sources.
May 30 2021
jnogues@gmail.com
@rprimTech
 
This example code is in the public domain
*/

#include <Arduino.h>

//------------------------------------------------------------------------------------------
//                              WIFI

#include <WiFiNINA.h>
NinaPin ledR = LEDR;
NinaPin ledG = LEDG;
NinaPin ledB = LEDB;
#include "secrets.h"

char ssid[] = SSID;      // your network SSID (name)
char pass[] = PASS;      // your network password 
int status = WL_IDLE_STATUS;     // the WiFi radio's status

WiFiClient wifiClient;

volatile bool wifiOK = 0;

void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  //Serial.print("[WIFI] IP Address: ");
  //Serial.println(ip);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  //Serial.print("[WIFI] SSID: ");
  //Serial.println(WiFi.SSID());

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  //Serial.print("[WIFI] signal strength (RSSI):");
  //Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  //Serial.print("[WIFI] Encryption Type:");
  //Serial.println(encryption, HEX);
  //Serial.println();
}

//------------------------------------------------------------------------------------------
//                              MQTT

#include <ArduinoMqttClient.h>
String payload = "";
bool retained = false;
int qos = 0;
bool dup = false;

#define MQTT_MAX_PACKET_SIZE 500

MqttClient mqttClient(wifiClient);

volatile bool mqttOK = 0;

void CheckMQTT()
{
  if (mqttClient.connected() && wifiOK)
  {
    mqttOK = 1;
    //Serial.println("MQTT OK");
  }

  if (mqttClient.connected() && !wifiOK)
  {
    mqttOK = 0;
  }

  if (!mqttClient.connected() && !wifiOK)
  {
    mqttOK = 0;
  }
  
  mqttClient.setUsernamePassword(TOKEN, "password");
  if (!mqttClient.connected() && wifiOK)
  {
    //Serial.print("[MQTT] Attempting MQTT connection...");
    if (!mqttClient.connect(broker, port)) {
      //Serial.print("MQTT connection failed! Error code = ");
      //Serial.println(mqttClient.connectError());
      return;
    }else{
      mqttOK = 1;
      //Serial.println("MQTT OK");
    }
  }
}

//------------------------------------------------------------------------------------------
//                              Data

#include "DataHandlerMic.h"

//------------------------------------------------------------------------------------------
//                             RTOS

#include "mbed.h"
#include <rtos.h>
#include <chrono>

using namespace rtos;
using namespace mbed;

Watchdog &watchdog = Watchdog::get_instance();

Thread th1;
Thread th2;
Thread th3;
Ticker tck1;

volatile bool ticTac=0;

void task_softwareReset()
{
  //Serial.println("[TASK] Starting Task softwareReset");
  //Time before resets in miliseconds
  //1800000 - 30 min ; 3600000 - 1h
  ThisThread::sleep_for(std::chrono::milliseconds(1800000));
  //Serial.println("[RST] *******Recurrent Reset*******");
  //NVIC_SystemReset();//works!
  system_reset();//works!
}

void task_blinkLed13()
{

 //Serial.println("[TASK] Starting Task blinkLed13");
 while (true) 
    {
        digitalWrite(ledB, HIGH);
        ThisThread::sleep_for(std::chrono::milliseconds(500));  
        digitalWrite(ledB, LOW);
        ThisThread::sleep_for(std::chrono::milliseconds(500));
    }
}

void task_reconnectingWIFI()
{
  //Serial.println("[TASK] Starting Task reconectingWIFI");
  while(true)
  {
        
          if (WiFi.status() == WL_CONNECTED)
          {
            //Serial.println("[WIFI] WiFi OK");
            wifiOK = 1;
            digitalWrite(ledG, HIGH);
            digitalWrite(ledR, LOW);
          }
        
          if (WiFi.status() != WL_CONNECTED)
          {
            wifiOK = 0;
            mqttOK = 0;
            digitalWrite(ledG, LOW);
            digitalWrite(ledR, HIGH);
            // Start connection to WLAN router and print a status value
            //Serial.println("[WIFI] Trying to connect to WLAN router");
            WiFi.disconnect();

            
            Watchdog::get_instance().kick();//clear whatchdog
            ThisThread::sleep_for(std::chrono::milliseconds(1000));
        
            status = WiFi.begin(ssid, pass);
            // WL_IDLE_STATUS     = 0
            // WL_NO_SSID_AVAIL   = 1
            // WL_SCAN_COMPLETED  = 2
            // WL_CONNECTED       = 3
            // WL_CONNECT_FAILED  = 4
            // WL_CONNECTION_LOST = 5
            // WL_DISCONNECTED    = 6
            ThisThread::sleep_for(std::chrono::milliseconds(2000));
            //Serial.print("[WIFI] Wifi status= ");
            //Serial.println (status);
            if (status == WL_CONNECTED)
            {
                wifiOK = 1;
                //Serial.println("[WIFI] Connection to WLAN router successful");
                printCurrentNet();
                printWifiData();
            }
          }

          CheckMQTT();
          ThisThread::sleep_for(std::chrono::milliseconds(1000));     
  }
}

void setup() 
{ 
  //Serial.begin(115200);
  //while (!Serial) {}; // wait for //Serial port to connect. Needed for native USB port only
  //Serial.println("[BOOT] .............Starting............");
  pinMode(13,OUTPUT);
  digitalWrite(13, LOW);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, LOW);
  digitalWrite(ledB, LOW);

  //Serial.print("[WTHD] Max Timeout Whatchdog = ");
  //Serial.print(Watchdog::get_instance().get_max_timeout());
  //Serial.println(" uS");

  //Watchdog &watchdog = Watchdog::get_instance();
  watchdog.start();
  Watchdog::get_instance().kick();//clear whatchdog

  th1.start(task_blinkLed13);
  th2.start(task_reconnectingWIFI);
  th3.start(task_softwareReset);

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and -10 on the Portenta Vision Shield
  // PDM.setGain(30);
  // Initialize PDM with: one channel (mono mode)
  if (!PDM.begin(channels, mic_sampling_frequency)) {
    //Serial.println("Failed to start PDM!");
    while (1);
  }

  for(int i = 0; i < mic_fft_sample_nr; i++){mic_freq[i] = (mic_sampling_frequency/2)*i/mic_fft_sample_nr;}
  mic_fft_handler_obj.set_fft_output_units(FFT_OUTPUT_NONE);
  mic_fft_handler_obj.set_fft_input_units(FFT_INPUT_MILI);
}

int state = 0;

void loop() 
{
  // the first value is only sent when mqtt is good to measure the delay between arduino and thingsboard
  if(mic_samplesRead && mqttOK){
    for (int i = 0; i < mic_samplesRead; i++) {
      mic_data[count_mic_samples++] = mic_sampleBuffer[i];
    }
    mic_samplesRead = 0;  // Clear the read count
  }
  /*if(count_mic_samples == int(mic_sample_nr/2)){
    //Serial.println("[Buffer] data mic half full");
  }*/
  if(count_mic_samples == mic_sample_nr){
    //Serial.println("[Buffer] data mic full");
    // call poll() regularly to allow the library to send MQTT keep alives which
    // avoids being disconnected by the broker
    mqttClient.poll();
    count_mic_samples = 0;

    /*-----------------------------Time Domain Statistical Analysis----------------------------------------------*/

    TD_mic_mean = TD_mic_statistical_handler_obj.MakeAnalysis(StatisticalMean);
    TD_mic_std_dev = TD_mic_statistical_handler_obj.MakeAnalysis(StatisticalStdDev);
    TD_mic_kurtosis = TD_mic_statistical_handler_obj.MakeAnalysis(StatisticalKurtosis);
    TD_mic_skewness = TD_mic_statistical_handler_obj.MakeAnalysis(StatisticalSkewness);
    TD_mic_rms = TD_mic_statistical_handler_obj.MakeAnalysis(StatisticalRootMeanSquare);

    /*-----------------------------FFT----------------------------------------------*/

    mic_fft_handler_obj.Windowing(FFT_WIN_TYPE_HANN);
    mic_fft_handler_obj.FFT(false,true,true);
    mic_peak_handler_obj.PEAKS();
    
    int cnt = 0;
    while(WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, pass);
      delay(500);
      cnt++;
      if(cnt == 3){
        //Serial.println("Unable to connect to wifi");
        return;
      }
    }
    if (!mqttClient.connected()) {
      if (!mqttClient.connect(broker, port)) {
        //Serial.print("MQTT connection failed! Error code = ");
        //Serial.println(mqttClient.connectError());
        return;
      }
    }
    
    String var_name;  
    payload = "";
    payload += "{";
    /*-----------------------------Send Frequency Domain Analysis----------------------------------------------*/
    for (int i=0;i<mic_nr_freq_peaks;i++){
      var_name = "FD_mic_data_peaks_freq_"+String(i);
      payload += var_name;payload += ":";payload += FD_mic_data_peaks_freq[i];payload += ",";
      var_name = "FD_mic_data_peaks_amps_"+String(i);
      payload += var_name;payload += ":";payload += FD_mic_data_peaks_amps[i];payload += ",";
    }
    /*-----------------------------Send Time Domain Statistical Analysis----------------------------------------------*/
    
    var_name = "TD_mic_mean";
    payload += var_name;payload += ":";payload += TD_mic_mean;payload += ",";
    var_name = "TD_mic_std_dev";
    payload += var_name;payload += ":";payload += TD_mic_std_dev;payload += ",";
    var_name = "TD_mic_kurtosis";
    payload += var_name;payload += ":";payload += TD_mic_kurtosis;payload += ",";
    var_name = "TD_mic_skewness";
    payload += var_name;payload += ":";payload += TD_mic_skewness;payload += ",";
    var_name = "TD_mic_rms";
    payload += var_name;payload += ":";payload += TD_mic_rms;
    
    payload += "}";
    mqttClient.beginMessage(topic,payload.length(), retained, qos, dup);mqttClient.print(payload);mqttClient.endMessage();

    mqttClient.flush();

    Watchdog::get_instance().kick();   //clear whatchdog
    ThisThread::sleep_for(std::chrono::milliseconds(1000));
    Watchdog::get_instance().kick();   //clear whatchdog
  }
}