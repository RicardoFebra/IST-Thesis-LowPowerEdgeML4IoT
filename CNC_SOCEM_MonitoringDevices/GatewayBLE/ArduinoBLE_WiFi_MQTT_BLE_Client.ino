#include <Arduino.h>

#include <esp_task_wdt.h>
#define WDT_TIMEOUT 10

#define CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE 1
//------------------------------------------------------------------------------------------
//                              BLE

#define ARDUINO_BLE
#include "DataHandlerBLE.h"

void get_BLE_data(BLEDevice peripheral);

void get_ACC_FD_Peaks();
void get_ACC_TD_Features();

bool handle_td_acc1_features(BLEDevice peripheral);
bool handle_fd_acc1_peaks(BLEDevice peripheral);
//bool handle_fd_acc1_features(BLEDevice peripheral);
void send_BLE_ACC_Features(BLEDevice peripheral);

//------------------------------------------------------------------------------------------
//                              WIFI

#include <WiFi.h>
#include "secrets.h"

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
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
}

//------------------------------------------------------------------------------------------
//                              MQTT

#include <ArduinoMqttClient.h>
bool retained = false;
int qos = 0;
bool dup_mqtt = false;

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

void task_reconnectingWIFI()
{
  //Serial.println("[TASK] Starting Task reconecting WIFI");
  if (WiFi.status() == WL_CONNECTED)
  {
    //Serial.println("[WIFI] WiFi OK");
    wifiOK = 1;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    wifiOK = 0;
    mqttOK = 0;
    // Start connection to WLAN router and print a status value
    //Serial.println("[WIFI] Trying to connect to WLAN router");
    WiFi.disconnect();

    status = WiFi.begin(ssid, pass);
    // WL_IDLE_STATUS     = 0
    // WL_NO_SSID_AVAIL   = 1
    // WL_SCAN_COMPLETED  = 2
    // WL_CONNECTED       = 3
    // WL_CONNECT_FAILED  = 4
    // WL_CONNECTION_LOST = 5
    // WL_DISCONNECTED    = 6

    //Serial.print("[WIFI] Wifi status= ");
    //Serial.println (status);
    if (status == WL_CONNECTED)
    {
        wifiOK = 1;
        //Serial.println("[WIFI] Connection to WLAN router successful");
        printCurrentNet();
        printWifiData();
    }
    delay(50);
  }

  CheckMQTT();     

}

void wifi_off(){
  
  //Serial.println("WIFI off");
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_OFF);
  delay(100);
}

const uint32_t TIMEOUT_MS = 10000;//don't work the maximum is 8.3 seconds

//------------------------------------------------------------------------------------------
//                              Data
#include "DataHandlerAcc.h"

//------------------------------------------------------------------------------------------
//                              SETUP

void setup() {

  //Serial.begin(115200);
  //while (!Serial) {}; // wait for //Serial port to connect. Needed for native USB port only
  //Serial.println("[BOOT] .............Starting............");

  /*-------------------WDG------------------*/
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  pinMode(13,OUTPUT);
  
  esp_task_wdt_reset();
  task_reconnectingWIFI();
  delay(500);

  ble_client_init();

}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    /*Serial.print("Found "); Serial.print(peripheral.address());
    Serial.print(" '");Serial.print(peripheral.localName());Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
    */
    esp_task_wdt_reset();
    
    if (peripheral.localName().indexOf("Features") < 0) {
      return;  // If the name doesn't have "Features" in it then ignore it
    }

    //Serial.println("[BLE] Connecting ...");
    if (peripheral.connect()) {
      //Serial.println("[BLE] Connected");

      get_BLE_data(peripheral);
    
      ble_client_off();
      
      esp_task_wdt_reset();
      task_reconnectingWIFI();

      get_ACC_FD_Peaks();
      get_ACC_TD_Features();

      send_BLE_ACC_Features(peripheral);

      // peripheral disconnected, start scanning again
      //wifi_off();
      ble_client_init();

    } else {
      //Serial.println("Failed to connect!");
      return;
    }
    //Serial.println("[BLE][Client] Finding new servers");
  }
}

void get_BLE_data(BLEDevice peripheral) {
  // discover peripheral attributes
  //Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    //Serial.println("Attributes discovered");
  } else {
    //Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }
  bool flag_td_acc1_features = false;
  bool flag_fd_acc1_peaks = false;
  bool flag_fd_acc1_features = false;
  flag_td_acc1_features = handle_td_acc1_features(peripheral);
  flag_fd_acc1_peaks = handle_fd_acc1_peaks(peripheral);

  //flag_fd_acc1_features = handle_fd_acc1_features(peripheral);

  if (flag_td_acc1_features && flag_fd_acc1_peaks) {
    peripheral.disconnect();
    //Serial.println("All data received");
  } else {
    peripheral.disconnect();
    //Serial.println("Data not received");
  }

  TD_ACC1_Features_Carac_value_index = 0;
  FD_ACC1_Peaks_Carac_value_index = 0;
}

bool handle_td_acc1_features(BLEDevice peripheral){
  // retrieve the TD_ACC1_Features_Carac characteristic
  BLECharacteristic TD_ACC1_Features_Carac = peripheral.characteristic(TD_ACC1_Features_Carac_UUID);
  if (!TD_ACC1_Features_Carac) {return false;}
  while (peripheral.connected()) {
    // while the peripheral is connected
    if (TD_ACC1_Features_Carac.canRead()) {
      TD_ACC1_Features_Carac.readValue(TD_ACC1_Features_Carac_value_buffer, TD_ACC1_Features_Carac_value_buffer_size);

      return true;
    }
  }
  return false;
}

bool handle_fd_acc1_peaks(BLEDevice peripheral){
  // retrieve the TD_ACC1_Features_Carac characteristic
  BLECharacteristic FD_ACC1_Peaks_Carac = peripheral.characteristic(FD_ACC1_Peaks_Carac_UUID);
  if (!FD_ACC1_Peaks_Carac) {return false;}
  while (peripheral.connected()) {
    // while the peripheral is connected
    if (FD_ACC1_Peaks_Carac.canRead()) {
      FD_ACC1_Peaks_Carac.readValue(FD_ACC1_Peaks_Carac_value_buffer, FD_ACC1_Peaks_Carac_value_buffer_size);

      return true;
    }
  }
  return false;
}

/*
bool handle_fd_acc1_features(BLEDevice peripheral){
  // retrieve the TD_ACC1_Features_Carac characteristic
  BLECharacteristic FD_ACC1_Features_Carac = peripheral.characteristic(FD_ACC1_Features_Carac_UUID);
  if (!FD_ACC1_Features_Carac) {return false;}
  while (peripheral.connected()) {
    // while the peripheral is connected
    if (FD_ACC1_Features_Carac.canRead()) {
      FD_ACC1_Features_Carac.readValue(FD_ACC1_Features_Carac_value_buffer, FD_ACC1_Features_Carac_value_buffer_size);

      for (int i = 0; i < FD_ACC1_Features_Carac_value_buffer_size; i++) {
        Serial.print(FD_ACC1_Features_Carac_value_buffer[i]);
        Serial.print(" ");
      }
      Serial.println();
      return true;
    }
  }
  return false;
}
*/

void get_ACC_TD_Features(){
  TD_acc_mean_x_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_mean_scalar;
  TD_acc_mean_y_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_mean_scalar;
  TD_acc_mean_z_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_mean_scalar;
  //Serial.print("TD_acc_mean: ");Serial.print(TD_acc_mean_x_axis);Serial.print(" ");Serial.print(TD_acc_mean_y_axis);Serial.print(" ");Serial.println(TD_acc_mean_z_axis);

  TD_acc_std_dev_x_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_std_dev_scalar;
  TD_acc_std_dev_y_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_std_dev_scalar;
  TD_acc_std_dev_z_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_std_dev_scalar;
  //Serial.print("TD_acc_std_dev: ");Serial.print(TD_acc_std_dev_x_axis);Serial.print(" ");Serial.print(TD_acc_std_dev_y_axis);Serial.print(" ");Serial.println(TD_acc_std_dev_z_axis);

  TD_acc_kurtosis_x_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_kurtosis_scalar;
  TD_acc_kurtosis_y_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_kurtosis_scalar;
  TD_acc_kurtosis_z_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_kurtosis_scalar;
  //Serial.print("TD_acc_kurtosis: ");Serial.print(TD_acc_kurtosis_x_axis);Serial.print(" ");Serial.print(TD_acc_kurtosis_y_axis);Serial.print(" ");Serial.println(TD_acc_kurtosis_z_axis);

  TD_acc_skewness_x_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_skewness_scalar;
  TD_acc_skewness_y_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_skewness_scalar;
  TD_acc_skewness_z_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_skewness_scalar;
  //Serial.print("TD_acc_skewness: ");Serial.print(TD_acc_skewness_x_axis);Serial.print(" ");Serial.print(TD_acc_skewness_y_axis);Serial.print(" ");Serial.println(TD_acc_skewness_z_axis);

  TD_acc_rms_x_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_rms_scalar;
  TD_acc_rms_y_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_rms_scalar;
  TD_acc_rms_z_axis = ble_decode_int32(4,&TD_ACC1_Features_Carac_value_index,TD_ACC1_Features_Carac_value_buffer)/TD_acc_rms_scalar;
  //Serial.print("TD_acc_rms: ");Serial.print(TD_acc_rms_x_axis);Serial.print(" ");Serial.print(TD_acc_rms_y_axis);Serial.print(" ");Serial.println(TD_acc_rms_z_axis);
}

void get_ACC_FD_Peaks(){
  
  for(int i = 0; i < acc_nr_freq_peaks; i++){
    FD_acc_data_peaks_freq_x_axis[i] = ble_decode_int32(4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer);
    FD_acc_data_peaks_amps_x_axis[i] = ble_decode_int32(4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer)/FD_acc_data_peaks_amp_scalar;
    //Serial.print("x-axis: ");Serial.print(FD_acc_data_peaks_freq_x_axis[i]);Serial.print(" ");Serial.print(FD_acc_data_peaks_amps_x_axis[i]);Serial.print("; ");

    FD_acc_data_peaks_freq_y_axis[i] = ble_decode_int32(4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer);
    FD_acc_data_peaks_amps_y_axis[i] = ble_decode_int32(4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer)/FD_acc_data_peaks_amp_scalar;
    //Serial.print("y-axis: ");Serial.print(FD_acc_data_peaks_freq_y_axis[i]);Serial.print(" ");Serial.print(FD_acc_data_peaks_amps_y_axis[i]);Serial.print("; ");

    FD_acc_data_peaks_freq_z_axis[i] = ble_decode_int32(4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer);
    FD_acc_data_peaks_amps_z_axis[i] = ble_decode_int32(4,&FD_ACC1_Peaks_Carac_value_index,FD_ACC1_Peaks_Carac_value_buffer)/FD_acc_data_peaks_amp_scalar;
    //Serial.print("z-axis: ");Serial.print(FD_acc_data_peaks_freq_z_axis[i]);Serial.print(" ");Serial.println(FD_acc_data_peaks_amps_z_axis[i]);
  }
}

void send_BLE_ACC_Features(BLEDevice peripheral){

  mqttClient.setUsernamePassword(TOKEN, "password");  

  /*----------------------------- Check connection ----------------------------------------------*/
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
  if (!mqttClient.connected() && wifiOK) {
    if (!mqttClient.connect(broker, port)) {
      //Serial.print("MQTT connection failed! Error code = ");
      //Serial.println(mqttClient.connectError());
      return;
    }
  }
  
  String var_name;  
  String payload;
  payload = "{";
  /*-----------------------------Send Frequency Domain Analysis----------------------------------------------*/
  for (int i=0;i<acc_nr_freq_peaks;i++){
    var_name = "FD_acc_peaks_freq_x_"+String(i);
    payload += var_name;payload += ":";payload += int(FD_acc_data_peaks_freq_x_axis[i]);payload += ",";
    var_name = "FD_acc_peaks_amps_x_"+String(i);
    payload += var_name;payload += ":";payload += int(FD_acc_data_peaks_amps_x_axis[i]);payload += ",";
    var_name = "FD_acc_peaks_freq_y_"+String(i);
    payload += var_name;payload += ":";payload += int(FD_acc_data_peaks_freq_y_axis[i]);payload += ",";
    var_name = "FD_acc_peaks_amps_y_"+String(i);
    payload += var_name;payload += ":";payload += int(FD_acc_data_peaks_amps_y_axis[i]);payload += ",";
    var_name = "FD_acc_peaks_freq_z_"+String(i);
    payload += var_name;payload += ":";payload += int(FD_acc_data_peaks_freq_z_axis[i]);payload += ",";
    var_name = "FD_acc_peaks_amps_z_"+String(i);
    payload += var_name;payload += ":";payload += int(FD_acc_data_peaks_amps_z_axis[i]);payload += ",";
  }
  
  /*-----------------------------Send Time Domain Statistical Analysis----------------------------------------------*/
  //mqttClient.beginMessage(topic);mqttClient.print("{");
  var_name = "TD_acc_mean_x";
  payload += var_name;payload += ":";payload += TD_acc_mean_x_axis;payload += ",";
  var_name = "TD_acc_mean_y";
  payload += var_name;payload += ":";payload += TD_acc_mean_y_axis;payload += ",";
  var_name = "TD_acc_mean_z";
  payload += var_name;payload += ":";payload += TD_acc_mean_z_axis;payload += ",";
  var_name = "TD_acc_std_dev_x";
  payload += var_name;payload += ":";payload += TD_acc_std_dev_x_axis;payload += ",";
  var_name = "TD_acc_std_dev_y";
  payload += var_name;payload += ":";payload += TD_acc_std_dev_y_axis;payload += ",";
  var_name = "TD_acc_std_dev_z";
  payload += var_name;payload += ":";payload += TD_acc_std_dev_z_axis;payload += ",";
  var_name = "TD_acc_kurtosis_x";
  payload += var_name;payload += ":";payload += TD_acc_kurtosis_x_axis;payload += ",";
  var_name = "TD_acc_kurtosis_y";
  payload += var_name;payload += ":";payload += TD_acc_kurtosis_y_axis;payload += ",";
  var_name = "TD_acc_kurtosis_z";
  payload += var_name;payload += ":";payload += TD_acc_kurtosis_z_axis;payload += ",";
  var_name = "TD_acc_skewness_x";
  payload += var_name;payload += ":";payload += TD_acc_skewness_x_axis;payload += ",";
  var_name = "TD_acc_skewness_y";
  payload += var_name;payload += ":";payload += TD_acc_skewness_y_axis;payload += ",";
  var_name = "TD_acc_skewness_z";
  payload += var_name;payload += ":";payload += TD_acc_skewness_z_axis;payload += ",";
  var_name = "TD_acc_rms_x";
  payload += var_name;payload += ":";payload += int(TD_acc_rms_x_axis);payload += ",";
  var_name = "TD_acc_rms_y";
  payload += var_name;payload += ":";payload += int(TD_acc_rms_y_axis);payload += ",";
  var_name = "TD_acc_rms_z";
  payload += var_name;payload += ":";payload += int(TD_acc_rms_z_axis);
  payload += "}";
  mqttClient.beginMessage(topic,payload.length(), retained, qos, dup_mqtt);mqttClient.print(payload);mqttClient.endMessage();

  mqttClient.flush();

  //Serial.print("[WiFi][MQTT] Data sent!");
}
