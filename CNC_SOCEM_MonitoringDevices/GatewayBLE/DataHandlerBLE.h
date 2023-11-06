char ACC1_Features_Service_UUID[37] = "19B10000-E8F2-537E-ACC1-D104768A1214";

// Size of FD_ACC1_Peaks_Carac is 4 bytes for the 1st peak frequency, 4 byte for the 1st peak amplitude, 4 byte 2nd, 4 byte 2nd
// 4 byte 3rd, 4 byte 3rd, 4 byte 4th, 4 byte 4th, 4 byte 5th, 4 byte 5th, 4 byte 6th, 4 byte 6th, 4 byte 7th, 4 byte 7th, 4 byte 8th, 4 byte 8th
// so 64 bytes for each axis
// so 192 bytes
const int FD_ACC1_Peaks_Carac_value_buffer_size = 192;
char FD_ACC1_Peaks_Carac_UUID[37] = "19B10001-E8F2-537E-ACC1-D104768A1214";
byte FD_ACC1_Peaks_Carac_value_buffer[FD_ACC1_Peaks_Carac_value_buffer_size];
int FD_ACC1_Peaks_Carac_value_index = 0;

char FD_ACC1_Features_Carac_UUID[37] = "19B10002-E8F2-537E-ACC1-D104768A1214";

// Size of TD_ACC1_Features_Carac is 4 bytes for the Mean, 4 bytes Std, 4 bytes kurtosis, 4 bytes skewness, 4 bytes RMS 
// so 20 bytes for each axis
// so 60 bytes
const int TD_ACC1_Features_Carac_value_buffer_size = 60;
char TD_ACC1_Features_Carac_UUID[37] = "19B10003-E8F2-537E-ACC1-D104768A1214";
byte TD_ACC1_Features_Carac_value_buffer[TD_ACC1_Features_Carac_value_buffer_size];
int TD_ACC1_Features_Carac_value_index = 0;

#include <ArduinoBLE.h>

BLEService ACC1_Features_Service(ACC1_Features_Service_UUID); // Bluetooth® Low Energy LED Service
/**/
// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, readable by client
// Maximum length of data is 512 bytes.

BLECharacteristic FD_ACC1_Peaks_Carac(FD_ACC1_Peaks_Carac_UUID, BLERead | BLEWrite, FD_ACC1_Peaks_Carac_value_buffer_size);
BLECharacteristic FD_ACC1_Features_Carac(FD_ACC1_Features_Carac_UUID, BLERead | BLEWrite, 192);
BLECharacteristic TD_ACC1_Features_Carac(TD_ACC1_Features_Carac_UUID, BLERead | BLEWrite, TD_ACC1_Features_Carac_value_buffer_size);

void ble_client_init(){
  if (!BLE.begin()) {
    Serial.println("[BLE][Client] Starting Bluetooth® Low Energy client module failed!");
    while (1);
  }
  // start scanning for Button Device BLE peripherals
  BLE.scanForUuid(ACC1_Features_Service_UUID);

}

void ble_client_start_scan(){

  // start scanning for Button Device BLE peripherals
  BLE.scanForUuid(ACC1_Features_Service_UUID);

}

void ble_client_off(){
  BLE.stopScan();
  BLE.end();
}

void ble_server_init(int machine_id = 99){
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("[BLE][Server] starting Bluetooth® Low Energy server module failed!");
    while (1);
  }

  //BLE.setLocalName("ACC_Features_Service");
  BLE.setLocalName("ACC_Features_Service_Printer6");
  //BLE.setLocalName("ACC_Features_Service_Printer4");


  BLE.setAdvertisedService(ACC1_Features_Service);
  // add the characteristic to the service
  ACC1_Features_Service.addCharacteristic(TD_ACC1_Features_Carac);
  ACC1_Features_Service.addCharacteristic(FD_ACC1_Peaks_Carac);
  // add service
  BLE.addService(ACC1_Features_Service);
  // set the initial value for the characeristic:
}

void ble_server_advertise_until_received(){
  BLE.advertise();

  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  while(!central){
    // listen for Bluetooth® Low Energy peripherals to connect:
    central = BLE.central();
  }
  if (central) {
    Serial.print("[BLE][Server] Connected to client: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // while the central is still connected to peripheral:
    int nr_ble_writes = 0;
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      int check_write_TD_ACC1_Features = TD_ACC1_Features_Carac.writeValue(TD_ACC1_Features_Carac_value_buffer,TD_ACC1_Features_Carac_value_buffer_size);
      int check_write_FD_ACC1_Peaks = FD_ACC1_Peaks_Carac.writeValue(FD_ACC1_Peaks_Carac_value_buffer,FD_ACC1_Peaks_Carac_value_buffer_size);
      /*
      if (check_write_FD_ACC1_Peaks & check_write_TD_ACC1_Features){
        Serial.print("Checks: ");Serial.print(check_write_FD_ACC1_Peaks);Serial.print(" ");Serial.println(check_write_TD_ACC1_Features);
        break;
      }
      */
      //Serial.print("[BLE][Server]  Nr of writeValues: ");Serial.println(nr_ble_writes++);
    }
    Serial.println("[BLE][Server] Disconected from client");
  }
}

void ble_server_end_advertise(){
  BLE.stopAdvertise();
}

void ble_server_off(){
  BLE.stopAdvertise();
  BLE.end();
}

void ble_encode_uint32(uint32_t variable,int variable_size, int *buffer_index, byte *buffer){
  for(int i = (variable_size-1); i>=0; i--){
    int byte_position = 8*i;
    buffer[(*buffer_index)++]= (byte)((variable) >> byte_position);
  }
}

void ble_encode_int32(int32_t variable,int variable_size, int *buffer_index, byte *buffer){
  for(int i = (variable_size-1); i>=0; i--){
    int byte_position = 8*i;
    buffer[(*buffer_index)++]= (byte)((variable) >> byte_position);
  }
}

uint32_t ble_decode_uint32(int variable_size,int *buffer_index, byte *buffer){
  uint32_t variable = 0;
  for(int i = (variable_size-1); i>=0; i--){
    int byte_position = 8*i;
    byte byte_to_add = buffer[(*buffer_index)++];
    variable |= byte_to_add << byte_position;
  }  
  return variable;
}

int32_t ble_decode_int32(int variable_size,int *buffer_index, byte *buffer){
  int32_t variable = 0;
  for(int i = (variable_size-1); i>=0; i--){
    int byte_position = 8*i;
    byte byte_to_add = buffer[(*buffer_index)++];
    variable |= byte_to_add << byte_position;
  }  
  return variable;
}