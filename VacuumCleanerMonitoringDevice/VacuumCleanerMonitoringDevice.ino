#include <Arduino.h>
#include <PDM.h>
#include <Arduino_LSM6DSOX.h>

bool LED_SWITCH = false;

// default number of output channels
byte channels = 1;
byte nr_samples=32;

static const int k_acc=1000;
static const int k_gyro=1000;
// default PCM output frequency
static const int frequency = 20000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];

char buffer[512];

// Number of audio samples read
volatile int samplesRead;

unsigned short label_time_switch = 10000;

unsigned long prev_time;

/* labels:
00 - speed 0, no obstruction in the end of the tube, closed middle cover    0:00
01 - speed 1, no obstruction in the end of the tube, closed middle cover    0:20
02 - speed 1, with obstruction in the end of the tube, closed middle cover  0:40 
03 - speed 1, with obstruction in the end of the tube, open middle cover    1:00
04 - speed 1, no obstruction in the end of the tube, open middle cover      1:20
05 - speed 2, no obstruction in the end of the tube, closed middle cover            1:40
06 - speed 2, with obstruction in the end of the tube, closed middle cover  2:00
07 - speed 2, with obstruction in the end of the tube, open middle cover    2:20
08 - speed 2, no obstruction in the end of the tube, open middle cover      2:40
09 - speed 3, no obstruction in the end of the tube, closed middle cover           3:00
10 - speed 3, with obstruction in the end of the tube, closed middle cover  3:20
11 - speed 3, with obstruction in the end of the tube, open middle cover    3:40
12 - speed 3, no obstruction in the end of the tube, open middle cover      4:00 - 4:10

*/

byte label=0;
byte iterations=0;

float Ax, Ay, Az;
float Gx, Gy, Gz;
int Ax_int, Ay_int, Az_int;
int Gx_int, Gy_int, Gz_int;

// each sensor send a flag to tell if their respective values are in the serial
// 1 is in the serial 0 is not
int flag_acc=0;
int flag_gyro=0;
int flag_micro=0;

byte nr_labels=13;

/**
   Callback function to process the data from the PDM microphone.
   NOTE: This callback is executed as part of an ISR.
   Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

byte b_init[2]; 
float gyro_sample_rate,acc_sample_rate;
unsigned short gyro_sample_rate_ushort,acc_sample_rate_ushort;

void setup() {
  Serial.begin(921600);
  pinMode(LEDB, OUTPUT);
  while (!Serial);
  // Configure the data receive callback
  PDM.onReceive(onPDMdata);


  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and -10 on the Portenta Vision Shields
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shields
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  acc_sample_rate=IMU.accelerationSampleRate();
  acc_sample_rate_ushort=int(acc_sample_rate*100);
  b_init[0] = acc_sample_rate_ushort>>8;  
  b_init[1] = (byte) acc_sample_rate_ushort;
  Serial.write(b_init,2);//AccelerometerSampleRate
  Serial.println("");

  gyro_sample_rate=IMU.gyroscopeSampleRate();
  gyro_sample_rate_ushort=int(gyro_sample_rate*100);
  b_init[0] = gyro_sample_rate_ushort>>8;  
  b_init[1] = (byte) gyro_sample_rate_ushort;
  Serial.write(b_init,2);//GyroscopeSampleRate
  Serial.println("");

  b_init[0] = frequency>>8;
  b_init[1] = (byte) frequency;
  Serial.write(b_init,2);//PDM_Frequency
  Serial.println("");

  b_init[0] = label_time_switch>>8;  
  b_init[1] = (byte) label_time_switch;
  Serial.write(b_init,2);//label_time_switch
  Serial.println("");

  Serial.write(channels);//NrChannels
  Serial.println("");

  Serial.write(nr_labels);//NrLabels
  Serial.println("");

  //Serial.print("label,iteration,timestamp,flag_a,flag_g,flag_m,ax,ay,az,gx,gy,gz,micro\n");

  pinMode(LEDR,OUTPUT);
  digitalWrite(LEDR,HIGH);
  pinMode(LEDB,OUTPUT);
  digitalWrite(LEDB,LOW);
  pinMode(LEDG, OUTPUT);
  digitalWrite(LEDR,LOW);


  prev_time = millis();
}

byte get_flag_val();
void set_label();
void print_data();

int i;
unsigned long time_aux;
int size_init_vals=7;
int size_acc_gyro=6;

byte flags;
byte acc_buffer[6];
byte gyr_buffer[6];

void loop() {

  // Wait for samples to be read
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);
    flag_acc=1;
    Ax_int=(int)(Ax*k_acc);
    Ay_int=(int)(Ay*k_acc);
    Az_int=(int)(Az*k_acc);

  }else{
    flag_acc=0;
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Gx, Gy, Gz);
    flag_gyro=1;
    Gx_int=(int)(Gx*k_gyro);
    Gy_int=(int)(Gy*k_gyro);
    Gz_int=(int)(Gz*k_gyro);

  }else{
    flag_gyro=0;
  }

  if (samplesRead) { 
    flag_micro=1;
  }else{
    flag_micro=0;
  }
  
  set_label();
  if (flag_micro==1 || flag_acc==1 || flag_gyro==1){
    flags=get_flag_val();
    print_data();
    // Clear the read count
    samplesRead = 0;
  }

}

byte get_flag_val(){

  if (flag_acc==1){
    if (flag_gyro==1){
      //return 111 and 110
      if (flag_micro==1){return 0x07;}else{return 0x06;}
    }else{
      //return 101 and 100
      if (flag_micro==1){return 0x05;}else{return 0x04;}
    }
  }else{
    if (flag_gyro==1){
      //return 011 and 010
      if (flag_micro==1){return 0x03;}else{return 0x02;}
    }else{
      //return 001 and 000
      if (flag_micro==1){return 0x01;}else{return 0x00;}
    }
  }
}

void set_label(){
  if ((millis()-prev_time>=label_time_switch)){
    if (label==(nr_labels-1)){
      label=-1;
      digitalWrite(LEDG,HIGH);
    }else{
      digitalWrite(LEDB, HIGH);
    }
    digitalWrite(LEDR, LOW);
    label=label+1;
    iterations=iterations+1;
    delay(label_time_switch);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);
    prev_time=millis();
    digitalWrite(LEDR, HIGH);
  }
}

void print_data(){

    int cnt_buffer=0;

    byte byte_buffer[90];

    time_aux=millis();

    byte_buffer[cnt_buffer++]=label;
    byte_buffer[cnt_buffer++]=iterations;
    byte_buffer[cnt_buffer++]=(byte) time_aux;
    byte_buffer[cnt_buffer++]=time_aux>>8;
    byte_buffer[cnt_buffer++]=time_aux>>16;
    byte_buffer[cnt_buffer++]=time_aux>>24;
    byte_buffer[cnt_buffer++]=flags;

    if ((flags & (1 << (3-1))) != 0){

      byte_buffer[cnt_buffer++]=Ax_int>>8;
      byte_buffer[cnt_buffer++]=(byte) Ax_int;
      byte_buffer[cnt_buffer++]=Ay_int>>8;
      byte_buffer[cnt_buffer++]=(byte) Ay_int;
      byte_buffer[cnt_buffer++]=Az_int>>8;
      byte_buffer[cnt_buffer++]=(byte) Az_int;
    }

    if ((flags & (1 << (2-1))) != 0){

      byte_buffer[cnt_buffer++]=Gx_int>>8;
      byte_buffer[cnt_buffer++]=(byte) Gx_int;
      byte_buffer[cnt_buffer++]=Gy_int>>8;
      byte_buffer[cnt_buffer++]=(byte) Gy_int;
      byte_buffer[cnt_buffer++]=Gz_int>>8;
      byte_buffer[cnt_buffer++]=(byte) Gz_int;
    }

    if ((flags & (1 << (1-1))) != 0){
      for (i=0;i<samplesRead;i++){
        byte_buffer[cnt_buffer++]=sampleBuffer[i];
        byte_buffer[cnt_buffer++]=sampleBuffer[i]>>8;
      }
    }
    Serial.write(byte_buffer,cnt_buffer);
    Serial.println();
}