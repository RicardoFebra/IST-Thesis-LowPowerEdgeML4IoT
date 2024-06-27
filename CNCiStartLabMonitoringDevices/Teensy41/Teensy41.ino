//------------------------------------------------------------------------------------------
//                              Teensy 4.1 - Init Definition

#define OrangeLedPin 13

//------------------------------------------------------------------------------------------
//                               ACC 

//const int acc_sampling_frequency = 400;                     
/*------Define the desired ACC------*/
//#define LIS3DH_acc
//#define MMA8451_acc
//#define LSM6DSOX_acc
#define LSM6DSO_acc

#include"DataHandlerAcc.h"
int available_acc_data_counter = 0;

//------------------------------------------------------------------------------------------
//                               Datalogger

volatile uint8_t label = 0;

const uint16_t sources_nr = 3;
// bit zero of sources flags corresponds to the first source, so last bit (more to the left) is last source (more to the right)
byte sources_flags = 0x07;
char header[] = "timestamp,label,sources_flags,acc_x,acc_y,acc_z/&/";

char filename[] = "dataLog_id_0000_nr_0000.TXT";
uint8_t start_id_val = 20;

//------------------------------------------------------------------------------------------
//                              SD

#include "RingBuf.h"
#include "SdFat.h"

#define single_line_byte_nr acc_fifo_wtm_val * 3 * 2 + 6
#define header_params_byte_nr sources_nr*(2 * 4) * 2
#define tot_log_time_per_file_seconds 60

const unsigned long time_between_open_close_file_miliseconds = 1000 * tot_log_time_per_file_seconds;

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

const unsigned long nr_samples_per_file = tot_log_time_per_file_seconds * acc_sampling_frequency;
const unsigned long nr_written_rows = nr_samples_per_file / acc_fifo_wtm_val;
const unsigned long nr_bytes_per_file_samples = nr_samples_per_file * 3 * 2;
const unsigned long nr_bytes_per_file_row_params = nr_written_rows * 6;
const unsigned long aux_log_file_size = sizeof(header) + header_params_byte_nr + nr_bytes_per_file_samples + nr_bytes_per_file_row_params;
#define LOG_FILE_SIZE aux_log_file_size

const int aux_ring_buf_size_capacity = single_line_byte_nr;
#define RING_BUF_CAPACITY aux_ring_buf_size_capacity

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

// Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
uint16_t adc = 17;

unsigned long prev_open_file_time = 0;

void logAvailableData() {

  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  // Min spare micros in loop.
  int32_t minSpareMicros = INT32_MAX;

  byte row_params[6];

  if (available_acc_data_counter == 0) {
    //digitalWrite(OrangeLedPin, HIGH);
    // Save time.
    unsigned long time_aux = millis();
    row_params[0] = time_aux >> 24;
    row_params[1] = time_aux >> 16;
    row_params[2] = time_aux >> 8;
    row_params[3] = (byte)time_aux;

    row_params[4] = (byte)0x00;


    row_params[5] = sources_flags;

    rb.write(row_params, 6);
  }

  size_t n = rb.bytesUsed();
  /*if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
    Serial.println("File full - quitting.");
    return;
  }*/
  if (n > maxUsed) {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy()) {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512)) {
      Serial.println("writeOut failed");
      return;
    }
  }

  int16_t acc_data[3];
  lsm6dso_obj.get_FIFO_DATA_OUT_raw(acc_data);
  rb.write(acc_data, 6);
  available_acc_data_counter++;

  if (available_acc_data_counter >= acc_fifo_wtm_val) {
    //digitalWrite(OrangeLedPin, LOW);
    rb.sync();
    available_acc_data_counter = 0;
  }
}

void logFIFOData() {

  //digitalWrite(OrangeLedPin, HIGH);

  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  // Min spare micros in loop.
  int32_t minSpareMicros = INT32_MAX;

  byte row_params[6];

  // Save time.
  unsigned long time_aux = millis();
  row_params[0] = time_aux >> 24;
  row_params[1] = time_aux >> 16;
  row_params[2] = time_aux >> 8;
  row_params[3] = (byte)time_aux;

  row_params[4] = (byte)0x00;

  row_params[5] = sources_flags;

  rb.write(row_params, 6);

  for (uint16_t i = 0; i < acc_fifo_wtm_val; i++) {
    // Amount of data in ringBuf.
    size_t n = rb.bytesUsed();
    /*if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
      Serial.println("File full - quitting.");
      break;
    }*/
    if (n > maxUsed) {
      maxUsed = n;
    }
    if (n >= 512 && !file.isBusy()) {
      // Not busy only allows one sector before possible busy wait.
      // Write one sector from RingBuf to file.
      if (512 != rb.writeOut(512)) {
        Serial.println("writeOut failed");
        break;
      }
    }

    int16_t fifo_data[3];
    byte acc_data[6];
    lsm6dso_obj.get_FIFO_DATA_OUT_raw(fifo_data);

    acc_data[0] = fifo_data[0] >> 8;
    acc_data[1] = (byte)fifo_data[0];
    acc_data[2] = fifo_data[1] >> 8;
    acc_data[3] = (byte)fifo_data[1];
    acc_data[4] = fifo_data[2] >> 8;
    acc_data[5] = (byte)fifo_data[2];

    rb.write(acc_data, 6);

    if (rb.getWriteError()) {
      // Error caused by too few free bytes in RingBuf.
      size_t n = rb.bytesUsed();
      Serial.println("Bytes Used: ");
      Serial.println(n);
      Serial.println("WriteError");
      break;
    }
  }
  // Write any RingBuf data to file.
  rb.sync();

  //digitalWrite(OrangeLedPin, LOW);
}

void clearSerialInput() {
  for (uint32_t m = micros(); micros() - m < 10000;) {
    if (Serial.read() >= 0) {
      m = micros();
    }
  }
}

void setup() {
  Serial.begin(115200);
  //while (!Serial) {}
  SPI.begin();

  /*------------------Set ACC----------------------------*/
  lsm6dso_obj.begin(SPI_com, SS_pin);
  lsm6dso_obj.set_mode_1(acc_fifo_wtm_val);

  //attachInterrupt(digitalPinToInterrupt(INT1_PIN), int1_fifo_wtm, RISING);

  //pinMode(OrangeLedPin, OUTPUT);
  //digitalWrite(OrangeLedPin, LOW);

  prev_open_file_time = millis() - time_between_open_close_file_miliseconds;

  /*-------------------SD Card BuiltIn slot------------------*/

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  //char filename[] = "dataLog_id_0000_nr_0000.TXT";
  for (uint8_t i = start_id_val; i < 10000; i++) {
    filename[11] = i / 1000 + '0';
    filename[12] = i / 100 + '0';
    filename[13] = i / 10 + '0';
    filename[14] = i % 10 + '0';
    if (!sd.exists(filename)) {
      // only open a new file if it doesn't exist
      break;
    }
  }
}

void create_new_file() {

  for (uint8_t i = 0; i < 10000; i++) {
    filename[19] = i / 1000 + '0';
    filename[20] = i / 100 + '0';
    filename[21] = i / 10 + '0';
    filename[22] = i % 10 + '0';
    if (!sd.exists(filename)) {
      // only open a new file if it doesn't exist
      break;
    }
  }
  // Open or create file - truncate existing file.
  if (!file.open(filename, O_WRITE | O_CREAT)) {
    Serial.println("setup - open failed\n");
    return;
  }
  /*// File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) {
    Serial.println("preAllocate failed\n");
    return;
  }*/

  // Write header.
  file.write(header, sizeof(header) - 1);

  byte header_params[header_params_byte_nr];
  int cnt_header_byte = 0;
  uint16_t sample_rate = 0;
  uint16_t sensitivity_1k = 0;
  uint16_t scale = 0;

  for (uint8_t i = 0; i < 3; i++) {
    //source sample rate
    sample_rate = lsm6dso_obj.get_acc_data_sampling_rate();
    header_params[cnt_header_byte++] = sample_rate >> 8;
    header_params[cnt_header_byte++] = (byte)sample_rate;
    //source sensitivity
    sensitivity_1k = lsm6dso_obj.get_acc_sensitivity();
    header_params[cnt_header_byte++] = sensitivity_1k >> 8;
    header_params[cnt_header_byte++] = (byte)sensitivity_1k;
    //source scale
    scale = lsm6dso_obj.get_acc_scale();
    header_params[cnt_header_byte++] = scale >> 8;
    header_params[cnt_header_byte++] = (byte)scale;
    //source samples group
    header_params[cnt_header_byte++] = acc_fifo_wtm_val >> 8;
    header_params[cnt_header_byte++] = (byte)acc_fifo_wtm_val;
  }
  /*//mic sample rate
  uint16_t mic_sample_rate = 0;
  header_params[cnt_header_byte++] = mic_sample_rate>>8;  
  header_params[cnt_header_byte++] = (byte) mic_sample_rate; 
  //mic sensitivity rate
  uint16_t mic_sensitivity_1k = 0;
  header_params[cnt_header_byte++] = mic_sensitivity_1k>>8;  
  header_params[cnt_header_byte++] = (byte) mic_sensitivity_1k; 
  //mic samples group
  uint16_t mic_wtm_val = 0;
  header_params[cnt_header_byte++] = mic_wtm_val>>8;  
  header_params[cnt_header_byte++] = (byte) mic_wtm_val;*/

  file.write(header_params, cnt_header_byte);

  // initialize the RingBuf.
  rb.begin(&file);
}

void loop() {

  if (((prev_open_file_time + time_between_open_close_file_miliseconds) < millis()) && (available_acc_data_counter == 0)) {
    if (file) {
      file.close();
      Serial.print("Closing file: ");
      Serial.println(filename);
    }
    prev_open_file_time = millis();
    create_new_file();
    Serial.print("Logging to: ");
    Serial.println(filename);
  }

  // FIFO sample number

  lsm6dso_obj.get_FIFO_STATUS();

  uint16_t fifo_sample_nr = lsm6dso_obj.check_FIFO_unread_samples();
  //Serial.print("FIFO sample nr: "); Serial.println(fifo_sample_nr);

  bool fifo_over_run_flag = lsm6dso_obj.check_FIFO_OVR();
  //Serial.print("FIFO over run flag: ");Serial.println(fifo_over_run_flag);

  bool fifo_wtm_flag_i2c = lsm6dso_obj.check_FIFO_WTM();
  //Serial.print("FIFO WTM flag: ");Serial.println(fifo_wtm_flag_i2c);

  bool fifo_full_flag = lsm6dso_obj.check_FIFO_FULL();
  //Serial.print("FIFO Full "); Serial.println(fifo_full_flag);

  // read FIFO data
  if (fifo_wtm_flag_i2c || fifo_over_run_flag) {
    logFIFOData();
  }
}
