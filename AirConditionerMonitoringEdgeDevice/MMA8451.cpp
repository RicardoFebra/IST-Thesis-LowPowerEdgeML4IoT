#include "MMA8451.h"

/**
 * @brief Construct a new MMA8451::MMA8451 object
 * 
 * @param received_data {byte *} - pointer to the array where the data received from the I2C bus will be stored
 */
MMA8451::MMA8451(byte *received_data){
    I2C_Handler I2C(received_data,MMA8451::DEVICE_ID,0x1A);
    this->_received_data = received_data;
    this->_i2c = I2C;
    this->_f_mode = 0x00;

    this->_acc_sampling_rate = 0;
    this->_acc_scale = 0;
    this->_acc_sensitivity = 0;

    this->_axis_aligned_index = -1;
    this->_axis_aligned_direction = false;
}

/**
 * @brief Check if the device is correctly connected and configured
 * 
 * @return true 
 * @return false 
 */
bool MMA8451::check_who_am_i(){
    this->_i2c.read(MMA8451::WHO_AM_I,1);

    if(this->_received_data[0] == MMA8451::WHO_AM_I_VAL){
        #ifdef INFO_ACC
            Serial.println("[INFO ][ACC] MMA8451::check_who_am_i() - DEVICE_ID matches");
        #endif
        return false;
    }else{
        #ifdef INFO_ACC
            Serial.println("[INFO ][ACC] MMA8451::check_who_am_i() - DEVICE_ID does not match");
        #endif
        return true;
    }
}

/**
 * @brief Initialize the device in mode 1, that is the main mode used for data acquisition
 * 
 * @param fifo_wtm_val {uint8_t} - FIFO watermark value (max 32)
 * @param sampling_rate {uint16_t} - sampling rate in Hz (max 800)
 * @param scale {uint16_t} - scale in G (max 8)
 */
void MMA8451::set_mode_1(uint8_t fifo_wtm_val, uint16_t sampling_rate, uint16_t scale){
    #ifdef INFO_ACC
        Serial.println("[INFO ][ACC] MMA8451::set_mode_1() - setting mode 1");
    #endif
    byte aux_sampling_rate = this->get_sampling_rate_bin(sampling_rate);
    byte aux_scale = this->get_scale_bin(scale);

    // CTRL_REG2 - reset
    this->set_CTRL_REG2(MMA8451::ST_DISBLE,MMA8451::RST_ENABLE,MMA8451::SLEEPMODS_NORMAL,MMA8451::SLPE_DISABLE, MMA8451::MODS_NORMAL);
    // wait for reset to finish
    while(this->check_who_am_i());
    // empty FIFO
    this->set_F_SETUP(MMA8451::FIFO_OFF, fifo_wtm_val);
    // standby mode to change settings
    this->set_CTRL_REG1(MMA8451::ASLP_RATE_1_56Hz, aux_sampling_rate, MMA8451::LNOISE_NORMAL, MMA8451::F_READ_NORMAL, MMA8451::STANDBY);

    this->set_XYZ_DATA_CFG(MMA8451::HPF_OUT_OFF, aux_scale);
    this->set_CTRL_REG2(MMA8451::ST_DISBLE,MMA8451::RST_DISABLE,MMA8451::SLEEPMODS_NORMAL,MMA8451::SLPE_DISABLE, MMA8451::MODS_NORMAL);
    this->set_F_SETUP(MMA8451::FIFO_CIRCULAR, fifo_wtm_val);
    this->set_CTRL_REG3(MMA8451::FIFO_GATE_DISABLE, MMA8451::WAKE_TRANS_SLEEP_MODE_DISABLE, MMA8451::WAKE_LNDPRT_SLEEP_MODE_DISABLE, MMA8451::WAKE_PULSE_SLEEP_MODE_DISABLE, MMA8451::WAKE_FF_MT_SLEEP_MODE_DISABLE, MMA8451::IPOL_ACTIVE_HIGH, MMA8451::PP_OD_PUSH_PULL);
    this->set_CTRL_REG4(MMA8451::INT_EN_ASLP_DISABLE, MMA8451::INT_EN_FIFO_ENABLE, MMA8451::INT_EN_TRANS_DISABLE, MMA8451::INT_EN_LNDPRT_DISABLE, MMA8451::INT_EN_PULSE_DISABLE, MMA8451::INT_EN_FF_MT_DISABLE, MMA8451::INT_EN_DRDY_ENABLE);
    this->set_CTRL_REG5(MMA8451::INT_CFG_ASLP_INT2, MMA8451::INT_CFG_FIFO_INT1, MMA8451::INT_CFG_TRANS_INT2, MMA8451::INT_CFG_LNDPRT_INT2, MMA8451::INT_CFG_PULSE_INT2, MMA8451::INT_CFG_FF_MT_INT2, MMA8451::INT_CFG_DRDY_INT1);
    
    // active mode to start reading data
    this->set_CTRL_REG1(MMA8451::ASLP_RATE_1_56Hz, aux_sampling_rate, MMA8451::LNOISE_NORMAL, MMA8451::F_READ_NORMAL, MMA8451::ACTIVE);

    #ifdef INFO_ACC
        Serial.print("[INFO ][ACC] MMA8451::get_acc_data() - Sensitivity: ");
        Serial.println(this->_acc_sensitivity);
    #endif
}

/**
 * @brief Initialize the device in mode 2, that is the main mode used to save power
 * 
 */
void MMA8451::set_sleep_mode_1(){
    #ifdef INFO_ACC
        Serial.println("[INFO ][ACC] MMA8451::set_sleep_mode_1() - setting sleep mode 1");
    #endif
    // CTRL_REG2 - reset
    this->set_CTRL_REG2(MMA8451::ST_DISBLE,MMA8451::RST_ENABLE,MMA8451::SLEEPMODS_NORMAL,MMA8451::SLPE_ENABLE, MMA8451::MODS_NORMAL);
    // wait for reset to finish
    while(this->check_who_am_i());
    // empty FIFO
    this->set_F_SETUP(MMA8451::FIFO_OFF,1);
    // standby mode to change settings
    this->set_CTRL_REG1(MMA8451::ASLP_RATE_1_56Hz, MMA8451::AccODR_1_56Hz, MMA8451::LNOISE_NORMAL, MMA8451::F_READ_NORMAL, MMA8451::STANDBY);

    this->set_XYZ_DATA_CFG(MMA8451::HPF_OUT_OFF, MMA8451::AccScale_2G);
    this->set_CTRL_REG2(MMA8451::ST_DISBLE,MMA8451::RST_DISABLE,MMA8451::SLEEPMODS_LOW_POWER,MMA8451::SLPE_ENABLE, MMA8451::MODS_LOW_POWER);
    this->set_CTRL_REG4(MMA8451::INT_EN_ASLP_DISABLE, MMA8451::INT_EN_FIFO_DISABLE, MMA8451::INT_EN_TRANS_DISABLE, MMA8451::INT_EN_LNDPRT_DISABLE, MMA8451::INT_EN_PULSE_DISABLE, MMA8451::INT_EN_FF_MT_DISABLE, MMA8451::INT_EN_DRDY_DISABLE);
    this->set_CTRL_REG5(MMA8451::INT_CFG_ASLP_INT2, MMA8451::INT_CFG_FIFO_INT2, MMA8451::INT_CFG_TRANS_INT2, MMA8451::INT_CFG_LNDPRT_INT2, MMA8451::INT_CFG_PULSE_INT2, MMA8451::INT_CFG_FF_MT_INT2, MMA8451::INT_CFG_DRDY_INT2);
}

/**
 * @brief Configure F_SETUP register
 * 
 * @param F_MODE {byte} - FIFO mode (OFF, FIFO, STREAM, TRIGGER)
 * @param F_WMRK {byte} - FIFO watermark value (max 32)
 */
void MMA8451::set_F_SETUP(byte F_MODE, byte F_WMRK){
    // set FIFO mode and watermark
    byte combined_data = ((F_MODE & 0b00000011)<<6) | (F_WMRK & 0b00111111);
    this->_f_mode = F_MODE;
    this->_i2c.write(MMA8451::F_SETUP, combined_data);
}

/**
 * @brief Configure XYZ_DATA_CFG register
 * 
 * @param HPF_OUT {byte} - High-pass filter on or off
 * @param FS {byte} - Full-scale range (2, 4, 8 G)
 */
void MMA8451::set_XYZ_DATA_CFG(byte HPF_OUT, byte FS){
    // set accelerometer scale
    byte combined_data = (0b000<<5) | (HPF_OUT<<4) | (0b00<<2) | (FS);

    this->_i2c.write(MMA8451::XYZ_DATA_CFG, combined_data);

    switch (FS)
    {
        case MMA8451::AccScale_8G:
            this->_acc_scale = 8; //datasheet value - Mechanical characteristics - Sensitivity - 1024 LSB/mg
            this->_acc_sensitivity = 0.9765625f; //datasheet value - Mechanical characteristics - Sensitivity - 1024 LSB/mg
            break;
        case MMA8451::AccScale_4G:
            this->_acc_scale = 4; //datasheet value - Mechanical characteristics - Sensitivity - 2048 LSB/g
            this->_acc_sensitivity = 0.48828125f; //datasheet value - Mechanical characteristics - Sensitivity - 2048 LSB/g
            break;
        case MMA8451::AccScale_2G:
            this->_acc_scale = 2; //datasheet value - Mechanical characteristics - Sensitivity - 4096 LSB/mg
            this->_acc_sensitivity = 0.244140625f; //datasheet value - Mechanical characteristics - Sensitivity - 4096 LSB/mg
            break;
    }
}

/**
 * @brief Configure the axis aligned with gravity force to remove the gravity component from the acceleration data
 * 
 * @param axis_aligned_index {byte} - index of the axis aligned with gravity force (0, 1, 2)
 * @param axis_aligned_direction {bool} - direction of the axis aligned with gravity force (true - positive, false - negative)
 */
void MMA8451::set_gravity_compensation(byte axis_aligned_index, bool axis_aligned_direction){
    this->_axis_aligned_index = axis_aligned_index;
    this->_axis_aligned_direction = axis_aligned_direction;
}

/**
 * @brief Configure the offset values to remove the gravity component from the acceleration data. If the accelerometer is tiled in more than one axis, the gravity component is present in more than one axis.
 * 
 * @param acc_offset {int16_t *} - pointer to the array where the offset values will be stored
 */
void MMA8451::set_offset_value_with_current_vals(){

    int16_t acc_aux_vals[3]={0,0,0};

    this->get_acc_data_raw(acc_aux_vals);
    this->_acc_offset[0] = acc_aux_vals[0];
    this->_acc_offset[1] = acc_aux_vals[1];
    this->_acc_offset[2] = acc_aux_vals[2];

    #ifdef INFO_ACC
        Serial.print("[INFO ][ACC] MMA8451::set_offset_value_with_current_vals() - offset values: ");
        Serial.print(this->_acc_offset[0]);
        Serial.print(" ");
        Serial.print(this->_acc_offset[1]);
        Serial.print(" ");
        Serial.println(this->_acc_offset[2]);
    #endif
}

void MMA8451::get_SYSMOD(byte FGERR, byte FGT, byte SYSMOD){
    // SYSMOD - get system mode
    this->_i2c.read(MMA8451::SYSMOD,1);
    FGERR = (this->_received_data[0] & 0b10000000) >> 7;
    FGT = (this->_received_data[0] & 0b01111100) >> 6;
    SYSMOD = (this->_received_data[0] & 0b00000011);
}

void MMA8451::set_CTRL_REG1(byte ASLP_RATE, byte DR, byte LNOISE, byte F_READ, byte ACTIVE){
    // CTRL_REG1 - set active mode, ODR
    byte combined_data = (ASLP_RATE<<6) | (DR<<3)| (LNOISE<<2) | (F_READ<<1) | (ACTIVE);

    this->_i2c.write(MMA8451::CTRL_REG1, combined_data);

    switch (DR)
    {
        case MMA8451::AccODR_800Hz:
            this->_acc_sampling_rate = 800;
            break;
        case MMA8451::AccODR_400Hz:
            this->_acc_sampling_rate = 400;
            break;
        case MMA8451::AccODR_200Hz:
            this->_acc_sampling_rate = 200;
            break;
        case MMA8451::AccODR_100Hz:
            this->_acc_sampling_rate = 100;
            break;
        case MMA8451::AccODR_50Hz:
            this->_acc_sampling_rate = 50;
            break;
        case MMA8451::AccODR_12_5Hz:
            this->_acc_sampling_rate = 12;
            break;
        case MMA8451::AccODR_6_25Hz:
            this->_acc_sampling_rate = 6;
            break;  
        case MMA8451::AccODR_1_56Hz:
            this->_acc_sampling_rate = 1;
            break;
    }
}

void MMA8451::set_CTRL_REG2(byte ST, byte RST, byte SLEEP, byte SLPE, byte MODS){
    // CTRL_REG2 - set sleep mode, sleep enable, reset
    byte combined_data = (ST<<7) | (RST<<6) | (SLEEP<<5) | (SLPE<<4) | (MODS<<2);
    this->_i2c.write(MMA8451::CTRL_REG2, combined_data);
}

void MMA8451::set_CTRL_REG3(byte FIFO_GATE, byte WAKE_TRANS, byte WAKE_LNDPRT, byte WAKE_PULSE, byte WAKE_FF_MT, byte IPOL, byte PP_OD){
    byte combined_data = (FIFO_GATE << 7) | (WAKE_TRANS << 6) | (WAKE_LNDPRT << 5) | (WAKE_PULSE << 4) | (WAKE_FF_MT << 3) | (0b0 <<2) |(IPOL << 1) | (PP_OD);
    this->_i2c.write(MMA8451::CTRL_REG3, combined_data);
}

void MMA8451::set_CTRL_REG4(byte INT_EN_ASLP, byte INT_EN_FIFO, byte INT_EN_TRANS, byte INT_EN_LNDPRT, byte INT_EN_PULSE, byte INT_EN_FF_MT, byte INT_EN_DRDY){
    
    byte combined_data = (INT_EN_ASLP<<7) | (INT_EN_FIFO<<6) | (INT_EN_TRANS<<5) | (INT_EN_LNDPRT<<4) | (INT_EN_PULSE<<3) | (INT_EN_FF_MT<<2) | (0b0<<1) |(INT_EN_DRDY);
    this->_i2c.write(MMA8451::CTRL_REG4, combined_data);
}

void MMA8451::set_CTRL_REG5(byte INT_CFG_ASLP, byte INT_CFG_FIFO, byte INT_CFG_TRANS, byte INT_CFG_LNDPRT, byte INT_CFG_PULSE, byte INT_CFG_FF_MT, byte INT_CFG_DRDY){

    byte combined_data = (INT_CFG_ASLP<<7) | (INT_CFG_FIFO<<6) | (INT_CFG_TRANS<<5) | (INT_CFG_LNDPRT<<4) | (INT_CFG_PULSE<<3) | (INT_CFG_FF_MT<<2) | (0b0<<1) |(INT_CFG_DRDY);
    this->_i2c.write(MMA8451::CTRL_REG5, combined_data);
}

void MMA8451::get_INT_SOURCE(){
    this->_i2c.read(MMA8451::INT_SOURCE,1);
}

bool MMA8451::check_INT_SOURCE_FIFO(){
    int fifo_int = (this->_received_data[0] & 0b01000000) >> 6;
    if (fifo_int == 1){
        return true;
    }else{
        return false;
    }
}

void MMA8451::get_DATA_STATUS(){
    this->_i2c.read(MMA8451::DATA_STATUS,1);
}

bool MMA8451::check_FIFO_WTM_status(){
    int fifo_wtm = (this->_received_data[0] & 0b01000000) >> 6;
    if (fifo_wtm == 1){
        return true;
    }else{
        return false;
    }
}

bool MMA8451::check_FIFO_over_run_status(){
    int fifo_over_run = (this->_received_data[0] & 0b10000000) >> 7;
    if (fifo_over_run == 1){
        return true;
    }else{
        return false;
    }
}

uint16_t MMA8451::check_FIFO_unread_samples(){
    int fifo_sample_nr = (this->_received_data[0] & 0b00111111);
    return fifo_sample_nr;
}

void MMA8451::get_acc_data(int16_t *data){
    
    int16_t acc_aux_vals[3] = {0,0,0};

    this->get_acc_data_raw(acc_aux_vals);

    float aux_data = 0;
    for (int i=0;i<3;i++){
        aux_data = ((acc_aux_vals[i]- this->_acc_offset[i]) * this->_acc_sensitivity);
        if (data[i]>=0){data[i]=static_cast<int16_t>(aux_data+0.5);}else{data[i]=static_cast<int16_t>(aux_data-0.5);}

        if (this->_axis_aligned_index == i){
            if (this->_axis_aligned_direction){
                data[i] = data[i] + 1000;
            }else{
                data[i] = data[i] - 1000;
            }
        }
    }

    #ifdef DEBUG_ACC
        Serial.print("[DEBUG][ACC] MMA8451::get_acc_data() - ");
        Serial.print(data[0]);
        Serial.print(" ");
        Serial.print(data[1]);
        Serial.print(" ");
        Serial.println(data[2]);
    #endif
}

void MMA8451::get_acc_data_raw(int16_t *data){

    this->_i2c.read(MMA8451::OUT_X_MSB,6);

    for (int i = 0; i < 3; i++){
      data[i] = (this->_received_data[i*2] << 8) | this->_received_data[i*2+1];
      //Aritmetic shift right 2 bits to get rid of the 2 LSBs and mantain the sign bit
      data[i] >>= 2;
    }

    #ifdef DEBUG_ACC
        Serial.print("[DEBUG][ACC] MMA8451::get_acc_data_raw() - ");
        Serial.print(data[0]);
        Serial.print(" ");
        Serial.print(data[1]);
        Serial.print(" ");
        Serial.println(data[2]);
    #endif

}

uint16_t MMA8451::get_acc_sensitivity(){
    return this->_acc_sensitivity*1000;
}

uint16_t MMA8451::get_acc_data_sampling_rate(){
    return this->_acc_sampling_rate;
}

uint16_t MMA8451::get_acc_scale(){
    return this->_acc_scale;
}

byte MMA8451::get_sampling_rate_bin(uint16_t sampling_rate){
    if (sampling_rate >= 800){
        return MMA8451::AccODR_800Hz;
    }else if (sampling_rate >= 400){
        return MMA8451::AccODR_400Hz;
    }else if (sampling_rate >= 200){
        return MMA8451::AccODR_200Hz;
    }else if (sampling_rate >= 100){
        return MMA8451::AccODR_100Hz;
    }else if (sampling_rate >= 50){
        return MMA8451::AccODR_50Hz;
    }else if (sampling_rate >= 12){
        return MMA8451::AccODR_12_5Hz;
    }else if (sampling_rate >= 6){
        return MMA8451::AccODR_6_25Hz;
    }else{
        return MMA8451::AccODR_1_56Hz;
    }
}

byte MMA8451::get_scale_bin(uint16_t scale){
    if (scale >= 8){
        return MMA8451::AccScale_8G;
    }else if (scale >= 4){
        return MMA8451::AccScale_4G;
    }else{
        return MMA8451::AccScale_2G;
    }
}