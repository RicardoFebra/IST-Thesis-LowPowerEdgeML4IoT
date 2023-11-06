#include "LSM6DSO.h"

LSM6DSO::LSM6DSO(byte *received_data){
    _received_data = received_data;

    I2C_Handler I2C_obj(received_data,DEVICE_ID);
    this->_i2c = I2C_obj;

    SPI_Handler SPI_obj(received_data,DEVICE_ID);
    this->_spi = SPI_obj;

    this->_acc_sampling_rate = 0;
    this->_acc_scale = 0;
    this->_acc_sensitivity = 0;
}

LSM6DSO::~LSM6DSO(){
    delete _received_data;
}  

void LSM6DSO::begin(byte communication_protocol){
  this->_communication_protocol = communication_protocol;
  this->_spi.set_SS_pin(5);
  this->_who_am_i_val = DEVICE_ID;
}

void LSM6DSO::begin(byte communication_protocol, int SS_pin){
  this->_communication_protocol = communication_protocol;
  this->_spi.set_SS_pin(SS_pin);
  this->_who_am_i_val = DEVICE_ID;
}

void LSM6DSO::begin(byte communication_protocol, int SS_pin, byte WHO_AM_I_VAL){
  this->_communication_protocol = communication_protocol;
  this->_spi.set_SS_pin(SS_pin);
  this->_who_am_i_val = WHO_AM_I_VAL;
}

bool LSM6DSO::check_who_am_i(){

    byte register_addr = LSM6DSO::WHO_AM_I;
    uint8_t request_bytes = 1;

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::check_who_am_i() - request_bytes: "); Serial.println(request_bytes);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.read(register_addr,request_bytes);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr | 0x80;
        this->_spi.read(register_addr,request_bytes);
    }

    if (this->_received_data[0] == this->_who_am_i_val){
        #ifdef INFO_LSM6DSO
            Serial.println("[INFO][LSM6DSO] LSM6DSO::check_who_am_i() - WHO_AM_I OK");
        #endif
        return false;
    }
    else{
        #ifdef INFO_LSM6DSO
            Serial.println("[INFO][LSM6DSO] LSM6DSO::check_who_am_i() - WHO_AM_I NOT OK");
            Serial.print("[INFO][LSM6DSO] LSM6DSO::check_who_am_i() - WHO_AM_I_VAL: "); Serial.println(this->_who_am_i_val, HEX);
        #endif
        return true;
    }
}

void LSM6DSO::set_mode_1(uint16_t acc_fifo_wtm_val){
    //"After power up, the trimming parameters can be re-loaded by setting the BOOT bit of the CTRL3_C register to 1"
    this->set_CTRL3_C(LSM6DSO::BOOT_REBOOT_MODE, LSM6DSO::BDU_CONTINUOS_UPDATE, LSM6DSO::H_LACTIVE_HIGH, LSM6DSO::PP_OD_PUSH_PULL, LSM6DSO::SIM_4WIRE, LSM6DSO::IF_INC_ENABLED, LSM6DSO::SW_RESET_NORMAL_MODE);
    this->set_FIFO_CTRL1(acc_fifo_wtm_val);
    this->set_FIFO_CTRL2(LSM6DSO::STOP_ON_WTM_DISABLED, LSM6DSO::FIFO_COMPR_RT_DISABLED, LSM6DSO::ODRCHG_DISABLED, LSM6DSO::UNCOPTR_RATE_DISABLED, acc_fifo_wtm_val);
    this->set_FIFO_CTRL3(LSM6DSO::FIFO_BDR_GY_DISABLED, LSM6DSO::FIFO_BDR_XL_6667Hz);
    this->set_FIFO_CTRL4(LSM6DSO::DEC_TS_BATCH_DISABLED, LSM6DSO::ODR_T_BATCH_DISABLED, LSM6DSO::FIFO_MODE_CONTINUOUS);
    this->set_CTRL1_XL(LSM6DSO::XL_ODR_6666Hz, LSM6DSO::XL_FS_16g_2g, LSM6DSO::LPF2_XL_EN_DISABLED);
    this->set_CTRL2_G(LSM6DSO::G_ODR_POWER_DOWN, LSM6DSO::FS_G_2000dps, LSM6DSO::FS_125_DISABLED);
    this->set_CTRL4_C(LSM6DSO::SLEEP_G_ENABLED, LSM6DSO::INT2_on_INT1_ENABLED, LSM6DSO::DRDY_MASK_DISABLED, LSM6DSO::I2C_DISABLE_DISABLED, LSM6DSO::LPF1_SEL_G_DISABLED);
    this->set_CTRL5_C(LSM6DSO::XL_ULP_EN_DISABLED, LSM6DSO::ROUNDING_DISABLED, LSM6DSO::ST_G_DISABLED, LSM6DSO::ST_XL_DISABLED);
    this->set_CTRL6_C(LSM6DSO::TRIG_EN_DISABLED, LSM6DSO::LVL1_EN_DISABLED, LSM6DSO::LVL2_EN_DISABLED, LSM6DSO::XL_HM_MODE_ENABLED, LSM6DSO::USR_OFF_W_DISABLED, LSM6DSO::FTYPE_2_DISABLED, LSM6DSO::FTYPE_1_DISABLED, LSM6DSO::FTYPE_0_DISABLED);
    this->set_CTRL8_XL(LSM6DSO::HPCF_XL_2_CONFIG1, LSM6DSO::HP_REF_MODE_DISABLED, LSM6DSO::FASTSETTL_MODE_XL_DISABLED, LSM6DSO::HP_SLOPE_XL_EN_DISABLED, LSM6DSO::XL_FS_MODE_DISABLED, LSM6DSO::LOW_PASS_ON_6D_DISABLED);

    this->set_FIFO_CTRL4(LSM6DSO::DEC_TS_BATCH_DISABLED, LSM6DSO::ODR_T_BATCH_DISABLED, LSM6DSO::FIFO_MODE_DISABLED);
    this->set_FIFO_CTRL4(LSM6DSO::DEC_TS_BATCH_DISABLED, LSM6DSO::ODR_T_BATCH_DISABLED, LSM6DSO::FIFO_MODE_CONTINUOUS);

    this->set_INT1_CTRL(LSM6DSO::DEN_DRDY_DISABLED, LSM6DSO::INT1_CNT_BDR_DISABLED, LSM6DSO::INT1_FIFO_FULL_DISABLED, LSM6DSO::INT1_FIFO_OVR_DISABLED, LSM6DSO::INT1_FIFO_TH_ENABLED, LSM6DSO::INT1_BOOT_DISABLED, LSM6DSO::INT1_DRDY_G_DISABLED, LSM6DSO::INT1_DRDY_XL_DISABLED);
    if(this->check_who_am_i()){
        while(1);
    }
  this->set_acc_offset();
}

void LSM6DSO::set_sleep_mode(){
    this->set_CTRL3_C(LSM6DSO::BOOT_REBOOT_MODE, LSM6DSO::BDU_CONTINUOS_UPDATE, LSM6DSO::H_LACTIVE_HIGH, LSM6DSO::PP_OD_PUSH_PULL, LSM6DSO::SIM_4WIRE, LSM6DSO::IF_INC_ENABLED, LSM6DSO::SW_RESET_NORMAL_MODE);
    //"After the device is powered up, it performs a 10 ms (maximum) boot procedure to load the trimming parameters"
    this->set_FIFO_CTRL4(LSM6DSO::DEC_TS_BATCH_DISABLED, LSM6DSO::ODR_T_BATCH_DISABLED, LSM6DSO::FIFO_MODE_DISABLED);
    this->set_CTRL1_XL(LSM6DSO::XL_ODR_POWER_DOWN, LSM6DSO::XL_FS_16g_2g, LSM6DSO::LPF2_XL_EN_DISABLED);
    this->set_CTRL2_G(LSM6DSO::G_ODR_POWER_DOWN, LSM6DSO::FS_G_2000dps, LSM6DSO::FS_125_DISABLED);
    delay(15);

/*    this->set_CTRL3_C(LSM6DSO::BOOT_NORMAL_MODE, LSM6DSO::BDU_CONTINUOS_UPDATE, LSM6DSO::H_LACTIVE_HIGH, LSM6DSO::PP_OD_PUSH_PULL, LSM6DSO::SIM_4WIRE, LSM6DSO::IF_INC_ENABLED, LSM6DSO::SW_RESET_RESET_DEVICE);
    //"Wait 50 µs"
    delay(60);*/
    //this->set_CTRL1_XL(LSM6DSO::XL_ODR_1_6Hz, LSM6DSO::XL_FS_16g_2g, LSM6DSO::LPF2_XL_EN_DISABLED);
    //this->set_CTRL6_C(LSM6DSO::TRIG_EN_DISABLED, LSM6DSO::LVL1_EN_DISABLED, LSM6DSO::LVL2_EN_DISABLED, LSM6DSO::XL_HM_MODE_DISABLED, LSM6DSO::USR_OFF_W_DISABLED, LSM6DSO::FTYPE_2_DISABLED, LSM6DSO::FTYPE_1_DISABLED, LSM6DSO::FTYPE_0_DISABLED);
    //this->set_CTRL2_G(LSM6DSO::G_ODR_POWER_DOWN, LSM6DSO::FS_G_2000dps, LSM6DSO::FS_125_DISABLED);
    //this->set_FIFO_CTRL3(LSM6DSO::FIFO_BDR_GY_DISABLED, LSM6DSO::FIFO_BDR_XL_DISABLED);
    //this->set_FIFO_CTRL4(LSM6DSO::DEC_TS_BATCH_DISABLED, LSM6DSO::ODR_T_BATCH_DISABLED, LSM6DSO::FIFO_MODE_DISABLED);
    //this->set_INT1_CTRL(LSM6DSO::DEN_DRDY_DISABLED, LSM6DSO::INT1_CNT_BDR_DISABLED, LSM6DSO::INT1_FIFO_FULL_DISABLED, LSM6DSO::INT1_FIFO_OVR_DISABLED, LSM6DSO::INT1_FIFO_TH_DISABLED, LSM6DSO::INT1_BOOT_DISABLED, LSM6DSO::INT1_DRDY_G_DISABLED, LSM6DSO::INT1_DRDY_XL_DISABLED);
}

void LSM6DSO::set_acc_offset(){
    /*int16_t acc_data_x_axis = 0;
    int16_t acc_data_y_axis = 0;
    int16_t acc_data_z_axis = 0;
    int offset_sample_nr = 10;
    for (int i = 0; i < offset_sample_nr; i++){
      int16_t accelerometer[3];
      this->get_FIFO_DATA_OUT(accelerometer);
      acc_data_x_axis += accelerometer[0];
      acc_data_y_axis += accelerometer[1];
      acc_data_z_axis += accelerometer[2];
    }
    this->_acc_offset[0] = int16_t(acc_data_x_axis/offset_sample_nr);
    this->_acc_offset[1] = int16_t(acc_data_y_axis/offset_sample_nr);
    this->_acc_offset[2] = int16_t(acc_data_z_axis/offset_sample_nr);*/
    for(int i = 0; i<3;i++){
      if (i==1){
        this->_acc_offset[i]=1000;
      }else{
        this->_acc_offset[i]=0;      
      }
    }
    //Serial.print("ACC offset: ");Serial.print(this->_acc_offset[0]);Serial.print(" ");Serial.print(this->_acc_offset[1]);Serial.print(" ");Serial.println(this->_acc_offset[2]);
}

void LSM6DSO::set_FIFO_CTRL1(uint16_t WTM){
    byte register_addr = LSM6DSO::FIFO_CTRL1;
    byte data = WTM;

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_FIFO_CTRL1() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_FIFO_CTRL2(byte STOP_ON_WTM, byte FIFO_COMPR_RT_EN, byte ODRCHG_EN, byte UNCOPTR_RATE, uint16_t WTM_8){
    byte register_addr = LSM6DSO::FIFO_CTRL2;
    byte aux_wtm_8 = (WTM_8 >> 8) & 1;
    byte data = (STOP_ON_WTM << 7) | (FIFO_COMPR_RT_EN << 6) | (ODRCHG_EN << 4) | (UNCOPTR_RATE << 1) | (aux_wtm_8);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_FIFO_CTRL2() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_FIFO_CTRL3(byte BDR_GY, byte BDR_XL){
    byte register_addr = LSM6DSO::FIFO_CTRL3;
    byte data = (BDR_GY << 4) | (BDR_XL << 0);

    switch(BDR_XL){
        case 0:
            this->_acc_sampling_rate = 0;
            break;
        case 1:
            this->_acc_sampling_rate = 12;
            break;
        case 2:
            this->_acc_sampling_rate = 26;
            break;
        case 3:
            this->_acc_sampling_rate = 52;
            break;
        case 4:
            this->_acc_sampling_rate = 104;
            break;
        case 5:
            this->_acc_sampling_rate = 208;
            break;
        case 6:
            this->_acc_sampling_rate = 416;
            break;
        case 7:
            this->_acc_sampling_rate = 833;
            break;
        case 8:
            this->_acc_sampling_rate = 1667;
            break;
        case 9:
            this->_acc_sampling_rate = 3333;
            break;
        case 10:
            this->_acc_sampling_rate = 6667;
            break;
        case 11:
            this->_acc_sampling_rate = 1;
            break;
    }

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_FIFO_CTRL3() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_FIFO_CTRL4(byte DEC_TS_BATCH ,byte ODR_T_BATCH , byte FIFO_MODE){
    byte register_addr = LSM6DSO::FIFO_CTRL4;
    byte data = (DEC_TS_BATCH << 6) | (ODR_T_BATCH << 4) | (FIFO_MODE);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_FIFO_CTRL4() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_COUNTER_BDR_REG1(byte datareeady_pulsed, byte RST_COUNTER_BDR, byte TRIG_COUNTER_BDR, byte CNT_BDR_TH){
    byte register_addr = LSM6DSO::COUNTER_BDR_REG1;
    byte data = (datareeady_pulsed << 7) | (RST_COUNTER_BDR << 6) | (TRIG_COUNTER_BDR << 5) | (CNT_BDR_TH);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_COUNTER_BDR_REG1() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_COUNTER_BDR_REG2(byte CNT_BDR_TH){
    byte register_addr = LSM6DSO::COUNTER_BDR_REG2;
    byte data = (CNT_BDR_TH);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_COUNTER_BDR_REG2() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_INT1_CTRL(byte DEN_DRDY, byte INT1_CNT_BDR, byte INT1_FIFO_FULL, byte INT1_FIFO_OVR, byte INT1_FIFO_TH, byte INT1_BOOT, byte INT1_DRDY_G, byte INT1_DRDY_XL){
    byte register_addr = LSM6DSO::INT1_CTRL;
    byte data = (DEN_DRDY << 7) | (INT1_CNT_BDR << 6) | (INT1_FIFO_FULL << 5) | (INT1_FIFO_OVR << 4) | (INT1_FIFO_TH << 3) | (INT1_BOOT << 2) | (INT1_DRDY_G << 1) | (INT1_DRDY_XL);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_INT1_CTRL() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_INT2_CTRL(byte INT2_CTN_BDR, byte INT2_FIFO_FULL, byte INT2_FIFO_OVR, byte INT2_FIFO_TH, byte INT2_DRDY_TEMP, byte INT2_DRDY_G, byte INT2_DRDY_XL){
    byte register_addr = LSM6DSO::INT2_CTRL;
    byte data = (INT2_CTN_BDR << 6) | (INT2_FIFO_FULL << 5) | (INT2_FIFO_OVR << 4) | (INT2_FIFO_TH << 3) | (INT2_DRDY_TEMP << 2) | (INT2_DRDY_G << 1) | (INT2_DRDY_XL);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_INT2_CTRL() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_CTRL1_XL(byte ODR_XL, byte FS1_XL, byte LPF2_XL_EN){
    byte register_addr = LSM6DSO::CTRL1_XL;
    byte data = (ODR_XL << 4) | (FS1_XL << 2) | (LPF2_XL_EN << 1);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_CTRL_XL() - data: "); Serial.println(data);
    #endif

    switch (ODR_XL){
        case 0:
            this->_acc_sampling_rate = 0;
            break;
        case 1:
            this->_acc_sampling_rate = 12;
            break;
        case 2:
            this->_acc_sampling_rate = 26;
            break;
        case 3:
            this->_acc_sampling_rate = 52;
            break;
        case 4:
            this->_acc_sampling_rate = 104;
            break;
        case 5:
            this->_acc_sampling_rate = 208;
            break;
        case 6:
            this->_acc_sampling_rate = 416;
            break;
        case 7:
            this->_acc_sampling_rate = 833;
            break;
        case 8:
            this->_acc_sampling_rate = 1667;
            break;
        case 9:
            this->_acc_sampling_rate = 3333;
            break;
        case 10:
            this->_acc_sampling_rate = 6667;
            break;
        case 11:
            this->_acc_sampling_rate = 1;
            break;
    }

    switch (FS1_XL){
        case 0:
            this->_acc_scale = 2;
            this->_acc_sensitivity = 0.061;
            break;
        case 1:
            this->_acc_scale = 16;
            this->_acc_sensitivity = 0.488;
            break;
        case 2:
            this->_acc_scale = 4;
            this->_acc_sensitivity = 0.122;
            break;
        case 3:
            this->_acc_scale = 8;
            this->_acc_sensitivity = 0.244;
            break;
    }

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_CTRL2_G(byte ODR_G, byte FS_G, byte FS_125){
    byte register_addr = LSM6DSO::CTRL2_G;
    byte data = (ODR_G << 4) | (FS_G << 2) | (FS_125 << 1);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_CTRL2_G() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_CTRL3_C(byte BOOT, byte BDU, byte H_LACTIVE, byte PP_OD, byte SIM, byte IF_INC, byte SW_RESET){
    byte register_addr = LSM6DSO::CTRL3_C;
    byte data = (BOOT << 7) | (BDU << 6) | (H_LACTIVE << 5) | (PP_OD << 4) | (SIM << 3) | (IF_INC << 2) | (SW_RESET);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_CTRL3_C() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
      register_addr = register_addr & 0x7F;
      this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_CTRL4_C(byte SLEEP_G, byte INT2_on_INT1, byte DRDY_MASK, byte I2C_disable, byte LPF1_SEL_G){
    byte register_addr = LSM6DSO::CTRL4_C;
    byte data = (SLEEP_G << 6) | (INT2_on_INT1 << 5) | (DRDY_MASK << 3) | (I2C_disable << 2) | (LPF1_SEL_G << 1);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_CTRL4_C() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }
    else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_CTRL5_C(byte XL_ULP_EN, byte ROUNDING, byte ST_G, byte ST_XL){
    byte register_addr = LSM6DSO::CTRL5_C;
    byte data = (XL_ULP_EN << 7) | (ROUNDING << 5) | (ST_G << 2) | (ST_XL);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_CTRL5_C() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_CTRL6_C(byte TRIG_EN, byte LVL1_EN, byte LVL2_EN, byte XL_HM_MODE, byte USR_OFF_W, byte FTYPE_2, byte FTYPE_1, byte FTYPE_0){
    byte register_addr = LSM6DSO::CTRL6_C;
    byte data = (TRIG_EN << 7) | (LVL1_EN << 6) | (LVL2_EN << 5) | (XL_HM_MODE << 4) | (USR_OFF_W << 3) | (FTYPE_2 << 2) | (FTYPE_1 << 1) | (FTYPE_0);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_CTRL6_C() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr, data);
    }else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr, data);
    }
}

void LSM6DSO::set_CTRL8_XL(byte HPCF_XL_2, byte HP_REF_MODE, byte FASTSETTL_MODE_XL, byte HP_SLOPE_XL_EN, byte XL_FS_MODE, byte LOW_PASS_ON_6D){
    byte register_addr = LSM6DSO::CTRL8_XL;
    byte data = (HPCF_XL_2 << 5) | (HP_REF_MODE << 4) | (FASTSETTL_MODE_XL << 3) | (HP_SLOPE_XL_EN << 2) | (XL_FS_MODE << 1) | (LOW_PASS_ON_6D);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_CTRL8_CL() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::set_WAKE_UP_SRC(byte SLEEP_CHANGE_IA, byte FF_IA, byte SLEEP_STATE, byte WU_IA, byte X_WU, byte Y_WU, byte Z_WU){
    byte register_addr = LSM6DSO::WAKE_UP_SRC;
    byte data = (0 << 7) | (SLEEP_CHANGE_IA << 6) | (FF_IA << 5) | (SLEEP_STATE << 4) | (WU_IA << 3) | (X_WU << 2) | (Y_WU << 1) | (Z_WU);

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::set_WAKE_UP_SRC() - data: "); Serial.println(data);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.write(register_addr,data);
    }else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr & 0x7F;
        this->_spi.write(register_addr,data);
    }
}

void LSM6DSO::get_STATUS_REG(){
    byte register_addr = LSM6DSO::STATUS_REG;
    uint8_t request_bytes = 1;

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::get_STATUS_REG() - request_bytes: "); Serial.println(request_bytes);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.read(register_addr,request_bytes);
    }else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr | 0x80;
        this->_spi.read(register_addr,request_bytes);
    }
}

bool LSM6DSO::check_XLDA(){
    bool XLDA = this->_received_data[0] & 0x01;
    return XLDA;
}

void LSM6DSO::get_OUTX_A(int16_t *data){

    this->get_OUTX_A_raw(data);

    for (int i = 0; i < 3; i++){
        data[i] = (int16_t)(data[i] * this->_acc_sensitivity);
    }
}

void LSM6DSO::get_OUTX_A_raw(int16_t *data){

    byte register_addr = LSM6DSO::OUTX_L_A;
    uint8_t request_bytes = 6;

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::GET_OUTX_A_raw() - request_bytes: "); Serial.println(request_bytes);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.read(register_addr,request_bytes);
    }else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr | 0x80;
        this->_spi.read(register_addr,request_bytes);
    }

    for (int i = 0; i < 3; i++){
        data[i] = (int16_t)((this->_received_data[i*2+1] << 8) | this->_received_data[i*2]);
    }
}

void LSM6DSO::get_FIFO_STATUS(){
    byte register_addr = LSM6DSO::FIFO_STATUS1;
    uint8_t request_bytes = 2;

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::get_FIFO_STATUS() - request_bytes: "); Serial.println(request_bytes);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.read(register_addr,request_bytes);
    }else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr | 0x80;
        this->_spi.read(register_addr,2);
    }
}

bool LSM6DSO::check_FIFO_FULL(){
    bool FIFO_FULL = this->_received_data[1] & 0b00100000;
    return FIFO_FULL;
}

bool LSM6DSO::check_FIFO_OVR(){
    bool FIFO_OVR = this->_received_data[1] & 0b01000000;
    return FIFO_OVR;
}

bool LSM6DSO::check_FIFO_WTM(){
    bool FIFO_WTM = this->_received_data[1] & 0b10000000;
    return FIFO_WTM;
}

uint16_t LSM6DSO::check_FIFO_unread_samples(){
    uint16_t FIFO_unread_samples = (uint16_t)(((this->_received_data[1] & 0b00000011) << 8 ) | this->_received_data[0]);
    return FIFO_unread_samples;
}

void LSM6DSO::get_FIFO_DATA_OUT(int16_t *data){

    this->get_FIFO_DATA_OUT_raw(data);

    for (int i = 0; i < 3; i++){
        data[i] = (int16_t)(data[i] * this->_acc_sensitivity - this->_acc_offset[i]);
    }
}

void LSM6DSO::get_FIFO_DATA_OUT_raw(int16_t *data){

    byte register_addr = LSM6DSO::FIFO_DATA_OUT_X_L;
    uint8_t request_bytes = 6;

    #ifdef DEBUG_LSM6DSO
        Serial.print("[DEBUG][LSM6DSO] LSM6DSO::get_FIFO_DATA_OUT_raw() - request_bytes: "); Serial.println(request_bytes);
    #endif

    if(this->_communication_protocol == I2C_com){
        this->_i2c.read(register_addr,request_bytes);
    }else if(this->_communication_protocol == SPI_com){
        register_addr = register_addr | 0x80;
        this->_spi.read(register_addr,request_bytes);
    }

    for (int i = 0; i < 3; i++){
        data[i] = (int16_t)((this->_received_data[i*2+1] << 8) | this->_received_data[i*2]);
    }

}

uint16_t LSM6DSO::get_acc_data_sampling_rate(){
    return this->_acc_sampling_rate;
}

uint16_t LSM6DSO::get_acc_scale(){
    return this->_acc_scale;
}

uint16_t LSM6DSO::get_acc_sensitivity(){
    return static_cast<int>((this->_acc_sensitivity)*1000);
}