#include "i2c_handler.h"

I2C_Handler::I2C_Handler()
{
    this->_received_data = NULL;
    this->_device_id = 0;
    
}

I2C_Handler::I2C_Handler(byte *received_data, byte DEVICE_ID)
{
    this->_received_data = received_data;
    this->_device_id = DEVICE_ID;
    this->_who_am_i_val = DEVICE_ID;
}

I2C_Handler::I2C_Handler(byte *received_data, byte DEVICE_ID, byte WHO_AM_I_VAL)
{
    this->_received_data = received_data;
    this->_device_id = DEVICE_ID;
    this->_who_am_i_val = WHO_AM_I_VAL;
}

int I2C_Handler::check_who_am_i(byte REG_WHO_AM_I)
{
    byte reg = REG_WHO_AM_I;
    uint8_t request_bytes = sizeof(reg);

    #ifdef DEBUG_I2C
        Serial.print("[DEBUG][I2C] I2C_Handler::check_who_am_i() - request_bytes: "); Serial.println(request_bytes);
    #endif

    byte data[request_bytes];

    Wire.beginTransmission(this->_device_id);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(this->_device_id, request_bytes);
    uint8_t i = 0;
    while (Wire.available()) { // peripheral may send less than requested
        data[i] = Wire.read(); // receive a byte as character
        i++;
        #ifdef DEBUG_I2C
            Serial.print("[DEBUG][I2C] I2C_Handler::check_who_am_i() - received byte: ");Serial.println(data[i], HEX);
        #endif
    }
    Wire.endTransmission();
    // compare the data with the DEVICE_ID
    if (data[0] == this->_who_am_i_val)
    {   
        return 0;
        #ifdef INFO_I2C
            Serial.println("[INFO ][I2C] I2C_Handler::check_who_am_i() - DEVICE_ID matches");
        #endif
    }
    else
    {
        return 1;
        #ifdef INFO_I2C
            Serial.println("[INFO ][I2C] I2C_Handler::check_who_am_i() - DEVICE_ID does not match");
        #endif
    }  
}

void I2C_Handler::write(byte reg, byte data, uint8_t len)
{
    Wire.beginTransmission(this->_device_id);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
    #ifdef DEBUG_I2C
        Serial.print("[DEBUG][I2C] I2C_Handler::write()");Serial.print(" - register: ");Serial.print(reg,HEX);Serial.print(" - data: ");Serial.print(data,BIN);Serial.print(" - len: ");Serial.println(len);
    #endif
}

void I2C_Handler::read(byte reg, uint8_t request_bytes, uint8_t first_position)
{

    #ifdef DEBUG_I2C
        Serial.print("[DEBUG][I2C] I2C_Handler::read()");Serial.print(" - register: ");Serial.print(reg,HEX);Serial.print(" - request_bytes: "); Serial.println(request_bytes);
    #endif

    Wire.beginTransmission(this->_device_id);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(this->_device_id, request_bytes);
    // create an array to store the data with size of the request
    uint8_t i = first_position;
    while (Wire.available()) { // peripheral may send less than requested
        this->_received_data[i] = Wire.read(); // receive a byte as character
        #ifdef DEBUG_I2C
            Serial.print("[DEBUG][I2C] I2C_Handler::read() - received byte: ");Serial.println(this->_received_data[i], HEX);
        #endif
        i++;
    }
    Wire.endTransmission();
}
