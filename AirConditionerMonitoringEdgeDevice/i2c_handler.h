#include <Arduino.h>
#include <Wire.h>

// Define the debug level and prints for the I2C communication
//#define DEBUG_I2C
//#define INFO_I2C

#ifdef DEBUG_I2C
    #define INFO_I2C 
#endif
// End of debug level and prints for the I2C communication

#ifndef i2c_handler_h
#define i2c_handler_h

class I2C_Handler
{
    public:
        I2C_Handler();
        I2C_Handler(byte *received_data, byte DEVICE_ID, byte WHO_AM_I_VAL);
        I2C_Handler(byte *received_data, byte DEVICE_ID);
        int check_who_am_i(byte REG_WHO_AM_I);
        void write(byte reg, byte data, uint8_t len = 1);
        void read(byte reg, uint8_t request_bytes = 1, uint8_t first_position = 0);
        byte *_received_data;
    private:
        uint8_t _nr_slaves;
        uint8_t _address;
        uint8_t _reg;
        uint8_t _data;
        uint8_t _len;
        uint8_t _status;
        uint8_t _error;
        uint8_t _count;
        uint8_t _stop;
        byte _device_id;
        byte _who_am_i_val;
};

#endif
