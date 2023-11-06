#include <Arduino.h>
#include <SPI.h>

// Define the debug level and prints for the SPI communication
//#define DEBUG_SPI
//#define INFO_SPI

#ifdef DEBUG_SPI
    #define INFO_SPI 
#endif
// End of debug level and prints for the SPI communication

#ifndef SPI_handler_h
#define SPI_handler_h

class SPI_Handler
{
    public:
        SPI_Handler();
        SPI_Handler(byte *received_data, byte DEVICE_ID, byte WHO_AM_I_VAL);
        SPI_Handler(byte *received_data, byte DEVICE_ID);
        void set_clock(unsigned long clock);
        void set_SS_pin(int SS_pin);
        void end();
        void write(byte reg, byte data, uint8_t len = 1);
        void read(byte reg, uint8_t request_bytes = 1, uint8_t first_position = 0);
        byte *_received_data;
    private:
        byte _SS_pin;
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
        unsigned long _clock;
};

#endif