#include "spi_handler.h"

SPI_Handler::SPI_Handler()
{
    this->_received_data = NULL;
    this->_device_id = 0;
}

SPI_Handler::SPI_Handler(byte *received_data, byte DEVICE_ID)
{
    this->_received_data = received_data;
    this->_device_id = DEVICE_ID;
    this->_who_am_i_val = DEVICE_ID;
}

SPI_Handler::SPI_Handler(byte *received_data, byte DEVICE_ID, byte WHO_AM_I_VAL)
{
    this->_received_data = received_data;
    this->_device_id = DEVICE_ID;
    this->_who_am_i_val = WHO_AM_I_VAL;
}

void SPI_Handler::set_SS_pin(int SS_pin)
{
    this->_SS_pin = SS_pin;
    pinMode(SS_pin, OUTPUT);
    digitalWrite(SS_pin, HIGH);
}

void SPI_Handler::write(byte reg, byte data, uint8_t len)
{
    this->_reg = reg;
    this->_data = data;
    this->_len = len;

    #ifdef DEBUG_SPI
        Serial.print("[DEBUG][SPI] SPI_Handler::write() - reg: "); Serial.print(this->_reg, HEX);
        Serial.print(" - data: "); Serial.print(this->_data, HEX);
        Serial.print(" - len: "); Serial.println(this->_len, HEX);
    #endif

    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
    digitalWrite(this->_SS_pin, LOW);
    SPI.transfer(this->_reg);
    SPI.transfer(this->_data);
    digitalWrite(this->_SS_pin, HIGH);
    SPI.endTransaction();
}

void SPI_Handler::read(byte reg, uint8_t request_bytes, uint8_t first_position)
{
    this->_reg = reg;
    this->_len = request_bytes;
    this->_count = first_position;

    #ifdef DEBUG_SPI
        Serial.print("[DEBUG][SPI] SPI_Handler::read() - reg: "); Serial.print(this->_reg, HEX);
        Serial.print(" - len: "); Serial.print(this->_len, HEX);
        Serial.print(" - count: "); Serial.println(this->_count, HEX);
    #endif

    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
    digitalWrite(this->_SS_pin, LOW);
    SPI.transfer(this->_reg);
    for (uint8_t i = 0; i < (this->_len); i++)
    {
        this->_received_data[i+first_position] = SPI.transfer(0x00);
        #ifdef DEBUG_SPI
            Serial.print("[DEBUG][SPI] SPI_Handler::read() - received byte: ");
            Serial.println(this->_received_data[i+first_position], HEX);
        #endif
    }
    digitalWrite(this->_SS_pin, HIGH);
    SPI.endTransaction();
}