#include <Arduino.h>
#include "aprox_fft.h"

#ifndef fft_handler_h
#define fft_handler_h

/* Tomvava*/

/* Windowing type */
#define FFT_WIN_TYPE_RECTANGLE 0x00 /* rectangle (Box car) */
#define FFT_WIN_TYPE_HAMMING 0x01 /* hamming */
#define FFT_WIN_TYPE_HANN 0x02 /* hann */
#define FFT_WIN_TYPE_TRIANGLE 0x03 /* triangle (Bartlett) */
#define FFT_WIN_TYPE_NUTTALL 0x04 /* nuttall */
#define FFT_WIN_TYPE_BLACKMAN 0x05 /* blackman */
#define FFT_WIN_TYPE_BLACKMAN_NUTTALL 0x06 /* blackman nuttall */
#define FFT_WIN_TYPE_BLACKMAN_HARRIS 0x07 /* blackman harris*/
#define FFT_WIN_TYPE_FLT_TOP 0x08 /* flat top */
#define FFT_WIN_TYPE_WELCH 0x09 /* welch */

/*Mathematial constants*/
#define twoPi 6.28318531 /*2π*/
#define fourPi 12.56637061 /*4π*/
#define sixPi 18.84955593 /*6π*/

/*FFT input values Units*/
#define FFT_INPUT_NONE 0x00 /*No prefix of international units system*/
#define FFT_INPUT_MILI 0x01 /*Milli (10^-3) prefix of international units system*/
#define FFT_INPUT_MICRO 0x02

/*FFT output values Units*/
#define FFT_OUTPUT_NONE 0x00
#define FFT_OUTPUT_DB 0x01 /*Decibel (dB) */

/* Methods for FFT */
#define ALGORITHM_exactFFT 0x00 /*exactFFT algorithm for FFT determination*/
#define ALGORITHM_aproxFFT 0x01 /*aaproxFFT algorithm for FFT determination*/
#define ALGORITHM_ARDUINOFFT 0x02 /*arduinoFFT algorithm for FFT determination*/

/* Objective of FFT*/
#define OBJECTIVE_PRINT_FFT_FULL 0x00
#define OBJECTIVE_DETERMINE_FFT 0x01

/* Methods for Peak determinatio */
#define zscore 0x00 /*zscore algorithm for peak determination*/
#define madealgorithm_one 0x01 /*madealgorithm1 algorithm for peak determination*/

class FFT_handler {

    public:
        FFT_handler(int raw_data_sample_nr, int windows_nr, int *raw_data,float *fft_amps);
        void Windowing(uint8_t window);
        void FFT(byte convert_units = true ,byte normalise_with_window_sample_nr = true, byte normalise_for_half_window = true );
        void set_windows_nr(int value);
        void set_fft_output_units(u_int8_t value);
        void set_fft_input_units(uint8_t units);

    private:
        float *_fft_amps;
        int _windows_nr;
        int _raw_data_sample_nr;
        int _window_sample_nr;
        int *_raw_data;
        uint8_t _fft_output_units;
        uint8_t _fft_input_units;
        float *_freq;
};
#endif