#include <Arduino.h>

void exactFFT_func(int in[],int N,float *fft_amps);
uint16_t bit_reverse(const uint16_t nbits, uint16_t val);
u_int16_t bit_reverse_original(const uint16_t nbits, uint16_t val);
void modulus(int in[], const int size, float *fft_amps);