#include <Arduino.h>

void aproxFFT_func(int in[],int N,float *fft_amps);
int fast_sine(int Amp,   int th);
int fast_cosine(int Amp, int th);
int fastRSS(int a, int b);