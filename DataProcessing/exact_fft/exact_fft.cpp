#include "exact_fft.h"

void exactFFT_func(int in[], const int N, float *fft_amps) {
  if (N == 1)
    return;
  /* indices */
  uint16_t i,j,k,n_1,array_num_bits;
  uint16_t n_2 = 1;
  /* temporary buffers that should be used right away. */
  float tmp,a,b,c,d,k1,k2,k3;
  /* Will store angles, and recursion values for cosine calculation */
  float cj, sj;

  uint16_t half_N = N >> 1;

  /* How many bits we need to store the positions in half the array.
     This switch could be replaced by a const variable to gain a bit
     of space if we were always computing on the same N of arrays.
  */
 
  switch(N) {
    case 2:
        array_num_bits = 0;
        break;
    case 4:
        array_num_bits = 1;
        break;
    case 8:
        array_num_bits = 2;
        break;
    case 16:
        array_num_bits = 3;
        break;
    case 32:
        array_num_bits = 4;
        break;
    case 64:
        array_num_bits = 5;
        break;
    case 128:
        array_num_bits = 6;
        break;
    case 256:
        array_num_bits = 7;
        break;
    case 512:
        array_num_bits = 8;
        break;
    case 1024:
        array_num_bits = 9;
        break;
    case 2048:
        array_num_bits = 10;
        break;
    case 4096:
        array_num_bits = 11;
        break;
    default:
      array_num_bits = 0;
      break;
  }

  /* Reverse-bit ordering */
  for(i=0; i<half_N; ++i) {
    j = bit_reverse(array_num_bits, i);
    
    if(i<j) {
      /* Swapping real part */
      tmp = in[i<<1];
      in[i<<1] = in[j<<1];
      in[j<<1] = tmp;
      /* Swapping imaginary part */
      tmp = in[(i<<1)+1];
      in[(i<<1)+1] = in[(j<<1)+1];
      in[(j<<1)+1] = tmp;
    }
  }

  /* Actual FFT */
  for(i=0; i<array_num_bits; ++i){
    /* n_1 gives the N of the sub-arrays */
    n_1 = n_2; // n_1 = 2^i
    /* n_2 gives the number of steps required to go from one group of sub-arrays to another */
    n_2 = n_2<<1; // n_2 = 2^(i+1)

    /* j will be the index in Xe and Xo */
    for(j=0;j<n_1;j++) {
      /* We combine the jth elements of each group of sub-arrays */
      cj = cos(-2*PI*j/n_2);
      sj = sin(-2*PI*j/n_2);
      for(k=j; k<half_N; k+=n_2) {
        /* Now we calculate the next step of the fft process, i.e.
           X[j] = Xᵉ[j] + exp(-2im*pi*j/n₂) * Xᵒ[j]
           X[j+n₂/2] = Xᵉ[j] - exp(-2im*pi*j/n₂) * Xᵒ[j]
        */
        a = in[(k+n_1)<<1];
        b = in[((k+n_1)<<1)+1];
        c = in[k<<1];
        d = in[(k<<1)+1];
        k1 = cj * (a+b);
        k2 = a * (sj - cj);
        k3 = b * (cj + sj);
        tmp = k1 - k3;
        in[k<<1]           = c + tmp; 
        in[(k+n_1)<<1]     = c - tmp; 
        tmp = k1 + k2;
        in[(k<<1)+1]       = d + tmp; 
        in[((k+n_1)<<1)+1] = d - tmp; 
      }
    }
  }
  // Serial.println(" ");

  /* Building the final FT from its entangled version */
  /* Special case n=0 */
  in[0] = in[0] + in[1];
  in[1] = 0;
  for(j=1; j<=(half_N>>1); ++j) {
    cj = cos(-PI*j/half_N);
    sj = sin(-PI*j/half_N);
    
    a = in[j<<1] + in[(half_N-j)<<1];
    b = in[(j<<1)+1] - in[((half_N-j)<<1)+1];
    c = -in[(j<<1)+1] - in[((half_N-j)<<1)+1];
    d = in[j<<1] - in[(half_N-j)<<1];
    k1 = cj * (c+d);
    k2 = c*(sj-cj);
    k3 = d*(cj+sj);

    tmp = k1 - k3;
    in[j<<1]                 = ( a - tmp ) * 0.5;
    in[(half_N-j)<<1]     = ( a + tmp) * 0.5; 
    tmp = k1 + k2;
    in[(j<<1)+1]             = ( b - tmp) * 0.5;
    in[((half_N-j)<<1)+1] = (-b - tmp) * 0.5;
  }
  modulus(in, N, fft_amps);
}

/* This is a bit uggly and can be replaced efficiently if we 
   always have the same size of array.
   */
u_int16_t bit_reverse(const uint16_t nbits, uint16_t val) {
    uint16_t i;
    uint16_t res = 0;
    for(i=0; i<nbits; ++i) {
        res <<= 1;
        res |= (val & 1);
        val >>= 1;
    }
    return res;
}

uint16_t bit_reverse_original(const uint16_t nbits, uint16_t val) {
  switch(nbits) {
    case 11:
      val = bit_reverse(5, (val & 0x7C0) >> 5) | (val & 0x20) | (bit_reverse(5, val & 0x1F) << 6);
      break;
    case 10:
      val = bit_reverse(5, (val & 0x3E0) >> 5) | (bit_reverse(5, val & 0x1F) << 5);
      break;
    case 9:
      val = bit_reverse(4, (val & 0x1E0) >> 4) | (val & 0x10) | (bit_reverse(4, val & 0x0F) << 5);
      break;
    case 8:
      val = bit_reverse(4, (val&0xf0)>>4) | (bit_reverse(4, val&0x0f)<<4);
      break;
    case 7:
      val = bit_reverse(3, (val&0x70)>>4) | (val&0x08) | (bit_reverse(3, val&0x07)<<4);
      break;
    case 6:
      val = bit_reverse(3, (val&0x38)>>3) | (bit_reverse(3, val&0x07)<<3);
      break;
    case 5:
      val = bit_reverse(2, (val&0x18)>>3) | (val&0x4) | (bit_reverse(2, val&0x3)<<3);
      break;
    case 4:
      val = bit_reverse(2, (val&0xc)>>2) | (bit_reverse(2, val&0x3)<<2);
      break;
    case 3:
      val = ((val&0x4)>>2) | (val&0x2) | ((val&0x1)<<2);
      break;
    case 2:
      val = ((val&0x2)>>1) | ((val&0x1)<<1);
      break;
    default:
      break;
  }
  return val;
}

/* 
   in is an array of float with in[2i] the real part and in[2i+1] the imaginary part.
   N is the size of the array.

   store the modulus of in in in[0:N/2-1]
 */
void modulus(int in[], const int N, float *fft_amps) {
  uint16_t i;
  for(i=0; i<(N >> 1) ; i++) { 
    int k = 1;
    // to prevent overflow, we compare the value to 2^14 that is the maximum value that can be stored in a 16 bits integer since the last bit is used for the sign.
    if (in[i<<1] > (2^14) || in[(i<<1)+1] > (2^14) || in[i<<1] < -(2^14) || in[(i<<1)+1] < -(2^14)) {
      k = 16;
      in[i<<1] = in[i<<1]/k;
      in[(i<<1)+1] = in[(i<<1)+1]/k;
    }

    fft_amps[i] = k*static_cast<float>(sqrt((in[i<<1]*in[i<<1]) + (in[(i<<1)+1]*in[(i<<1)+1])));
  }
}