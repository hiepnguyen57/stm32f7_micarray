#ifndef __DSP_H
#define __DSP_H

#include "main.h"


#define PI 3.1415927

/* ***** Decimation FIR Filter ***************************************************************************************

y[n] = sum_{k=0}^{K-1}     x[nM-k].h[k],

M =64 : this is derived from the characteristic of MEMS microphone 
n: nth of sampke output_iterator
K (captial): number of cofficience of FIR filter: 256 , this is as much as possible depend of the resource of MCU

************************************************************************************************************************/
#define DSP_NUMCOFF         64
#define DSP_NUMCOFFHANNIING  PAR_FFT_LEN
#define DSP_NUMBYTECONV (DSP_NUMCOFF>>3)  /* Number of input sample bytes uses for 1 convolution */

void Window(float *fir64Coff);
void BeamFormingSD(const Mic_Array_Data * MicData, uint8_t Dir,int16_t * Audio_Sum);
void BeamFormingSDCom(const Mic_Array_Data * MicData, uint8_t Dir, Audio_Out  Audio_Sum);
void BeamFormingSD_Init(void);


//arm_q15_to_float((int16_t *)stBuf,fbuffer,PAR_N);                                     \
//arm_mult_f32(win,fbuffer,fbuffer,PAR_N);                                \

// Multify with Window function and FFT transform
#define RFFT(stBuf,S,bufferFFT,win)                                            \
       {                                                                       \
       for(uint16_t j=0;j<PAR_FFT_LEN;j++)                                     \
       {                                                                       \
         _value = (int32_t)stBuf[j];                                           \
          fbuffer[j]=(float)(_value*win[j]);                                   \
       }                                                                       \
         arm_rfft_fast_f32(&(S), (float *)fbuffer, (float *)(bufferFFT),0);    \
       }

/* FFT transform */
#define RFFT_GCC(stBuf,S,bufferFFT,Len)                                        \
       {                                                                       \
	   for(uint16_t j=0;j<Len;j++)                                             \
	   {                                                                       \
	       fbuffer[j]=(float)(stBuf[j]*1.0f);                                  \
	   }                                                                       \
       arm_rfft_fast_f32(&(S), (float *)fbuffer, (float *)(bufferFFT),0);      \
       }


#define MUL_C(o,w,s)            /* MULtiply complex vector (w is conjunction and only have half of bin) */  \
{                                                                                                           \
    int _i;                                                                                                 \
    for (_i = 0; _i < PAR_FFT_LEN+2; _i=_i+2)                                                               \
	{                                                                                                       \
        o[_i] = w[_i] * s[_i] + w[_i+1] * s[_i+1];                                                          \
		o[_i+1] = w[_i] * s[_i+1] - w[_i+1] * s[_i];                                                        \
		if ((_i!=0)&&(_i!=PAR_FFT_LEN+1))                                                                   \
        {                                                                                                   \
            o[2*PAR_FFT_LEN -_i] = -o[_i+1];                                                                \
            o[2*PAR_FFT_LEN -_i-1] =  o[_i];                                                                \
        }                                                                                                   \
	}                                                                                                       \
}



#define SUM_C(d,s)  /* covert from float to complex */                  \
{                                                                       \
    int _i;                                                             \
    for (_i = 0; _i < 2*PAR_FFT_LEN; _i=_i+2)                           \
	{                                                                   \
	    d[_i] += s[_i];                                                 \
        d[_i+1] += s[_i+1];                                             \
	}                                                                   \
}

	   
#endif /* __DSP_H */


       

	



//           arm_rfft_fast_f32(&(S), (float *)fbuffer, (float *)(bufferFFT), 0); \



