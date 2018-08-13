/*****************************************************************************
  *    Author: Phan Le Son                                                                                           
  *                                             
  *    email: plson03@gmail.com
  *****************************************************************************/

#include "DelayEstimation.h"
#include <stdio.h>



/* Cross-correlation */
int8_t CrssCor(const int16_t * vDataIn1, const int16_t * vDataIn2, uint16_t numLen, uint32_t * CrssCorVal )
{
    int64_t Sum, SumMax;
	uint16_t j;
	int8_t idxPos=0;
	int8_t i;
#if _MALLOC
	int16_t *vDataIn1Out = (int16_t *)malloc(sizeof(int16_t)*numLen);
	int16_t *vDataIn2Out = (int16_t *)malloc(sizeof(int16_t)*numLen);
	for (uint16_t i=0;i<numLen;i++)
    {
        vDataIn1Out[i]= (int16_t)(vDataIn1[i]);//fir256Coff[i]
        vDataIn2Out[i]= (int16_t)(vDataIn2[i]);//fir256Coff[i]
    }
#endif

    SumMax=0;
#if 0	
	static int16_t vDataIn1Old, vDataIn2Old;
	LowPassIIR(vDataIn1,vDataIn1Out ,&vDataIn1Old, numLen,8);
	LowPassIIR(vDataIn2,vDataIn2Out ,&vDataIn2Old, numLen,8);
#endif
	
    for ( i=-PAR_RES;i<=PAR_RES;i++)  /* physical limit */
    {
       Sum = 0;
	   if (i>=0)
	   {
	       for( j=0;j<numLen-i;j++)
	       {
#if _MALLOC
	           Sum += vDataIn1Out[j+i]*vDataIn2Out[j];
#else
			   Sum += vDataIn1[j+i]*vDataIn2[j];
#endif 
	       }                  
	   }
	   else
	   {
           for(j=0;j<numLen + i;j++)
	       {
#if _MALLOC
	           Sum += vDataIn1Out[j]*vDataIn2Out[j-i];
#else
			   Sum += vDataIn1[j]*vDataIn2[j-i];
#endif
	       }
	   }

	   Sum /= (numLen-i); 

	   if (Sum > SumMax) 
	   {
	       SumMax = Sum;  	
		   
	       idxPos = i;
	       *CrssCorVal = SumMax;//(uint32_t)((SumMax>>15));
	   }
	         
    }
#if _MALLOC
    free(vDataIn1Out);
	free(vDataIn2Out);
#endif

    return idxPos;
}



/* Cross-Correlation the signal after resampling     */
/* vDataIn1 is resampled                             */
/* vDataIn2 is not resampled                         */
/* numLen is the length of block without resample    */
/* Coef = NumberOfBin(Resample/Orginal)              */
/* CrssCorVal return value of max energy             */
int8_t CrssCorResample(const int16_t * vDataIn1, const int16_t * vDataIn2, uint16_t numLen,uint8_t Coef, uint32_t * CrssCorVal )
{
    int64_t Sum, SumMax;
    uint16_t j;
    int8_t idxPos=0;
    int8_t i;


    SumMax=-2^31;
    //printf("test");

    for ( i=-PAR_RES;i<=PAR_RES;i++)  /* physical limit */
    {
        Sum = 0;
        //printf("i: %d --", i);


	/* Do the cross correlation for the sub-array after remove first and last 128 samples*/
	for( j=128;j<numLen-128;j++)
	{
	    Sum += (vDataIn1[j*Coef+i])*(vDataIn2[j]);
	    //printf("%d - %d ",vDataIn1[j*Coef+i],vDataIn2[j]);
	}

	Sum /= (numLen-256);

	    
	   if (Sum > SumMax) 
	   {
	       SumMax = Sum;  	
		   
	       idxPos = i;
	       *CrssCorVal = SumMax;//(uint32_t)((SumMax>>15));
	   }
	         
    }

    return idxPos;
}


void Resampling(const int16_t * vDataIn, int16_t * vDataOut, uint16_t numLen)
{
	Complex *FFT_In;
	Complex *FFT_Out;
	uint16_t i = 0;
	uint8_t Coef;

	Coef = (uint8_t)(numLen/PAR_N);

	/* ask for memory */
	FFT_In = (Complex *)malloc(sizeof(Complex)*PAR_N);
    if (FFT_In == NULL)
    {
       //printf(" Out of memory!\n");
       exit(1);
    }
	FFT_Out = (Complex *)malloc(numLen*sizeof(Complex));
    if (FFT_Out == NULL)
    {
       //printf(" Out of memory!\n");
       exit(1);
    }
	/* Complex buffer storage*/
	for (i=0; i < PAR_N; i++)
	{
		FFT_In[i].real = (float)(vDataIn[i]);
		FFT_In[i].imag = 0.0f;
		//printf ("%d - %6f|", i,FFT_In[i].real );
	}
	
	/* FFT transform the input */
	//TODO: change to CMSIS lib fft(FFT_In, PAR_N);

	/* Adding zero to FFT bin */
	for (i=0; i < PAR_N/2+1;i++)  // +1: only handle for the case PAR_N is even
	{
		FFT_Out[i].real = 4*FFT_In[i].real;
		FFT_Out[i].imag = 4*FFT_In[i].imag;
		if (i < PAR_N/2)
		{
		    FFT_Out[i+3*PAR_N + PAR_N/2 ].real = 4*FFT_In[i+ PAR_N/2 ].real;
		    FFT_Out[i+3*PAR_N + PAR_N/2 ].imag = 4*FFT_In[i+ PAR_N/2 ].imag;
		}
	}

	/* TODO: file the library to set to zero of array */
	for (i=PAR_N/2+1; i < 4*PAR_N-PAR_N/2;i++)
	{
		FFT_Out[i].real = 0.0f;
		FFT_Out[i].imag = 0.0f;
	}

	/* Invert FFT transform */
	//TODO: change to CMSIS ifft(FFT_Out,Coef*PAR_N);

	/* Update output buffer*/
	for (i=0; i < Coef*PAR_N;i++)
	{
		vDataOut[i] = (int16_t)FFT_Out[i].real;
                //printf("%d ", vDataOut[i]);
	}

	free(FFT_In);
	free(FFT_Out);
}

void ComputeDelay_Couple(const Mic_Array_Data * Audio_Data, int8_t Delay_In_Sample[])
{
#if DOA_RESAMPLE
	int16_t * In1;
	uint32_t Val;
	In1 = (int16_t *)malloc(4*PAR_N*sizeof(int16_t));// 4 is factor of resampling
	if (In1 == NULL)
    {
       //printf(" Out of memory!\n");
       exit(1);
    }
	/* Delay in sample of couple 1 */
	Resampling(Audio_Data->bufMIC1, In1, 4*PAR_N);
	Delay_In_Sample[0] = CrssCorResample(In1,&Audio_Data->bufMIC4[0],PAR_N,4,&Val);
    //printf("D: %d \n",Delay_In_Sample[0]);
    
	/* Delay in sample of couple 2 */
	Resampling(Audio_Data->bufMIC2, In1, 4*PAR_N);
	Delay_In_Sample[1] = CrssCorResample(In1,&Audio_Data->bufMIC5[0],PAR_N,4,&Val);
    //printf("D: %d \n",Delay_In_Sample[1]);
    
	/* Delay in sample of couple 3 */
    Resampling(Audio_Data->bufMIC3, In1, 4*PAR_N);
	Delay_In_Sample[2] = CrssCorResample(In1,&Audio_Data->bufMIC6[0],PAR_N,4,&Val);



	free(In1);
#else
    uint32_t Val;
    Delay_In_Sample[0] = CrssCor(&Audio_Data->bufMIC1[0],&Audio_Data->bufMIC4[0],PAR_N,&Val);
    Delay_In_Sample[1] = CrssCor(&Audio_Data->bufMIC2[0],&Audio_Data->bufMIC5[0],PAR_N,&Val);
    Delay_In_Sample[2] = CrssCor(&Audio_Data->bufMIC3[0],&Audio_Data->bufMIC6[0],PAR_N,&Val);
#endif
}

