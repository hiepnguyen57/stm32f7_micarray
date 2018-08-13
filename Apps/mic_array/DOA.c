/*****************************************************************************
  *    Author: Phan Le Son                                                                                           
  *                                             
  *    email: plson03@gmail.com
  *****************************************************************************/

#include "DOA.h"

const float DOA[PAR_NUMDIR][PAR_M/2] = 
{
#if DOA_RESAMPLE

#else
    {-4.0583,   -2.0292,    2.0292},    //0
    {-2.0292,   -4.0583,  -2.0292},    //60
    {2.0292,   -2.0292,   -4.0583},    //120
    {4.0583 ,   2.0292,   -2.0292},    //180
    { 2.0292 ,   4.0583,    2.0292},    //240
    {-2.0292  ,  2.0292,    4.0583}     //300

#endif
};

const float Angle[PAR_NUMDIR] = {0 , 60, 120, 180, 240, 300};

//static float PwrChnnl1Old;
float Power = 20000;
float dynamic_power_adj_damping = 0.0225;
float damping=0.6223; // dynamic_power_adj_damping ** (1.0/8.0)
float dynamic_power_ratio = 1.5;
float offset = 0;
uint8_t Dir = 0;
uint8_t DirOld = 0;
uint8_t flgContinue;
uint8_t cntDeb;
int8_t Dly_Sample[PAR_M/2]; /* PAR_M/2 number of couple*/

uint8_t DOACalc(const Mic_Array_Data * Audio_Data)
{
	
	float PwrChnnl1 = 0.0f;
	//float Target_Power;
	uint16_t i,j;
	int16_t Likely;
	uint16_t LikelyMin=0xFFFF;
	uint8_t idxDir=0;
    uint8_t DirCurr = 0;
	/* Computate the power density of channel 1*/
	for (i=0; i< PAR_N; i++)
	{
		PwrChnnl1 += (float)(Audio_Data->bufMIC1[i]*Audio_Data->bufMIC1[i]);
	}
	PwrChnnl1 /= PAR_N;


	//TODO: to increase the accuracy and the computation cost of DOA, Set-On detect could be considered

	if (PwrChnnl1 > Power + offset)
	{
		ComputeDelay_Couple(Audio_Data,Dly_Sample);

		for (i=0; i < PAR_NUMDIR;i++)
		{
			Likely = 0;
			for (j=0; j < PAR_M/2; j++)
			{
			    Likely+=(Dly_Sample[j] - DOA[i][j])*(Dly_Sample[j] - DOA[i][j]);	
			}

			if (Likely <= LikelyMin)
			{
				LikelyMin = Likely;
				idxDir = i;
				DirCurr = idxDir;
			}
		}

		/* Deboucing direction of sound */
		if (flgContinue)
		{
			if (DirCurr == DirOld)
			{
			    cntDeb += 1;
			}
			else
			{
				cntDeb = 0;
			}
		}

		if ((cntDeb>0)&&(LikelyMin < 10))
		{
			Dir = DirCurr;
            ////printf("Likely: %f --", Likely);
			////printf("Angle: %f :[%2d %2d %2d  %2d] \n\r", Angle[idxDir], Dly_Sample[0], Dly_Sample[1], Dly_Sample[2], Dly_Sample[3]);
		}

		flgContinue = 1;
		DirOld = DirCurr;
	}
	else
	{
		/* Update Power of background noise */
		//Target_Power = PwrChnnl1*dynamic_power_ratio;
		//Power = Power*damping + Target_Power*(1-damping);
		flgContinue = 0;
        cntDeb = 0;
	}
    
    //PwrChnnl1Old = PwrChnnl1;

	return Dir;
}
