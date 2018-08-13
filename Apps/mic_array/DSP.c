/*****************************************************************************
  *    Author: Phan Le Son                                                                                           
  *    Company: Autonomous.ai                                            
  *    email: plson03@gmail.com
  *****************************************************************************/



#include "DSP.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>
//#include <complex.h>
#include "arm_math.h"
#include "W.h"
#include "DelayEstimation.h"

//#include <malloc.h>

//extern int16_t PreCalcBuff[129][256]; /* 1byte have 256 values */


/* Manual calculation */
//#define A0     10000
//#define A1   (-19322)
//#define A2   (9344)

//#define B0   (6)
//#define B1   2*B0
//#define B2   B0

//#define FLOATING_POINT


/* ******************* MATLAB comuptation ***********************
case 1: [b,a] = [b,a] = cheby2(2,60,1/32);
       b	0.0010   -0.0020    0.0010
       a	1.0000   -1.9938    0.9938
case 2: [b,a] = butter(2,1/256); --> cut to 64Khz
       b    3.73251989292944e-05	7.46503978585889e-05	3.73251989292944e-05
       a    1	-1.98264541850412	0.982794719299834

*****************************************************************/

#ifdef FLOATING_POINT

  #define A0   (1)
  #define A1   (-1.9826)
  #define A2   0.9828

  #define B0   (3.7325e-05)                
  #define B1   (7.4650e-05)       
  #define B2   (3.7325e-05)                
#else
  #define A0   (10000)
  #define A1   (-19322)
  #define A2   9322

  #define B0   (6)                
  #define B1   (12)       
  #define B2   (6) 

#endif

Mic_Array_Data_Concate MicData_Concate;
//Mic_Array_Data MicData_Old;

float const (*W_ZP)[PAR_FFT_LEN + 2];
float const (*W_ZP_COM)[PAR_FFT_LEN + 2];



/*
float Coef[129] = { 
-0.02340  ,-0.06595  ,-0.02743  ,0.01425  ,-0.03335  ,-0.08782  ,-0.04168  ,0.01282  ,-0.05293  ,-0.13171 ,
-0.06756  ,0.01115  ,-0.08598  ,-0.20323  ,-0.10855  ,0.00800  ,-0.13553  ,-0.30735  ,-0.16711  ,0.00362 ,
-0.20339  ,-0.44812  ,-0.24434  ,0.00021  ,-0.28985  ,-0.62873  ,-0.33968  ,0.00256  ,-0.39350  ,-0.85181 ,
-0.45087  ,0.01875  ,-0.51123  ,-1.12042  ,-0.57396  ,0.06130  ,-0.63834  ,-1.43986  ,-0.70359  ,0.14924 ,
-0.76886  ,-1.82151  ,-0.83329  ,0.31292  ,-0.89598  ,-2.29113  ,-0.95603  ,0.60590  ,-1.01255  ,-2.90980 ,
-1.06471  ,1.13962  ,-1.11170  ,-3.83936  ,-1.15280  ,2.21212  ,-1.18736  ,-5.63026  ,-1.21482  ,5.05728 ,
-1.23477  ,-11.80705  ,-1.24686  ,30.58676  ,48.78572  ,30.58676  ,-1.24686  ,-11.80705  ,-1.23477  ,5.05728 ,
-1.21482  ,-5.63026  ,-1.18736  ,2.21212  ,-1.15280  ,-3.83936  ,-1.11170  ,1.13962  ,-1.06471  ,-2.90980 ,
-1.01255  ,0.60590  ,-0.95603  ,-2.29113  ,-0.89598  ,0.31292  ,-0.83329  ,-1.82151  ,-0.76886  ,0.14924 ,
-0.70359  ,-1.43986  ,-0.63834  ,0.06130  ,-0.57396  ,-1.12042  ,-0.51123  ,0.01875  ,-0.45087  ,-0.85181 ,
-0.39350  ,0.00256  ,-0.33968  ,-0.62873  ,-0.28985  ,0.00021  ,-0.24434  ,-0.44812  ,-0.20339  ,0.00362 ,
-0.16711  ,-0.30735  ,-0.13553  ,0.00800  ,-0.10855  ,-0.20323  ,-0.08598  ,0.01115  ,-0.06756  ,-0.13171 ,
-0.05293  ,0.01282  ,-0.04168  ,-0.08782  ,-0.03335  ,0.01425  ,-0.02743  ,-0.06595  ,-0.02340 };

*/
/*
int32_t Coef[129] = { 
-234  ,-659  ,-274  ,142  ,-333  ,-878  ,-416  ,128  ,-529  ,-1317 ,
-675  ,111  ,-859  ,-2032  ,-1085  ,80  ,-1355  ,-3073  ,-1671  ,36 ,
-2033  ,-4481  ,-2443  ,2  ,-2898  ,-6287  ,-3396  ,25  ,-3935  ,-8518 ,
-4508  ,187  ,-5112  ,-11204  ,-5739  ,612  ,-6383  ,-14398  ,-7035  ,1492 ,
-7688  ,-18215  ,-8332  ,3129  ,-8959  ,-22911  ,-9560  ,6059  ,-10125  ,-29097 ,
-10647  ,11396  ,-11117  ,-38393  ,-11527  ,22121  ,-11873  ,-56302  ,-12148  ,50572 ,
-12347  ,-118070  ,-12468  ,305867  ,487857  ,305867  ,-12468  ,-118070  ,-12347  ,50572 ,
-12148  ,-56302  ,-11873  ,22121  ,-11527  ,-38393  ,-11117  ,11396  ,-10647  ,-29097 ,
-10125  ,6059  ,-9560  ,-22911  ,-8959  ,3129  ,-8332  ,-18215  ,-7688  ,1492 ,
-7035  ,-14398  ,-6383  ,612  ,-5739  ,-11204  ,-5112  ,187  ,-4508  ,-8518 ,
-3935  ,25  ,-3396  ,-6287  ,-2898  ,2  ,-2443  ,-4481  ,-2033  ,36 ,
-1671  ,-3073  ,-1355  ,80  ,-1085  ,-2032  ,-859  ,111  ,-675  ,-1317 ,
-529  ,128  ,-416  ,-878  ,-333  ,142  ,-274  ,-659  ,-234 };

*/

/* Background noise takes from experiments */
const int16_t NoiseBG[1024] = { 
560  ,1304  ,785  ,1224  ,772  ,1266  ,837  ,1284  ,863  ,1317 ,
937  ,1551  ,996  ,1424  ,1053  ,1502  ,987  ,1489  ,919  ,1319 ,
905  ,1308  ,822  ,1440  ,850  ,1313  ,942  ,1451  ,890  ,1222 ,
953  ,1425  ,801  ,1255  ,903  ,1384  ,985  ,1205  ,865  ,1378 ,
917  ,1095  ,953  ,865  ,828  ,685  ,664  ,590  ,637  ,585 ,
599  ,421  ,527  ,273  ,427  ,449  ,400  ,346  ,566  ,160 ,
217  ,-154  ,170  ,-244  ,164  ,-347  ,-14  ,-263  ,-39  ,-132 ,
-24  ,-114  ,-26  ,22  ,-14  ,-292  ,111  ,-354  ,-41  ,-412 ,
-181  ,-651  ,-280  ,-890  ,-314  ,-1265  ,-510  ,-1464  ,-628  ,-1580 ,
-645  ,-1713  ,-749  ,-1557  ,-879  ,-1354  ,-840  ,-1398  ,-752  ,-1194 ,
-737  ,-1219  ,-701  ,-1448  ,-791  ,-1473  ,-766  ,-1738  ,-904  ,-1471 ,
-957  ,-1355  ,-812  ,-1217  ,-815  ,-1056  ,-733  ,-1097  ,-682  ,-681 ,
-615  ,-625  ,-449  ,-322  ,-375  ,49  ,-276  ,287  ,6  ,221 ,
79  ,140  ,5  ,528  ,43  ,393  ,339  ,-40  ,181  ,106 ,
198  ,18  ,-31  ,92  ,103  ,-24  ,102  ,-78  ,23  ,-331 ,
21  ,-426  ,-17  ,-731  ,-201  ,-723  ,-271  ,-856  ,-301  ,-973 ,
-542  ,-885  ,-434  ,-902  ,-379  ,-878  ,-460  ,-646  ,-412  ,-832 ,
-372  ,-651  ,-373  ,-677  ,-419  ,-508  ,-448  ,-404  ,-335  ,-256 ,
-402  ,-385  ,-450  ,-284  ,-325  ,-372  ,-303  ,-293  ,-394  ,-211 ,
-340  ,162  ,-255  ,158  ,-199  ,-86  ,-296  ,-138  ,-247  ,-183 ,
-327  ,-192  ,-403  ,-190  ,-400  ,-224  ,-296  ,-221  ,-380  ,-239 ,
-332  ,-383  ,-282  ,-457  ,-409  ,-446  ,-478  ,-561  ,-439  ,-520 ,
-406  ,-571  ,-399  ,-389  ,-426  ,-154  ,-271  ,-284  ,-240  ,13 ,
-43  ,-65  ,-51  ,23  ,66  ,-43  ,-19  ,-5  ,-2  ,12 ,
-146  ,178  ,90  ,202  ,65  ,458  ,253  ,503  ,261  ,632 ,
288  ,777  ,398  ,774  ,464  ,721  ,492  ,695  ,506  ,736 ,
501  ,583  ,467  ,700  ,424  ,702  ,545  ,868  ,679  ,914 ,
616  ,951  ,698  ,1046  ,662  ,1053  ,676  ,777  ,590  ,840 ,
560  ,516  ,562  ,556  ,477  ,371  ,447  ,301  ,368  ,210 ,
384  ,-56  ,91  ,8  ,195  ,6  ,146  ,-20  ,220  ,-227 ,
147  ,-382  ,71  ,-458  ,68  ,-663  ,-59  ,-751  ,-280  ,-766 ,
-336  ,-870  ,-364  ,-669  ,-352  ,-449  ,-329  ,-700  ,-179  ,-676 ,
-395  ,-392  ,-183  ,-650  ,-178  ,-699  ,-294  ,-455  ,-267  ,-196 ,
-48  ,260  ,81  ,367  ,234  ,612  ,162  ,672  ,312  ,746 ,
359  ,817  ,517  ,848  ,536  ,860  ,665  ,1023  ,490  ,1011 ,
581  ,1086  ,593  ,1266  ,603  ,1070  ,705  ,1162  ,642  ,1140 ,
518  ,901  ,587  ,893  ,410  ,692  ,513  ,534  ,364  ,589 ,
433  ,869  ,504  ,712  ,558  ,816  ,436  ,730  ,627  ,774 ,
619  ,921  ,541  ,756  ,466  ,981  ,517  ,895  ,499  ,608 ,
378  ,659  ,442  ,619  ,337  ,335  ,345  ,320  ,280  ,534 ,
320  ,597  ,273  ,533  ,370  ,516  ,356  ,437  ,272  ,487 ,
374  ,373  ,404  ,227  ,311  ,295  ,240  ,304  ,324  ,-41 ,
209  ,110  ,102  ,-214  ,38  ,-198  ,-114  ,-142  ,-36  ,-196 ,
-86  ,-151  ,-72  ,-419  ,-47  ,-406  ,-133  ,-437  ,-98  ,-124 ,
-59  ,-212  ,180  ,-208  ,40  ,-559  ,147  ,-805  ,-180  ,-1143 ,
-324  ,-1249  ,-488  ,-1382  ,-701  ,-1078  ,-684  ,-1130  ,-553  ,-910 ,
-404  ,-1308  ,-418  ,-1441  ,-698  ,-1560  ,-690  ,-1719  ,-727  ,-1817 ,
-894  ,-1791  ,-902  ,-1640  ,-931  ,-1292  ,-806  ,-1347  ,-597  ,-1127 ,
-578  ,-899  ,-622  ,-643  ,-402  ,-734  ,-459  ,-427  ,-433  ,-736 ,
-296  ,-527  ,-451  ,-454  ,-345  ,-296  ,-251  ,-269  ,-207  ,25 ,
-157  ,132  ,46  ,214  ,-7  ,452  ,-22  ,235  ,62  ,222 ,
7  ,186  ,-24  ,228  ,-77  ,56  ,-35  ,345  ,6  ,535 ,
51  ,301  ,41  ,229  ,-34  ,281  ,-84  ,193  ,-48  ,-3 ,
-125  ,22  ,-264  ,-25  ,-134  ,-99  ,-100  ,-121  ,-183  ,39 ,
-167  ,-90  ,-176  ,-152  ,-120  ,-222  ,-259  ,-216  ,-208  ,-397 ,
-303  ,-426  ,-279  ,-230  ,-404  ,-253  ,-298  ,-228  ,-379  ,-316 ,
-391  ,-62  ,-397  ,55  ,-190  ,-77  ,-292  ,-4  ,-176  ,-262 ,
-272  ,-256  ,-226  ,-79  ,-218  ,-92  ,-151  ,-16  ,-137  ,42 ,
-199  ,187  ,-148  ,155  ,-24  ,268  ,100  ,255  ,59  ,178 ,
130  ,-26  ,-92  ,-94  ,-213  ,-221  ,-289  ,-206  ,-284  ,-227 ,
-228  ,-251  ,-141  ,-210  ,-300  ,-168  ,-138  ,-194  ,-101  ,-203 ,
-181  ,-496  ,-231  ,-394  ,-134  ,-340  ,-236  ,-551  ,-207  ,-406 ,
-167  ,-771  ,-226  ,-821  ,-365  ,-909  ,-479  ,-971  ,-481  ,-976 ,
-577  ,-965  ,-538  ,-1053  ,-633  ,-1186  ,-609  ,-1184  ,-743  ,-1223 ,
-579  ,-1353  ,-579  ,-1477  ,-818  ,-1281  ,-743  ,-1084  ,-667  ,-1064 ,
-717  ,-790  ,-672  ,-714  ,-557  ,-776  ,-620  ,-531  ,-586  ,-567 ,
-448  ,-311  ,-423  ,-365  ,-442  ,-72  ,-67  ,-388  ,-33  ,-598 ,
-194  ,-560  ,-312  ,-725  ,-358  ,-486  ,-286  ,-449  ,-126  ,-367 ,
-22  ,-287  ,-107  ,-337  ,-44  ,-88  ,63  ,-4  ,13  ,31 ,
110  ,-62  ,-21  ,8  ,35  ,47  ,88  ,172  ,80  ,-10 ,
310  ,-283  ,21  ,-211  ,-49  ,-261  ,36  ,-281  ,0  ,-550 ,
-148  ,-510  ,-244  ,-592  ,-212  ,-691  ,-278  ,-772  ,-321  ,-921 ,
-421  ,-889  ,-464  ,-1194  ,-490  ,-1331  ,-705  ,-1204  ,-663  ,-1398 ,
-669  ,-1385  ,-802  ,-1036  ,-695  ,-1211  ,-684  ,-1469  ,-718  ,-1408 ,
-812  ,-1311  ,-763  ,-1691  ,-834  ,-1582  ,-952  ,-1599  ,-955  ,-1466 ,
-960  ,-1115  ,-958  ,-956  ,-838  ,-783  ,-724  ,-529  ,-591  ,-64 ,
-457  ,-265  ,-199  ,-21  ,-255  ,-153  ,-266  ,-216  ,-314  ,-221 ,
-439  ,-219  ,-383  ,-395  ,-456  ,-240  ,-299  ,-231  ,-260  ,-17 ,
-180  ,18  ,-83  ,154  ,-113  ,125  ,-71  ,384  ,20  ,434 ,
121  ,387  ,132  ,824  ,232  ,942  ,482  ,1417  ,563  ,1134 ,
611  ,1026  ,485  ,802  ,468  ,1099  ,477  ,1286  ,632  ,1532 ,
757  ,1732  ,903  ,1488  ,1027  ,1480  ,1017  ,1495  ,950  ,1454 ,
900  ,1531  ,883  ,1598  ,913  ,1314  ,919  ,1223  ,821  ,983 ,
683  ,846  ,560  ,625  ,437  ,605  ,374  ,325  ,330  ,343 ,
276  ,249  ,265  ,157  ,261  ,73  ,277  ,-17  ,140  ,-75 ,
223  ,46  ,91  ,-115  ,125  ,-413  ,-36  ,-336  ,-81  ,-382 ,
-123  ,-474  ,-112  ,-683  ,-135  ,-639  ,-252  ,-589  ,-262  ,-306 ,
-188  ,227  ,8  ,268  ,255  ,310  ,209  ,473  ,428  ,467 ,
479  ,457  ,341  ,257  ,268  ,152  ,178  ,58  ,76  ,-1 ,
68  ,-203  ,5  ,-311  ,-48  ,-319  ,-117  ,-242  ,-89  ,-232 ,
-89  ,-114  ,-57  ,168  ,25  ,198  ,251  ,367  ,202  ,185 ,
169  ,237  ,84  ,185  ,88  ,575  ,127  ,806  ,270  ,1263 ,
541  ,1506  ,822  ,1625  ,854  ,1489  ,1018  ,1600  ,956  ,1535 ,
1013  ,1583  ,1037  ,1662  ,1072  ,1722  ,1093  ,1685  ,1088  ,1695 ,
1154  ,1826  ,1312  ,2064  ,1329  ,2196  ,1447  ,2348  ,1520  ,2433 ,
1655  ,2410  ,1745  ,2398  ,1625  ,2227  ,1555  ,2052  ,1500  ,2158 ,
1455  ,1907  ,1453  ,1680  ,1420  ,1444  ,1276  ,1204  ,1112  ,870 ,
1003  ,942  ,910  ,579  ,800  ,568  ,602  ,633  ,684  ,163 ,
540  ,80  ,491  ,-41  ,385  ,-41  ,290  ,-331  ,164  ,-440 ,
106  ,-575  ,-94  ,-896  ,-267  ,-788  ,-170  ,-1030  ,-249  ,-933 ,
-287  ,-1127  ,-363  ,-1212  ,-472  ,-1428  ,-474  ,-1460  ,-614  ,-1911 ,
-813  ,-1824  ,-887  ,-1632  ,-872  ,-1409  ,-742  ,-1085  ,-596  ,-999 ,
-450  ,-720  ,-311  ,-732 };



/* 
Library:
https://github.com/piratfm/codec2_m4f/tree/master/lib
*/


/*--------------EXTERN VARIABLES-----------------------------------------------------------------------------*/

#if MAIN_CRSCORR
arm_rfft_instance_q15 RealFFT_Ins, RealIFFT_Ins;
#endif

//extern arm_cfft_radix4_instance_f32 SS,SS1,SS2,SS3,SS4,ISS; 
//extern arm_rfft_instance_f32 S,S1,S2,S3,S4,IS;
extern const float W0[PAR_M][PAR_FFT_LEN + 2];
extern const float W1[PAR_M][PAR_FFT_LEN + 2];
extern const float W2[PAR_M][PAR_FFT_LEN + 2];
extern const float W3[PAR_M][PAR_FFT_LEN + 2];
extern const float W4[PAR_M][PAR_FFT_LEN + 2];
extern const float W5[PAR_M][PAR_FFT_LEN + 2];
extern const float W6[PAR_M][PAR_FFT_LEN + 2];
extern const float W7[PAR_M][PAR_FFT_LEN + 2];


/*------------------------------------------------------------------------------------------------------------*/
/*--------------------- VARIABLES-----------------------------------------------------------------------------*/
arm_rfft_fast_instance_f32 S,S1,S2,S3,S4,S4,S5,S6,S7,S8,IS;

float fir1024Coff[DSP_NUMCOFFHANNIING];
float bufferFFTSum[2*PAR_FFT_LEN];    //storage the SUM in Fourier domain
float bufferFFTSumCom[2*PAR_FFT_LEN];    //storage the SUM in Fourier domain


//float Out_Sum[PAR_FFT_LEN];           //storage the output buffer in float type
float Out_Sum_Pre[PAR_FFT_LEN];       //storage the input buffer in float type
float Out_Sum_PreCom[PAR_FFT_LEN];       //storage the input buffer in float type

float Audio_Sum_Old[PAR_HOP];
float Audio_Sum_OldCom[PAR_HOP];

float fbuffer[PAR_FFT_LEN];           // using in Macro, should not be removed
#if EXT_RAM
#pragma location= (SDRAM_BANK_ADDR+ 3*BUFFER_SIZE_BYTE)
#endif
Mic_Array_Data_f  DataFFT;                  //storage DFT's coefficients for microphones
uint32_t EnergySound,EnergyError;

float Tmp_FFT[2*PAR_FFT_LEN];
float Tmp_FFTCom[2*PAR_FFT_LEN];

//float vDataIn2_FFT[2*AUDIO_OUT_BUFFER_SIZE];
//float vDataIn2_FFT_CJ[2*AUDIO_OUT_BUFFER_SIZE];
//float vDataIn_FFT[2*AUDIO_OUT_BUFFER_SIZE];
//float vDataOut[2*AUDIO_OUT_BUFFER_SIZE];
//float vDataIn[2*AUDIO_OUT_BUFFER_SIZE];
#define vDataIn2_FFT bufferFFTSum
#define vDataIn Tmp_FFT
#define vDataOut Tmp_FFT
#define vDataIn2_FFT_CJ Tmp_FFT 
#define vDataIn_FFT Tmp_FFT



void Window(float *fir64Coff)
{
    for (int i = 0; i < DSP_NUMCOFFHANNIING; i++) //DSP_NUMCOFF
	{
        //fir64Coff[i] = (double_t)((1 << 10)-1);
        fir64Coff[i] = 1.0f; //(float)(DSP_NUMCOFFHANNIING);
		//Hanning Window (less noise than hamming?
        fir64Coff[i] *= 0.5f * (
		                       1.0f - cos((2.0f * PI * i)/ ((float)DSP_NUMCOFFHANNIING - 1.0f))  
		                      );
    }
	 //fir64Coff[DSP_NUMCOFF-1] = 0;
}


void BeamFormingSD_Init(void)
{
		arm_rfft_fast_init_f32(&S1, PAR_FFT_LEN);
		arm_rfft_fast_init_f32(&S2, PAR_FFT_LEN);
		arm_rfft_fast_init_f32(&S3, PAR_FFT_LEN);
		arm_rfft_fast_init_f32(&S4, PAR_FFT_LEN);
		arm_rfft_fast_init_f32(&S5, PAR_FFT_LEN);
		arm_rfft_fast_init_f32(&S6, PAR_FFT_LEN);        
        arm_rfft_fast_init_f32(&IS, PAR_FFT_LEN);
        Window(fir1024Coff);
}


/************************************************************************************************************
-----------------------------Superdirective Beamforming------------------------------------------------------

*************************************************************************************************************/
void BeamFormingSD(const Mic_Array_Data * MicData, uint8_t Dir,int16_t * Audio_Sum)
{

    int32_t _value;  //use in macro, should not be removed
    //W_ZP = W3;
 
	switch (Dir)
	{
		case 0:
			W_ZP = W0;
			break;
		case 1:
			W_ZP = W1;
			break;
		case 2:
			W_ZP = W2;
			break;
		case 3:
			W_ZP = W3;
			break;
		case 4:
			W_ZP = W4;
			break;
		case 5:
			W_ZP = W5;
			break;
		default:
			//W_ZP = W0;
			break;
	}   
 
    /*************************************************************************************************************************/
	/* Concatenate the old frame with current frame */
	for (uint16_t i=0; i< PAR_HOP; i++)
	{
		for(uint16_t j=0; j< PAR_M;j++)
        {      
			*(&MicData_Concate.bufMIC1[i+PAR_HOP]+PAR_FFT_LEN*j) = *(&MicData->bufMIC1[i]+PAR_N*j);
		}
	}

    /**************************************************************************************************************************/   
    RFFT(MicData_Concate.bufMIC1,S1,DataFFT.bufMIC1,fir1024Coff);  
    RFFT(MicData_Concate.bufMIC2,S2,DataFFT.bufMIC2,fir1024Coff);
    RFFT(MicData_Concate.bufMIC3,S3,DataFFT.bufMIC3,fir1024Coff);
    RFFT(MicData_Concate.bufMIC4,S4,DataFFT.bufMIC4,fir1024Coff);
    RFFT(MicData_Concate.bufMIC5,S5,DataFFT.bufMIC5,fir1024Coff);
    RFFT(MicData_Concate.bufMIC6,S6,DataFFT.bufMIC6,fir1024Coff);
    /* Adding in Fourier Domain */			 
    //arm_add_f32((float *)bufferFFT,(float *)bufferFFT_1, (float *)bufferFFTSum,lenFFT*2);

    MUL_C(bufferFFTSum,W_ZP[0],DataFFT.bufMIC1)
    for (uint8_t iMic=1; iMic < PAR_M; iMic++)
    {
        MUL_C(Tmp_FFT,W_ZP[iMic], (DataFFT.bufMIC1+PAR_FFT_LEN*2*iMic))
        SUM_C(bufferFFTSum,Tmp_FFT)  
    }
    /* Revert FFT*/
    arm_rfft_fast_f32(&IS, (float *)bufferFFTSum, (float *)&Out_Sum_Pre[0],1);
    //arm_rfft_fast_f32(&IS, (float *)bufferFFTSum, (float *)&fbufferOut[iFrm*lenFFT], 1);
	

    /* Recontruct the signal and storage hop frame                */
	for (uint16_t i=0;i<PAR_HOP;i++)
	{
		Audio_Sum[i] = (int16_t)(Out_Sum_Pre[i] + Audio_Sum_Old[i]);
		Audio_Sum_Old[i] = (float)Out_Sum_Pre[i+PAR_HOP];
	}
    
	
	/*************************************************************************************************************************/
	/* storage audio data input */
	for (uint16_t i=0; i<PAR_N; i++)
	{
		for (uint16_t j=0; j<PAR_M;j++)
		{
             *(&MicData_Concate.bufMIC1[i  ]+PAR_FFT_LEN*j) = *(&MicData->bufMIC1[i]+PAR_N*j);
		}
	}
	//arm_float_to_q15((float32_t *)fbufferOut,(q15_t *)stBufOut,AUDIO_OUT_BUFFER_SIZE); 
}

/************************************************************************************************************
-----------------------------Super directive Beam forming: 2 beams-------------------------------------------

*************************************************************************************************************/
void BeamFormingSDCom(const Mic_Array_Data * MicData, uint8_t Dir, Audio_Out  Audio_Sum)
{

	int16_t _value;  //use in macro, should not be removed
	switch (Dir)
	{
		case 0:
			W_ZP = W0;
            W_ZP_COM = W3;
			break;
		case 1:
			W_ZP = W1;
            W_ZP_COM = W4;
			break;
		case 2:
			W_ZP = W2;
            W_ZP_COM = W5;
			break;
		case 3:
			W_ZP = W3;
            W_ZP_COM = W0;
			break;
		case 4:
			W_ZP = W4;
            W_ZP_COM = W1;
			break;
		case 5:
			W_ZP = W5;
            W_ZP_COM = W2;
			break;
		default:
			break;
	}

	
	/*************************************************************************************************************************/
	/* Concatenate the old frame with current frame */
	for (uint16_t i=0; i< PAR_HOP; i++)
	{
		for(uint16_t j=0; j< PAR_M;j++)
		{
			*(&MicData_Concate.bufMIC1[i+PAR_HOP]+PAR_FFT_LEN*j) = *(&MicData->bufMIC1[i]+PAR_N*j);
		}
	}

	/**************************************************************************************************************************/
	RFFT(MicData_Concate.bufMIC1,S1,DataFFT.bufMIC1,fir1024Coff);
	RFFT(MicData_Concate.bufMIC2,S2,DataFFT.bufMIC2,fir1024Coff);
	RFFT(MicData_Concate.bufMIC3,S3,DataFFT.bufMIC3,fir1024Coff);
	RFFT(MicData_Concate.bufMIC4,S4,DataFFT.bufMIC4,fir1024Coff);
	RFFT(MicData_Concate.bufMIC5,S5,DataFFT.bufMIC5,fir1024Coff);
	RFFT(MicData_Concate.bufMIC6,S6,DataFFT.bufMIC6,fir1024Coff);

    
	/* Adding in Fourier Domain */
	//arm_add_f32((float *)bufferFFT,(float *)bufferFFT_1, (float *)bufferFFTSum,lenFFT*2);

	MUL_C(bufferFFTSum,W_ZP[0],DataFFT.bufMIC1)
	MUL_C(bufferFFTSumCom,W_ZP_COM[0],DataFFT.bufMIC1)
	for (uint8_t iMic=1; iMic < PAR_M; iMic++)
	{
		MUL_C(Tmp_FFT,W_ZP[iMic], (DataFFT.bufMIC1+PAR_FFT_LEN*2*iMic))
		SUM_C(bufferFFTSum,Tmp_FFT)
		
		MUL_C(Tmp_FFTCom,W_ZP_COM[iMic], (DataFFT.bufMIC1+PAR_FFT_LEN*2*iMic))
		SUM_C(bufferFFTSumCom,Tmp_FFTCom)
	}
#if 1	
    /* Binary mask for 2 direction sources */
	
	/* magnitude of source 1*/
	arm_cmplx_mag_f32(bufferFFTSum,Tmp_FFT,PAR_FFT_LEN);
	/* magnitude of source 2*/
	arm_cmplx_mag_f32(bufferFFTSumCom,Tmp_FFTCom,PAR_FFT_LEN);
	/* Compare and do the binary mask */	
	for (uint16_t i=0; i < PAR_N; i++)
	{
		if (Tmp_FFT[i] < 1.0*Tmp_FFTCom[i])
		{
			bufferFFTSum[2*i] = 0.0*bufferFFTSum[2*i];
			bufferFFTSum[2*i+1] = 0.0*bufferFFTSum[2*i+1];
		}
		if (Tmp_FFTCom[i] < 1.1*Tmp_FFT[i] )
		{
		    bufferFFTSumCom[2*i] = 0.2*bufferFFTSumCom[2*i];
			bufferFFTSumCom[2*i+1] = 0.2*bufferFFTSumCom[2*i+1];
		}
		else
		{
			;
		}
	}
#endif

	/* Revert FFT*/
	arm_rfft_fast_f32(&IS, (float *)bufferFFTSum, (float *)&Out_Sum_Pre[0],1);
    arm_rfft_fast_f32(&IS, (float *)bufferFFTSumCom, (float *)&Out_Sum_PreCom[0],1);
	

	/* Reconstruct the signal and storage hop frame                */
	for (uint16_t i=0;i<PAR_HOP;i++)
	{
		Audio_Sum.bufMIC1[i] = (int16_t)((Out_Sum_Pre[i] + Audio_Sum_Old[i]));
		Audio_Sum_Old[i] = (float)Out_Sum_Pre[i+PAR_HOP];
		
		Audio_Sum.bufMIC2[i] = (int16_t)((Out_Sum_PreCom[i] + Audio_Sum_OldCom[i]));
		Audio_Sum_OldCom[i] = (float)Out_Sum_PreCom[i+PAR_HOP];
	}
	
	
	/*************************************************************************************************************************/
	/* storage audio data input */
	for (uint16_t i=0; i<PAR_N; i++)
	{
		for(uint16_t j=0; j<PAR_M;j++)
		{
			*(&MicData_Concate.bufMIC1[i  ]+PAR_FFT_LEN*j) = *(&MicData->bufMIC1[i]+PAR_N*j);
		}
	}

}


