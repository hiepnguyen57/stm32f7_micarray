
// Sound Speed  c=343 m/s
#define  PAR_C 343
// epsilon
#define PAR_EPS  1e-6

//Number of microphone
#define PAR_M  6

//Number of OUT channel
#define PAR_CHNNL  2


// Diameter of circle D=0.052 m
#define PAR_D   0.074

//Sampling frequency
#define PAR_FS  16000

// Length of frame
#define PAR_N  512

//Zero padding front
#define PAR_ZPF  0

//Zero padding bacl
#define PAR_ZPB  0

//Lenght with zero-padding
#define PAR_L  (PAR_N + PAR_ZPF + PAR_ZPB)

//Haft-lengnth
#define PAR_L_H   (int)(PAR_L/2)

// Frequency slot with Zerro Padding
//FS = np.linspace(0,fs, (N+ zpb  + zpf))

// Frequency slot without Zerro Padding
//FS_RAW = np.linspace(0,fs,N)

// Haft frequency slot
// FS_H = np.linspace(fs/N,fs/2 + fs/N,N/2)  # e.g: N=1024 -> 512 slot
// FS_H = np.linspace(0, fs/2 + fs/N,(N+ zpb  + zpf)/2+1)
// FS_T = np.linspace(0, fs ,N)
// FS_H = FS[:math.floor((N+zpf+zpb)/2)+1]

//offset
#define PAR_OFFSET  6 

//Radius of circle
#define PAR_R   0.037

//Resolution of fraction delay: RES = 2.5/T : ex:t = 1/4 Sample -> RES = 10
#define PAR_RES  4

//Frequency after fraction delay
#define PAR_FS_F  (int)(PAR_FS*PAR_RES*2/5)

//Hope
#define PAR_HOP PAR_N

//fft len
#define PAR_FFT_LEN 2*PAR_N

#define PAR_GAMMA  0.0

//number of raw audio buffer frame
#define CNTBUF  6

// Number of frame to get environment noise
#define COHE_FRM  160

//Number of direction
#define PAR_NUMDIR  6


