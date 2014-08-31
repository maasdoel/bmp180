#define sleepms(ms)  usleep((ms)*1000)

// BMP085 & BMP180 Specific code
#define BMPx8x_I2CADDR           0x77
#define BMPx8x_CtrlMeas          0xF4
#define BMPx8x_TempConversion    0x2E
#define BMPx8x_PresConversion0   0x34
#define BMPx8x_Results           0xF6
#define BMPx8x_minDelay          4     //require 4.5ms *1000/700 'turbo mode fix'= 6.4-Retry =4.4
#define BMPx8x_RetryDelay        2     //min delay for temp 4+2=6ms, max 4+2*20=44ms for pressure
                                       //Will stop waiting if conversion is complete

const unsigned char BMPx8x_OverSampling = 3;

// Calibration values - These are stored in the BMP085/180
short int            bmp_ac1;
short int            bmp_ac2; 
short int            bmp_ac3; 
unsigned short int   bmp_ac4;
unsigned short int   bmp_ac5;
unsigned short int   bmp_ac6;
short int            bmp_b1; 
short int            bmp_b2;
int                  bmp_b5; 
short int            bmp_mb;
short int            bmp_mc;
short int            bmp_md;

int      bmp_ReadInt          (int fd, uint8_t *devValues,uint8_t startReg,uint8_t bytesToRead);
int      bmp_WriteCmd         (int fd, uint8_t devAction);
int      bmp_Calibration      (int fd);
int      WaitForConversion    (int fd);
int      bmp_GetPressure      (int fd, double *Pres);
int      bmp_GetTemperature   (int fd, double *Temp);

double   bmp_altitude         (double p);
double   bmp_qnh              (double p,double StationAlt);
double   ppl_DensityAlt       (double PAlt,double Temp);
