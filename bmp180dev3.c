/*  (Dev)   -  (Pi)
    SDA     -  SDA
    SCL     -  SCL
    GND     -  GND
    VCC     -  3.3V

   This code is compatible with bmp085 & bmp180

   Note: Check your pin-out
   Note: Make sure you connect the PI's 3.3 V line to the BMP085/180 boards Vcc 'Vin' line not the 3.3v 'OUT'

   How to compile, @ command line type

        gcc -Wall -o bmp180dev3 ./bmp180dev3.c -lm

   the '-lm' is required for 'math.h'

   for constants such as O_RWRD or I2C_M_RD checkout i2c.h & i2c-dev.h
   this also contains the definition of 'struct i2c_msg' so if you want to see what is
   possible check it out.
   also have a look at

>>>>>  https://www.kernel.org/doc/Documentation/i2c/i2c-protocol <<<<<<<<<< NB! read it


   In general communication functions return an integer < 0 on failure 0 for success and > 0 if a 'handle'
   is being returned.

   Conversion functions return a double

   PS there are better density and QNH formulae.

   Use as you see fit.

   Eric Maasdorp 2014-08-30

   PS !!!!!!
   I have found quite a few examples of similar code but all had a problem with
   overclocked pi's resulting in corrupted results typically 5-15 lost samples per 6 hours
   based on a 5 minute sample interval.  My method poles for conversion completion.

   and does not require smbus or wire libs

   Remember to set to correct I2CBus

*/

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <math.h>
#include "bmp180dev3.h"

#define sleepms(ms)  usleep((ms)*1000)

#define I2CBus             "/dev/i2c-1"      //New Pi's
//#define I2CBus             "/dev/i2c-0"    //Old, but not stale Pi's

// Returns a file id for the port/bus
int i2c_Open(char *I2CBusName){
  int fd;
  //Open port for reading and writing
  if ((fd = open(I2CBusName, O_RDWR)) < 0){
    printf ("\n");
    printf ("%s : Failed to open the i2c bus, error : %d\n",__func__,errno);
    printf ("Check to see if you have a bus: %s\n",I2CBusName);
    printf ("This is not a slave device problem, I can not find the bus/port with which to talk to the device\n");
    printf ("\n");
    // Only one of the following lines should be used
    // the second line allows you to retry another bus, PS disable all the printf 's
    exit(1);      //Use this line if the function must terminate on failure
    //return fd;  //Use this line if it must return to the caller for processing
    }
    else{
     return fd;
    }
}

// BMP085 & BMP180 Specific code

int bmp_ReadInt(int fd, uint8_t *devValues,uint8_t startReg,uint8_t bytesToRead){
  int rc;
  struct i2c_rdwr_ioctl_data messagebuffer;

  //Build a register read command
  //Requires a one complete message containing a command
  //and anaother complete message for the reply
  struct i2c_msg read_reg[2]={
    {BMPx8x_I2CADDR,0,1,&startReg},
    {BMPx8x_I2CADDR,I2C_M_RD,bytesToRead,devValues}
  };

  messagebuffer.nmsgs = 2;                  //Two message/action
  messagebuffer.msgs = read_reg;            //load the 'read__reg' message into the buffer
  rc = ioctl(fd, I2C_RDWR, &messagebuffer); //Send the buffer to the bus and returns a send status
  if (rc < 0 ){
    printf("\n");
    printf("%s :Reg Read command failed with error :%d\n",__func__,errno);
    printf("This means that device with address :0x%0x failed to receive this command\n",BMPx8x_I2CADDR);
    printf("This command was preceded by a reset if that worked\n");
    printf("and this failed, then possible causes are Delay timing to short (overclock stuffing timing up)\n");
    printf("or bus unstable ,wire length,power supply unstable, terminating resistors.\n");
    printf("\n");
    // Only one of the following lines should be used
    //exit(1);       //Use this line if the function must terminate on failure
    return rc;       //Use this line if it must return to the caller for processing
  }
  //note that the return data is contained in the array pointed to by devValues (passed by-ref)
  return 0;
}

int bmp_WriteCmd(int fd, uint8_t devAction){
  int rc;
  struct i2c_rdwr_ioctl_data messagebuffer;
  uint8_t datatosend[2];

  datatosend[0]=BMPx8x_CtrlMeas;
  datatosend[1]=devAction;
  //Build a register write command
  //Requires one complete message containing a reg address and command
  struct i2c_msg write_reg[1]={
    {BMPx8x_I2CADDR,0,2,datatosend}
  };

  messagebuffer.nmsgs = 1;                  //One message/action
  messagebuffer.msgs = write_reg;           //load the 'write__reg' message into the buffer
  rc = ioctl(fd, I2C_RDWR, &messagebuffer); //Send the buffer to the bus and returns a send status
  if (rc < 0 ){
    printf("\n");
    printf("%s :Write reg command failed with error :%d\n",__func__,errno);
    printf("This means that device with address :0x%0x failed to receive this command\n",BMPx8x_I2CADDR);
    printf("This command was preceded by a reset if that worked\n");
    printf("and this failed, then possible causes are Delay timing to short (overclock stuffing timing up)\n");
    printf("or bus unstable ,wire length,power supply unstable, terminating resistors.\n");
    printf("\n");
    // Only one of the following lines should be used
    //exit(1);       //Use this line if the function must terminate on failure
    return rc;       //Use this line if it must return to the caller for processing
  }
  return 0;
}

int bmp_Calibration(int fd)
{
  uint8_t rValue[21];
  //printf("Entering Calibration\n");
  if (bmp_ReadInt(fd,rValue,0xAA,22) == 0){
    bmp_ac1=((rValue[0]<<8)|rValue[1]);
    bmp_ac2=((rValue[2]<<8)|rValue[3]);
	 bmp_ac3=((rValue[4]<<8)|rValue[5]);
	 bmp_ac4=((rValue[6]<<8)|rValue[7]);
	 bmp_ac5=((rValue[8]<<8)|rValue[9]);
	 bmp_ac6=((rValue[10]<<8)|rValue[11]);
	 bmp_b1=((rValue[12]<<8)|rValue[13]);
	 bmp_b2=((rValue[14]<<8)|rValue[15]);
	 bmp_mb=((rValue[16]<<8)|rValue[17]);
	 bmp_mc=((rValue[18]<<8)|rValue[19]);
	 bmp_md=((rValue[20]<<8)|rValue[21]);
/*	
	printf ("\nac1:0x%0x\n",bmp_ac1);
	printf ("ac2:0x%0x\n",bmp_ac2);
	printf ("ac3:0x%0x\n",bmp_ac3);
	printf ("ac4:0x%0x\n",bmp_ac4);
	printf ("ac5:0x%0x\n",bmp_ac5);
	printf ("ac6:0x%0x\n",bmp_ac6);
	printf ("b1:0x%0x\n",bmp_b1);
	printf ("b2:0x%0x\n",bmp_b2);
	printf ("mb:0x%0x\n",bmp_mb);
	printf ("mc:0x%0x\n",bmp_mc);
	printf ("md:0x%0x\n",bmp_md);
*/
    return 0;	
  }
  return -1;
}

int WaitForConversion(int fd){
  uint8_t rValues[3];
  int counter=0;
  //Delay can now be reduced by checking that bit 5 of Ctrl_Meas(0xF4) == 0
  do{
    sleepms (BMPx8x_RetryDelay);
    if (bmp_ReadInt(fd,rValues,BMPx8x_CtrlMeas,1) != 0 ) return -1;
    counter++;
    //printf("GetPressure:\t Loop:%i\trValues:0x%0x\n",counter,rValues[0]);
  }while ( ((rValues[0] & 0x20) != 0)  &&  counter < 20 );  
  return 0;
}

// Calculate calibrated pressure 
// Value returned will be in hPa
int bmp_GetPressure(int fd, double *Pres)
{
  unsigned int up;  
  uint8_t rValues[3];
    
  // Pressure conversion with oversampling 0x34+ BMPx8x_OverSampling 'bit shifted'
  if (bmp_WriteCmd(fd, (BMPx8x_PresConversion0+(BMPx8x_OverSampling<<6)))!=0) return -1;
  
  //Delay gets longer the higher the oversampling must be at least 26 ms plus a bit for turbo 
  //clock error ie 26 * 1000/700 or 38 ms
  //sleepms (BMPx8x_minDelay + (4<<BMPx8x_OverSampling));  //39ms at oversample = 3
  
  //Code is now 'turbo' overclock independent 
  sleepms (BMPx8x_minDelay);
  if (WaitForConversion(fd) !=0 ) return -1;  
  
  //printf ("\nDelay:%i\n",(BMPx8x_minDelay+(4<<BMPx8x_OverSampling))); 
  if (bmp_ReadInt(fd, rValues, BMPx8x_Results,3) !=0 ) return -1;
  up = (((unsigned int) rValues[0] << 16) | ((unsigned int) rValues[1] << 8) | (unsigned int) rValues[2]) >> (8-BMPx8x_OverSampling);
  
  int x1, x2, x3, b3, b6, p;
  unsigned int b4, b7;
  
  b6 = bmp_b5 - 4000;
  x1 = (bmp_b2 * (b6 * b6)>>12)>>11;
  x2 = (bmp_ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((int)bmp_ac1)*4 + x3)<<BMPx8x_OverSampling) + 2)>>2;
  
  x1 = (bmp_ac3 * b6)>>13;
  x2 = (bmp_b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (bmp_ac4 * (unsigned int)(x3 + 32768))>>15;
  
  b7 = ((unsigned int)(up - b3) * (50000>>BMPx8x_OverSampling));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
	
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  *Pres = ((double)p/100);  
  return 0;
}

// Calculate calibrated temperature
// Value returned will be in units of 0.1 deg C
int bmp_GetTemperature(int fd, double *Temp)
{
  unsigned int ut;
  uint8_t rValues[2];
  
  if (bmp_WriteCmd(fd, BMPx8x_TempConversion)!=0) return -1;
  //Code is now 'turbo' overclock independent 
  sleepms (BMPx8x_minDelay);
  if (WaitForConversion(fd) !=0 ) return -1;  

  if (bmp_ReadInt(fd, rValues, BMPx8x_Results,2) !=0 ) return -1;
  ut=((rValues[0]<<8)|rValues[1]);  

  int x1, x2;
  x1 = (((int)ut - (int)bmp_ac6)*(int)bmp_ac5) >> 15;
  x2 = ((int)bmp_mc << 11)/(x1 + bmp_md);
  bmp_b5 = x1 + x2;

  double result = ((bmp_b5 + 8)>>4);  
  *Temp = result/10;
  return 0;
}

double bmp_altitude(double p){
  return 145437.86*(1- pow((p/1013.25),0.190294496)); //return feet
  //return 44330*(1- pow((p/1013.25),0.190294496)); //return meters
}

double bmp_qnh(double p,double StationAlt){
  return p / pow((1-(StationAlt/145437.86)),5.255) ; //return hPa based on feet
  //return p / pow((1-(StationAlt/44330)),5.255) ; //return hPa based on feet
}

double ppl_DensityAlt(double PAlt,double Temp){
  double ISA = 15 - (1.98*(PAlt/1000));
  return PAlt+(120*(Temp-ISA)); //So,So density altitude
}

int main(int argc, char **argv)
{
   int fd;
   double temperature, pressure;
   double PAlt;
   fd=i2c_Open(I2CBus);
	printf ("\nCalibration:%i (0= worked)\n",bmp_Calibration(fd));
	printf ("Return:%i\tTemperature\t:%.1f C\n",bmp_GetTemperature(fd,&temperature),temperature);
	printf ("Return:%i\tPressure\t:%.2f hPa/mB\tThis is known as station pressure\n",bmp_GetPressure(fd,&pressure),pressure);
	PAlt=bmp_altitude(pressure);
	printf ("\t\tP.Altitude\t:%.0f ft\tThis is known as a Pressure Altitude\n",PAlt);
	printf ("\t\tQNH\t\t:%.2f hPa/mB\tSetting required on an Aircraft\n",bmp_qnh(pressure,5085)); //5085 ft is my station elevation
	printf ("\t\t\t\t\t\taltimeter for it to read Elevation while on the ground\n");	             //use your's
	printf ("\t\t\t\t\t\tand Altitude while in the air\n");
	printf ("\t\t\t\t\t\tSensor Elevation MUST BE KNOWN ACCURATELY\n");						
	printf ("\t\tD.Altitude\t:%.0f ft\tThis is known as a density Altitude\n\n",ppl_DensityAlt(PAlt,temperature));
   close (fd);

	return 0;
}