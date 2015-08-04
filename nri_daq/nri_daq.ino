#include <IntervalTimer.h>
#include <SPI.h>

#include "Wire.h"
#include <math.h>

#include "TimerObject.h"
TimerObject *timer1 = new TimerObject(1);

#include "I2Cdev.h"
#include "LSM303DLHC.h"
#include "L3GD20H.h"

LSM303DLHC accelMag;
L3GD20H gyro;


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

uint8_t buffer_accel[50];
uint8_t buffer_gyro[50];
uint8_t buffer_mag[6];


uint8_t number_gyro;
uint8_t number_accel;
bool imuReady = 0;

#define RESET       0x10

#define opto      0
#define stick     1
#define biotac    2
byte PS_Status =  0;

#define LED_1     4
#define LED_2     3
#define LED_3     6
#define LED_4     5

#define PS_request    0x04

#define SETUP_ACC     0x64
#define CONV_ACC    0xC0
#define CS_ACC      7

#define SETUP_FT    0x6B
#define CONV_FT     0xD8
#define CS_FT     9

#define SAMPLE_RATE   333 // Delay in us between samples

#define SCAN_MODE00   0x0
#define SCAN_MODE01   0x1
#define SCAN_MODE10   0x2
#define SCAN_MODE11   0x3

#define CLOCK_MODE00  0x0
#define CLOCK_MODE01  0x1
#define CLOCK_MODE10  0x2
#define CLOCK_MODE11  0x3

#define REF_INT_DELAY   0x0
#define REF_INT_NODELAY 0x2
#define REF_EXT_SINGLE  0x1
#define REF_EXT_DIFF  0x3

#define SINGLE    0x0
#define UNIPOLAR  0x2
#define BIPOLAR   0x3

#define START '1' 
#define STOP '2'
#define PING '3'
#define REQUEST_PS '4'

IntervalTimer irq;

void readIMU(){

  number_accel = accelMag.getAccelFIFOStoredSamples();
  for (byte i=0; i<number_accel; i++){
    accelMag.getAccelByte(&buffer_accel[6*i]);
  }
  accelMag.getMagByte(buffer_mag);
  number_gyro = gyro.getFIFOStoredDataLevel();
  for (byte i=0; i<number_gyro; i++){ 
    gyro.getGyroByte(&buffer_gyro[6*i]);
  }
  imuReady = 1;

}


byte convReg(byte channel, byte scan_mode, byte temp)
{
  byte command = 0x80 | (channel << 3) | (scan_mode << 1) | (temp);
  return command;
}

byte setupReg(byte cksel, byte refsel, byte diffsel)
{
  byte command = 0x40 | (cksel << 4) | (refsel << 2) | (diffsel);
  return command;
}

void readADC(void)
{

  static byte j = 0;
  static byte data[50];
  byte i = 0;

  
  j++;


  Serial.flush();
  Serial.write("aaa");
  
  
  if (imuReady==1){
    Serial.write(33+6*(number_accel+number_gyro+1));
    Serial.write(number_accel); Serial.write(number_gyro);
    Serial.write(buffer_accel, 6*number_accel);
    Serial.write(buffer_gyro,  6*number_gyro);
    Serial.write(buffer_mag,   6);
    imuReady = 0;
  }
  else{
    Serial.write(31);
  }

  for (i = 0; i < 12; i++)
  { 
    digitalWrite(CS_FT, LOW);
    data[i] = SPI.transfer(0x00);
    digitalWrite(CS_FT, HIGH);
    delayMicroseconds(2);
  }

  for (i = 12; i < 30; i++)
  { 
    digitalWrite(CS_ACC, LOW);
    data[i] = SPI.transfer(0x00);
    digitalWrite(CS_ACC, HIGH);
    delayMicroseconds(2);
  }

  data[30] = j;

  Serial.write(data, 31);
  
  digitalWrite(CS_FT, LOW);
  SPI.transfer(CONV_FT);
  digitalWrite(CS_FT, HIGH);

  digitalWrite(CS_ACC, LOW);
  SPI.transfer(CONV_ACC);
  digitalWrite(CS_ACC, HIGH);


}
void pulseCS(char pin)
{
  // Pulses the CS line in between SPI bytes
  digitalWrite(pin, HIGH);
  delayMicroseconds(2);
  digitalWrite(pin, LOW);
}

void setupFT(void)
{
  digitalWrite(CS_FT, LOW);
  SPI.transfer(RESET);
  pulseCS(CS_FT);
  SPI.transfer(SETUP_FT);
  SPI.transfer(0xFF);
  pulseCS(CS_FT);
  SPI.transfer(CONV_FT);
  digitalWrite(CS_FT, HIGH);
}

void setupACC(void)
{
  digitalWrite(CS_ACC, LOW);
  SPI.transfer(RESET);
  pulseCS(CS_ACC);
  SPI.transfer(SETUP_ACC);
  pulseCS(CS_ACC);
  SPI.transfer(CONV_ACC);
  digitalWrite(CS_ACC, HIGH);
}

void parkingSpot(void)
{
  //if the status of the parking spot is requested this function is called
  if(digitalRead(opto)==HIGH)PS_Status=PS_Status   |1;
  if(digitalRead(stick)==HIGH)PS_Status=PS_Status  |2;
  if(digitalRead(biotac)==HIGH)PS_Status=PS_Status |4;
  Serial.write(PS_Status);
  PS_Status = 0;
}

void setup(void)
{

  //Declare parking spot pins as input
  
  pinMode(opto,INPUT);
  pinMode(stick,INPUT);
  pinMode(biotac,INPUT);
  
  // Turn on LEDS (only LED_4 connected)

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_3, HIGH);
  digitalWrite(LED_4, HIGH);

  // Start USB
  Serial.begin(115200);
  

  // Declare chip select pins, set to idle high
  pinMode(CS_ACC, OUTPUT);
  pinMode(CS_FT, OUTPUT);
  digitalWrite(CS_ACC, HIGH);
  digitalWrite(CS_FT, HIGH);

  // Start SPI, 8Mhz speed, Defaults to mode 0
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);

   Wire.begin();
  accelMag.initialize();
  accelMag.setAccelFullScale(4);
  accelMag.setAccelBlockDataUpdateEnabled(true);
  accelMag.setAccelHighResOutputEnabled(true);
  accelMag.setAccelOutputDataRate(1344);
  accelMag.setMagOutputDataRate(220);
  accelMag.setMagGain(230);
  accelMag.setMagMode(LSM303DLHC_MD_CONTINUOUS);


  gyro.initialize();
  gyro.setFullScale(500);
  gyro.setBlockDataUpdateEnabled(true);
  gyro.setOutputDataRate(800);
  gyro.setBandwidthCutOffMode(L3GD20H_BW_MED_LOW);

   
  timer1->setOnTimer(&readIMU);
}

void loop(void)
{
//  Serial.println("asdasd");
  byte message[5];
  byte sample_rate_buff[2];
  int sample_rate;
  byte packet_length;
  timer1->Update();

  if (Serial.available())
  {
    noInterrupts();
    packet_length = Serial.available();

    for (int i = 0; i < packet_length; i++)
    {
      message[i] = Serial.read();
    }

    Serial.flush();

    if (message[0] == START)
    {
      if (packet_length > 1)
      {
        sample_rate = 1000000/((int)(message[1]<<8) + (int)message[2]);
        // delay(1);
      }

      else
      {
        sample_rate = SAMPLE_RATE;
      }
      
      // delay(1);
      setupACC();
      setupFT();
      
      delayMicroseconds(sample_rate);
      accelMag.setAccelFIFOEnabled(false);
      accelMag.setAccelFIFOMode(LSM303DLHC_FM_BYBASS);
      accelMag.setAccelFIFOMode(LSM303DLHC_FM_STREAM);
      accelMag.setAccelFIFOEnabled(true);
      accelMag.rebootAccelMemoryContent();
      gyro.setFIFOEnabled(false);
      gyro.setFIFOMode(L3GD20H_FM_BYPASS);
      gyro.setFIFOMode(L3GD20H_FM_STREAM);
      gyro.setFIFOEnabled(true);
      gyro.rebootMemoryContent();
      irq.begin(readADC, sample_rate);
      timer1->Start();
    }

    if (message[0] == STOP){
      irq.end();
      timer1->Stop();
    }

    if (message[0] == PING)
      {
        irq.end();
        Serial.flush();
        Serial.write(0x01);
      }
    
    if (message[0] == REQUEST_PS)
      {
        parkingSpot();
      }
    
    if (!Serial.dtr())
      irq.end();  
    interrupts();

  }

}