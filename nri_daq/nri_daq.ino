#include <IntervalTimer.h>
#include <SPI.h>

#include "TimerObject.h"
TimerObject *timer1 = new TimerObject(12);


#include "Wire.h"
#include <math.h>

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelGyroMag;
uint8_t ajx, ajy, ajz;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

uint8_t buffer_accel[6];
uint8_t buffer_gyro[6];
uint8_t buffer_mag[6];



int16_t mx_max,  mx_min, my_max, my_min, mz_max, mz_min;

float mx_off, mx_sca, my_off, my_sca, mz_off, mz_sca;

#define RESET       0x10

#define LED_1     4
#define LED_2     3
#define LED_3     6
#define LED_4     5

#define SETUP_ACC     0x64
#define CONV_ACC    0xC0
#define CS_ACC      7

#define SETUP_FT    0x6B
#define CONV_FT     0xD8
#define CS_FT     9

#define SAMPLE_RATE   500 // Delay in us between samples

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

#define START 0x01
#define STOP 0x02
#define PING 0x03
#define CALI 0x00

IntervalTimer irq;


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
  static long int pre = 0;

  j++;
  

  Serial.flush();

  Serial.write(buffer_accel, 6);
  Serial.write(buffer_gyro, 6);
  Serial.write(buffer_mag, 6);
  
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
  //  Serial.print(' ');


  digitalWrite(CS_FT, LOW);
  SPI.transfer(CONV_FT);
  digitalWrite(CS_FT, HIGH);

  digitalWrite(CS_ACC, LOW);
  SPI.transfer(CONV_ACC);
  digitalWrite(CS_ACC, HIGH);
  



//  Serial.print(ax);
//  Serial.print('\t');
//  Serial.print(ay);
//  Serial.print('\t');
//  Serial.print(az);
//  Serial.print('\t');
//  Serial.print(gx);
//  Serial.print('\t');
//  Serial.print(gy);
//  Serial.print('\t');
//  Serial.print(gz);
//  Serial.print('\t');
//  Serial.print((mx-mx_off)/mx_sca);
//  Serial.print('\t');
//  Serial.print((my-my_off)/my_sca);
//  Serial.print('\t');
//  Serial.print((mz-mz_off));
//  Serial.print('\t');
//  Serial.print(atan2((mx-mx_off)*mx_sca, (my-my_off)*my_sca));
//  Serial.print(' ');
//  Serial.println(micros() - pre);
//  pre = micros();
  

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


void readIMU(){

  accelGyroMag.getAccelByte(buffer_accel);
  accelGyroMag.getGyroByte(buffer_gyro);
  accelGyroMag.getMagByte(buffer_mag);

}

void calibrateCompass(float* mx_off, float* mx_sca, float* my_off, float* my_sca, float* mz_off, float* mz_sca){
  Serial.println(F("Calibrating compass. Send any key to abort."));
  Serial.flush();
  static long int pp;

//  accelGyroMag.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelGyroMag.getMagnetometer(&mx, &my, &mz);
  

  mx_max = mx;
  mx_min = mx-1;
  my_max = my;
  my_min = my-1;
  mz_max = mz;
  mz_min = mz-1;
  while(1) {
//    
    pp = micros();
    accelGyroMag.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accelGyroMag.getMagnetometer(&mx, &my, &mz);
    Serial.print(micros()-pp); Serial.print(' ');
    Serial.print(mx); Serial.print(' ');
    Serial.print(my); Serial.print(' ');
    Serial.println(mz);
    if (mx>mx_max){
      mx_max = mx;
    }
    else if (mx<mx_min){
      mx_min = mx;
    }
    if (my>my_max){
      my_max = my;
    }
    else if (my<my_min){
      my_min = my;
    }
    if (mz>mz_max){
      mz_max = mz;
    }
    else if (mz<mz_min){
      mz_min = mz;
    }
  
    delay(10);
  
  
    if (Serial.available()){
      
      *mx_off = (mx_max+mx_min)/2.0;
      *my_off = (my_max+my_min)/2.0;
      *mz_off = (mz_max+mz_min)/2.0;
      *mx_sca = (mx_max-mx_min) * 1.0 / (mz_max-mz_min);
      *my_sca = (my_max-my_min) * 1.0 / (mz_max-mz_min);
      *mz_sca = 1.0;
      Serial.flush();
      Serial.print(mz_min); Serial.print(' ');
      Serial.print(mz_max); Serial.print(' ');
      Serial.print(*mx_off); Serial.print(' ');
      Serial.print(*mx_sca); Serial.print(' ');
      Serial.print(*my_off); Serial.print(' ');
      Serial.print(*my_sca); Serial.print(' ');
      Serial.print(*mz_off); Serial.print(' ');
      Serial.print(*mz_sca); Serial.print(' ');
      return;
    }
  }
}


void setup(void)
{

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
  while (!Serial);

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
  TWBR = 12;
  accelGyroMag.initialize();
  accelGyroMag.setDLPFMode(MPU9150_DLPF_BW_20);
  accelGyroMag.setFullScaleGyroRange(MPU9150_GYRO_FS_500);
  accelGyroMag.setFullScaleAccelRange(MPU9150_ACCEL_FS_4);
  
  accelGyroMag.getAdj(&ajx, &ajy, &ajz);

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

    if (message[0] == CALI){
      calibrateCompass(&mx_off, &mx_sca, &my_off, &my_sca, &mz_off, &mz_sca);
    }
    if (message[0] == START)
    {
      
      if (packet_length > 1)
      {
        sample_rate = 1000000 / ((int)(message[1] << 8) + (int)message[2]);
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

    if (!Serial.dtr())
      irq.end();
    interrupts();

  }

}
