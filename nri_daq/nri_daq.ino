#include <IntervalTimer.h>
#include <SPI.h>

/*
class SPIClass {
    public:
        void begin(byte sck, byte miso, byte mosi) {
            delay_ = 0;
            sck_ = sck;
            miso_ = miso;
            mosi_ = mosi;

            pinMode(sck_, OUTPUT);
            pinMode(miso_, INPUT);
            pinMode(mosi_, OUTPUT);
        }

        uint8_t transfer(uint8_t out) {
            // mode 0
            //  - CPOL = 0, clock starts low
            //  - CPHA = 0, input on rising edge and output on falling edge

            // clock starts low
            digitalWrite(sck_, LOW);
            
            uint8_t in = 0;
            for (byte i = 8; i > 0; i--) {
                uint8_t bit = 1 << (i-1);

                // stage output
                digitalWrite(mosi_, !!(out & bit));

                // rising edge
                digitalWrite(sck_, HIGH);

                // read input
                if (digitalRead(miso_)) {
                    in |= bit;
                }

                // falling edge
                digitalWrite(sck_, LOW);
            }

            return in;
        }

    private:
        byte sck_, miso_, mosi_;
        uint32_t delay_; // 1/4 period (us)
} SPI;
*/

#include "Wire.h"
#include <math.h>

#include "TimerObject.h"
//TimerObject *timer1 = new TimerObject(0);

#include "I2Cdev.h"
#include "LSM303DLHC.h"
#include "L3GD20H.h"

LSM303DLHC accelMag;
L3GD20H gyro;


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

volatile uint8_t buffer_accel[200];
volatile uint8_t buffer_gyro[200];
volatile uint8_t buffer_mag[6];


volatile uint8_t number_gyro = 0;
volatile uint8_t number_accel = 0;
volatile bool imuReady = 0;
volatile bool imuLocked = 0;
bool reading = false;

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
SPISettings spi_settings(1000000L, MSBFIRST, SPI_MODE0);

void readIMU(){
  while (imuLocked == 1) { } // spin
  
  number_accel = accelMag.getAccelFIFOStoredSamples();
  
  for (byte i=0; i<number_accel; i++){
    accelMag.getAccelByte((uint8_t*)&buffer_accel[6*i]);
  }
  
  //accelMag.setAccelFIFOMode(LSM303DLHC_FM_BYBASS);
  //accelMag.setAccelFIFOMode(LSM303DLHC_FM_STREAM);
  
  accelMag.getMagByte((uint8_t*)buffer_mag);
  
  number_gyro = gyro.getFIFOStoredDataLevel();
  
  for (byte i=0; i<number_gyro; i++){ 
    gyro.getGyroByte((uint8_t*)&buffer_gyro[6*i]);
  }
  
  //gyro.setFIFOMode(L3GD20H_FM_BYPASS);
  //gyro.setFIFOMode(L3GD20H_FM_STREAM);
  
  /*
  unsigned long m = micros();
  buffer_mag[0] = (m & 0xFF000000) >> 24;
  buffer_mag[1] = (m & 0x00FF0000) >> 16;
  buffer_mag[2] = (m & 0x0000FF00) >> 8;
  buffer_mag[3] = (m & 0x000000FF);
  buffer_mag[4] = number_accel;
  buffer_mag[5] = number_gyro;
  number_accel = 0;
  number_gyro = 0;
  //*/
  
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

#define w(b) { data[k] = (uint8_t)(b); sum += data[k]; ++k; }
#define wsum() { data[k] = (uint8_t)(sum); ++k; }
#define wlen(ll) { w((ll) >> 8); w((ll)); sum = 0; }

void readADC(void)
{
  noInterrupts();
  static byte j = 0;
  uint8_t data[500];
  byte i = 0;
  
  j++;

  int k = 0;
  uint8_t sum = 0;
  bool checksum = true;
  w('a');
  w('a');
  w('a');
  
  if (imuReady==1){
    imuLocked = 1;
    wlen(33 + 6*(number_accel+number_gyro+1) + checksum);
    w(number_accel); w(number_gyro);
    for (int l = 0; l < 6*number_accel; l++) w(buffer_accel[l]);
    for (int l = 0; l < 6*number_gyro;  l++) w(buffer_gyro[l]);
    for (int l = 0; l < 6;              l++) w(buffer_mag[l]);
    //Serial.write((uint8_t*)buffer_accel, 6*number_accel);
    //Serial.write((uint8_t*)buffer_gyro,  6*number_gyro);
    //Serial.write((uint8_t*)buffer_mag,   6);
    imuReady = 0;
    imuLocked = 0;
  }
  else{
    wlen(31 + checksum);
  }

  //int sk = 0;
  SPI.beginTransaction(spi_settings);
  for (i = 0; i < 12; i++)
  {
    //if (i == 0) { sk = k; }
    digitalWrite(CS_FT, LOW);
    w(SPI.transfer(0x00));
    digitalWrite(CS_FT, HIGH);
  }

  for (i = 12; i < 30; i++)
  {
    digitalWrite(CS_ACC, LOW);
    w(SPI.transfer(0x00));
    digitalWrite(CS_ACC, HIGH);
  }

  parkingSpot(false);
  w(PS_Status);
  if (checksum) wsum();


  /*
  char buf[50] = "";
  sprintf(buf, "%02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X    %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X",
               data[sk], data[sk+1], data[sk+2], data[sk+3], data[sk+4], data[sk+5], data[sk+6], data[sk+7], data[sk+8], data[sk+9], data[sk+10], data[sk+11],
               data[sk+12], data[sk+13], data[sk+14], data[sk+15], data[sk+16], data[sk+17], data[sk+18], data[sk+19], data[sk+20], data[sk+21], data[sk+22], data[sk+23], data[sk+24], data[sk+25], data[sk+26], data[sk+27], data[sk+28], data[sk+29]);
  Serial.println(buf);
  */
  Serial.write(data, k);
  //Serial.flush();
  
  digitalWrite(CS_FT, LOW);
  SPI.transfer(CONV_FT);
  digitalWrite(CS_FT, HIGH);

  digitalWrite(CS_ACC, LOW);
  SPI.transfer(CONV_ACC);
  digitalWrite(CS_ACC, HIGH);
  SPI.endTransaction();
  interrupts();
}
void pulseCS(char pin)
{
  // Pulses the CS line in between SPI bytes
  digitalWrite(pin, HIGH);
  delayMicroseconds(1);
  digitalWrite(pin, LOW);
}

void setupFT(void)
{
  SPI.beginTransaction(spi_settings);
  digitalWrite(CS_FT, LOW);
  SPI.transfer(RESET);
  pulseCS(CS_FT);
  SPI.transfer(SETUP_FT);
  SPI.transfer(0xFF);
  pulseCS(CS_FT);
  SPI.transfer(CONV_FT);
  digitalWrite(CS_FT, HIGH);
  SPI.endTransaction();
}

void setupACC(void)
{
  SPI.beginTransaction(spi_settings);
  digitalWrite(CS_ACC, LOW);
  SPI.transfer(RESET);
  pulseCS(CS_ACC);
  SPI.transfer(SETUP_ACC);
  pulseCS(CS_ACC);
  SPI.transfer(CONV_ACC);
  digitalWrite(CS_ACC, HIGH);
  SPI.endTransaction();
}

void parkingSpot(bool write)
{
  //if the status of the parking spot is requested this function is called
  PS_Status = 0;
  if(digitalRead(opto)==HIGH)PS_Status=PS_Status   |1;
  if(digitalRead(stick)==HIGH)PS_Status=PS_Status  |2;
  if(digitalRead(biotac)==HIGH)PS_Status=PS_Status |4;
  if (write) { Serial.write(0x30 + PS_Status); }
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

  // Start SPI
  //SPI.begin(13, 12, 11);
  SPI.begin();
  //SPI.usingInterrupt(irq);
  //SPI.setClockDivider(SPI_CLOCK_DIV8);
  //SPI.setDataMode(SPI_MODE0);
  //SPI.setBitOrder(MSBFIRST);

  Wire.begin();
  Wire.setClock(400000L);
  accelMag.initialize();
  accelMag.rebootAccelMemoryContent();
  accelMag.setAccelFullScale(4);
  accelMag.setAccelBlockDataUpdateEnabled(true);
  accelMag.setAccelHighResOutputEnabled(true);
  accelMag.setAccelOutputDataRate(1344);
  accelMag.setAccelLowPowerEnabled(false);
  accelMag.setMagOutputDataRate(220);
  accelMag.setMagGain(230);
  accelMag.setMagMode(LSM303DLHC_MD_CONTINUOUS);


  gyro.initialize();
  gyro.rebootMemoryContent();
  gyro.setFullScale(500);
  gyro.setBlockDataUpdateEnabled(true);
  gyro.setOutputDataRate(800);
  gyro.setBandwidthCutOffMode(L3GD20H_BW_MED_LOW);
   
  //timer1->setOnTimer(&readIMU);
}

void loop(void)
{
//  Serial.println("asdasd");
  byte message[5];
  byte sample_rate_buff[2];
  int sample_rate;
  byte packet_length;
  //timer1->Update();
  if (reading) readIMU();

  if (Serial.available())
  {
    noInterrupts();
    packet_length = Serial.available();

    for (int i = 0; i < packet_length; i++)
    {
      message[i] = Serial.read();
    }

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
      //accelMag.rebootAccelMemoryContent();
      accelMag.setAccelFIFOEnabled(true);
      gyro.setFIFOEnabled(false);
      gyro.setFIFOMode(L3GD20H_FM_BYPASS);
      gyro.setFIFOMode(L3GD20H_FM_STREAM);
      //gyro.rebootMemoryContent();
      gyro.setFIFOEnabled(true);
      
      irq.begin(readADC, sample_rate);
      //timer1->Start();
      reading = true;
    }

    if (message[0] == STOP){
      irq.end();
      //timer1->Stop();
      reading = false;
    }

    if (message[0] == PING)
      {
        irq.end();
        reading = false;
        Serial.write(0x01);
      }
    
    if (message[0] == REQUEST_PS)
      {
        parkingSpot(true);
      }
    
    if (!Serial.dtr())
    {
      irq.end();
      reading = false;
    }
    
    interrupts();

  }

}
