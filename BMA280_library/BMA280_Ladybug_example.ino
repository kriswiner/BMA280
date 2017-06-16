 /* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The BMA280 is an inexpensive (~$1), three-axis, high-resolution (14-bit) acclerometer in a tiny 2 mm x 2 mm LGA12 package with 32-slot FIFO, 
 *  two multifunction interrupts and widely configurable sample rate (15 - 2000 Hz), full range (2 - 16 g), low power modes, 
 *  and interrupt detection behaviors. This accelerometer is nice choice for low-frequency sound and vibration analysis,
 *  tap detection and simple orientation estimation.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "BMA280.h"

#define myLed   13  // board led
#define intPin1 4   // interrupt1 pin definitions
#define intPin2 1   // interrupt2 pin definitions

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      BW_7_81Hz, BW_15_63Hz, BW_31_25Hz, BW_62_5Hz, BW_125Hz, BW_250Hz, BW_500Hz, BW_1000Hz
      normal_Mode, deepSuspend_Mode, lowPower_Mode, suspend_Mode
      sleep_0_5ms, sleep_1ms, sleep_2ms, sleep_4ms, sleep_6ms, sleep_10ms, sleep_25ms, sleep_50ms, sleep_100ms, sleep_500ms, sleep_1000ms
*/ 
uint8_t Ascale = AFS_2G, BW = BW_125Hz, power_Mode = normal_Mode, sleep_dur = sleep_0_5ms, tapStatus, tapType;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
uint32_t delt_t = 0, count = 0; // used to control display output rate

bool newData = false;
bool newTap  = false;

  BMA280 BMA280(intPin1, intPin2);
  
void setup() {
  
  // Set up the led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // active HIGH
  
  Serial.begin(115200);
  delay(4000);
  Wire.begin(); // set master mode, on Ladybug uses pins 20/21 by default
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);
 
  BMA280.I2Cscan();  // check for I2C devices on the bus

  // Read the BMS280 Chip ID register, this is a good test of communication
  Serial.println("BMA280 accelerometer...");
  byte c = BMA280.getChipID();  // Read CHIP_ID register for BMA280
  Serial.print("BMA280 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0xFB, HEX);
 
  if (c == 0xFB) // CHIP_ID should always be 0xFB
  {  
   Serial.println("BMA280 is online...");
   
   aRes = BMA280.getAres(Ascale);                                     // get sensor resolutions, only need to do this once
   BMA280.selfTestBMA280();                                           // perform sensor self test
   BMA280.resetBMA280();                                              // software reset before initialization
   delay(1000);                                                       // give some time to read the screen
   BMA280.initBMA280(Ascale, BW, power_Mode, sleep_dur);              // initialize sensor for data acquisition
   BMA280.fastCompensationBMA280();                                   // quickly estimate offset bias

   attachInterrupt(intPin1, myinthandler1, RISING);  // define interrupt for INT1 pin output of BMA280
   attachInterrupt(intPin2, myinthandler2, RISING);  // define interrupt for INT2 pin output of BMA280

  }
  
  else Serial.print("BMA280 is not answering...0x"); Serial.println(c, HEX);
  
}


void loop() {
 
    if(newData == true) {  // On interrupt, read data
     newData = false;  // reset newData flag

     BMA280.readBMA280AccelData(accelCount); // get 14-bit signed accel data
     
     // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes/4.0f;  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes/4.0f;   
     az = (float)accelCount[2]*aRes/4.0f;  
    }

    // Check for taps
    if(newTap == true) {  // On interrupt, identify tap
      newTap = false;
 
      tapType = BMA280.getTapType();
      if(tapType & 0x20) Serial.println("Single tap detected!");
      if(tapType & 0x10) Serial.println("Double tap detected!"); 
      
      tapStatus = BMA280.getTapStatus();  // Read tap status register

      if(tapStatus & 0x80) {
        Serial.println("Tap is negative");
        }
      else {
        Serial.println("Tap is positive");
      }

       if(tapStatus & 0x40) {
        Serial.println("Tap is on x");
       }
       if(tapStatus & 0x20) {
        Serial.println("Tap is on y");
       }
       if(tapStatus & 0x10) {
        Serial.println("Tap is on z");
       }      
       
    }
  
    // Serial print and/or display at 2Hz rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update serial once per half-second independent of read rate
   
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");

    tempCount = BMA280.readBMA280GyroTempData();  // Read the gyro adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Gyro chip temperature in degrees Centigrade
   // Print temperature in degrees Centigrade      
    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

    digitalWrite(myLed, !digitalRead(myLed)); // blink led at end of loop
    count = millis();  
    }

}

/* end of main program*/

void myinthandler1()
{
  newData = true;  // Just set flag when interrupt received, don't try reading data in an interrupt handler
}

void myinthandler2()
{
  newTap = true;
}
