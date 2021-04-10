#include <Arduino.h>

/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout
  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.
  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.
  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"
#define INTERRUPT_PIN 18
// HR
MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

typedef struct _hr_rate_t
{
  long ir;
  float bpm;
  int bpm_avg;
  bool is_valid;
} hr_rate_t;

typedef struct _acc_data_t
{
  bool isMoving;
  bool isOnFeet;
  bool isOnSide;
} acc_data_t;

// ACC
MPU6050 accelgyro;

void hr_rate_init();
void acc_init();
void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");
  hr_rate_init();
  acc_init();
}

hr_rate_t hr_rate_meas();
acc_data_t acc_loop();
void loop()
{
  hr_rate_t hr_rate = hr_rate_meas();
  Serial.print("IR=");
  Serial.print(hr_rate.ir);
  Serial.print(", BPM=");
  Serial.print(hr_rate.bpm);
  Serial.print(", BPM_AVG=");
  Serial.print(hr_rate.bpm_avg);
  Serial.print(", IS_VALID=");
  Serial.print(hr_rate.is_valid);

  acc_data_t acc_data = acc_loop();
  Serial.print("isMoving=");
  Serial.print(acc_data.isMoving);
  Serial.print(", isOnFeet=");
  Serial.print(acc_data.isOnFeet);
  Serial.print(", isOnSide=");
  Serial.print(acc_data.isOnSide);
  Serial.println();

  delay(10);
}

void hr_rate_init()
{
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(0x4F);                    //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x00); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  //Turn off Green LED
}

hr_rate_t hr_rate_meas()
{
  hr_rate_t hr;
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE;                    //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  hr.ir = irValue;
  hr.bpm = beatsPerMinute;
  hr.bpm_avg = beatAvg;
  hr.is_valid = true;
  if (irValue < 50000)
    hr.is_valid = false;

  return hr;
}

void acc_init()
{
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

#define MAG_AVG_APLHA (0.01/0.5) // Probing period divided by time constant (63% settling time)
int16_t ax, ay, az;
VectorInt16 vector;
float mag_avg = 18000;
acc_data_t acc_loop()
{
  acc_data_t acc_data;
  accelgyro.getAcceleration(&ax, &ay, &az);
  vector = VectorInt16(ax, ay, az);
  float mag = vector.getMagnitude();
  acc_data.isMoving = false;
  acc_data.isOnFeet = false;
  acc_data.isOnSide = false;
  mag_avg = mag_avg + MAG_AVG_APLHA*(mag - mag_avg);
  if(abs(mag_avg - mag)/mag_avg > 0.1) {
    acc_data.isMoving = true;
  } else {
    if(az > 2*ax && az > 2*ay && az > 0) {
      acc_data.isOnFeet = true;
    } else if(abs(ax) > 2*az && abs(ax) > 2*ay) {
      acc_data.isOnSide = true;
    }
  }
  return acc_data;
}