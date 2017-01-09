#include <MotorDriver.h>

/*
   Line Folower Robot with autocalibration of analog Sensor Values.
   This programs automaticaly calibrates its sensor values to follwo line where ambient conditions or line background is changing.
   Motor Driver : L293D H Bridge
   Sensors : array of 5 Photdiode LED pair
   Requires MotorDriver.h library https://github.com/siddharthdeore/MotorDriver
*/

//update Your motor driver pins here
#define LeftMotorPinPWM 5
#define LeftMotorPinA 4
#define LeftMotorPinB 5
#define rightMotorPinPWM 9
#define rightMotorPinA 7
#define rightMotorPinB 8

// update your sensor pins here from left to right of your robot
int sensorPins[] = {A0 , A1 , A2 , A3 , A4};

Motor LeftMotor(LeftMotorPinPWM, LeftMotorPinA, LeftMotorPinB);
Motor RightMotor(rightMotorPinPWM, rightMotorPinA, rightMotorPinB);



float sensorValue[5];
float sensorValueMin[5] = {1024, 1024, 1024, 1024, 1024};
float sensorValueMax[5] = {0, 0, 0, 0, 0};
float threshold[5] = {10, 10, 10, 10, 10};
float sensorValueAvg[5] = {512, 512, 512, 512, 512};
boolean digitalValue[5];
short error = 0;
void setup()
{
  Serial.begin(9600);
  for (short i = 0; i < 5; i++)
  {
    pinMode(sensorPins[i], INPUT);
  }
}
int SPD = 50;
float PID;
float kp = 5.5, ki = 1.0, kd = 0.5;
float last_error, integ, deriv;
void loop()
{
  updateSensor();
  updateMinMax();

  // Calculate PID
  PID = error * kp + integ * ki + deriv * kd;
  integ += error;
  deriv = error - last_error;
  last_error = error;

  //rotate Motors
  LeftMotor.rotate(SPD + PID);
  RightMotor.rotate(SPD - PID);
}

void calculateError()
{
  error = 0;
  short sum = 0;
  for (short i = 0; i < 5; i++)
  {
    sum += digitalValue[i];
    error += digitalValue[i] * (i + 1);
  }
  error = error / sum - 3;
}
void updateSensor()
{
  for (short i = 0; i < 5; i++)
  {
    sensorValue[i] = analogRead(sensorPins[i]);
    digitalValue[i] = sensorValue[i] > (sensorValueAvg[i] + threshold[i]);
  }
}
void updateMinMax()
{
  for (short i = 0; i < 5; i++)
  {
    sensorValueMin[i] = sensorValue[i] < sensorValueMin[i] ? sensorValue[i] : sensorValueMin[i];
    sensorValueMax[i] = sensorValue[i] > sensorValueMax[i] ? sensorValue[i] : sensorValueMax[i];
    sensorValueAvg[i] = (sensorValueMin[i] + sensorValueMax[i]) / 2; // update the avarage value
    threshold[i] = (sensorValueMax[i] - sensorValueMax[i]) * 0.1; // Threshold is 10% of total sensor range
  }
}
