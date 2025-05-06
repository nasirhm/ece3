// Car control file v1 - proportional steering adjustment 
// 11:33 am, 5/6/2025, Zhongwen Zhang 

#include "ECE3.h"

uint16_t sensorValues[8]; 

const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

bool print_directions = true;
const int LED_RF = 41;
const int num_samples = 5;

///////////////////////////////////
void setup() {
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);

  pinMode(LED_RF, OUTPUT);

  ECE3_Init();

  Serial.begin(9600); 
  delay(2000);

  int leftSpd = 70;
  int rightSpd = 70;
  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);
}

/////////////////////////////////////////////
// Weighted error calculation
////////////////////////////////////////////

int weightScheme_8421(const uint16_t n[8]){
  return (-8*n[0] - 4*n[1] - 2*n[2] - n[3] + n[4] + 2*n[5] + 4*n[6] + 8*n[7]) / 4;  
}

int weightScheme_5_14_12_8(const uint16_t n[8]){
  return (-5*n[0] - 14*n[1] - 12*n[2] - 8*n[3] + 8*n[4] + 12*n[5] + 14*n[6] + 5*n[7]) / 8;
}

int getError() {  
  int mins[8] = {667, 575, 553, 691, 553, 599, 621, 714};
  int maxs[8] = {1833, 1925, 1947, 1807, 1947, 1901, 1879, 1786};

  ECE3_read_IR(sensorValues);

  for (int i = 0; i < 8; i++) {
    sensorValues[i] -= mins[i]; // zero the data 
    sensorValues[i] *= (1000 / maxs[i]); // normalize the data to have max of 1000
  }

  return weightScheme_8421(sensorValues);
}

/////////////////////////////////////////////
// Symmetric proportional control
////////////////////////////////////////////

void adjustWheelSpeed(int error) {
  const int baseSpeed = 50;
  const float k = 0.0008; // tuning coefficient, controls sensitivity of response

  // positive error - veered right            ///   negative error - veered left
    // increase rightSpd, decrease leftSpd    ///     decrease rightSpd, increase leftSpd
  int rightSpd = baseSpeed + k * error;
  int leftSpd = baseSpeed - k * error; 

  leftSpd = constrain(leftSpd, 0, 255);
  rightSpd = constrain(rightSpd, 0, 255);

  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);
}

/////////////////////////////////////////////
// Main loop
////////////////////////////////////////////

void loop() {
  int error = getError(); // get the sensor readings and convert them into an error term 
  adjustWheelSpeed(error); // adjust the wheel speed in response to the error term
}
