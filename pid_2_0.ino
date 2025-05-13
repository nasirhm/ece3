#include "ECE3.h"

uint16_t sensorValues[8];

const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

//bool print_directions = true;
//const int LED_RF = 41;
//const int num_samples = 5;

// PID Constants - Needs tuning

// Start with Ziegler-Nichols

float Kp = 0.8;
float Kd = 0.1;

int previous_error = 0; // state storage for the last loop iteration for D-Term
const int BASE_SPEED = 70;
const int MAX_MOTOR_PWM = 255;
const int MIN_MOTOR_PWM = 0;

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

  analogWrite(left_pwm_pin, BASE_SPEED);
  analogWrite(right_pwm_pin, BASE_SPEED);
}

/////////////////////////////////////////////
// Weighted error calculation
////////////////////////////////////////////

int weightScheme_8421(const uint16_t n[8]){
  return (-8*n[0] - 4*n[1] - 2*n[2] - n[3] + n[4] + 2*n[5] + 4*n[6] + 8*n[7]) / 4;
}

int getError() {
  int mins[8] = {667, 575, 553, 691, 553, 599, 621, 714};
  int maxs[8] = {1833, 1925, 1947, 1807, 1947, 1901, 1879, 1786};

  ECE3_read_IR(sensorValues);


 // for (int i = 0; i < 8; i++) {
 //   sensorValues[i] -= mins[i]; // zero the data, potential < 0, set to 0
 //   sensorValues[i] *= 1000; // normalize the data to have max of 1000
 //   sensorValues[i] /= maxs[i];
 // }

  // Potential Normalization
  for (int i = 0; i < 8; i++){
    long current_reading = sensorValues[i]; // temporary long
    current_reading -= mins[i];
    if(current_reading < 0){ current_reading = 0; }

    long range = maxs[i] - mins[i];
    if(range <=0){
      range = 1;
    }

    current_reading = (current_reading * 1000) / range;

    if(current_reading > 1000){
      current_reading = 1000;
    }

    sensorValues[i] = (uint16_t)current_reading;

  }

  return weightScheme_8421(sensorValues);
}

void adjustWheelSpeed(float correction){

  // if err > 0 (robot is to the right of the line), then turn left, by increasing left motor speed and decreasing right motor speed.
  // if err < 0 (robot is to the left of the line), turn turn right, by inc right motor spd and then dec. left motor sped.
  int leftSpd_adj;
  int rightSpd_adj;

  leftSpd_adj = BASE_SPEED + (int) correction_signal;
  rightSpd_adj = BASE_SPEED - (int) correction_signal;

  leftSpd_adj = constrain(leftSpd_adj, MIN_MOTOR_PWM, MAX_MOTOR_PWM);
  rightSpd_adj = constrain(rightSpd_adj, MIN_MOTOR_PWM, MAX_MOTOR_PWM);

  analogWrite(left_pwm_pin, leftSpd_adj);
  analogWrite(right_pwm_pin, rightSpd_adj);

}

/////////////////////////////////////////////
// Main loop
////////////////////////////////////////////

void loop() {
  int current_error = getError(); // get the sensor readings and convert them into an error term
  int derivative_term = current_error - previous_error; // will be executed every clock cycles, so / 1 would be fine
  float correction = (Kp * current_error) + (Kd * derivative_term);
  previous_error = current_error;
  adjustWheelSpeed(correction); // adjust the wheel speed in response to the error term
  delay(1); // allows for better Kd tuning, can be removed for fast.
}
