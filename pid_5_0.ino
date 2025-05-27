#include "ECE3.h"

String dummy;
uint16_t sensorValues[8];

const int left_nslp_pin = 31; // sleep pin 
const int left_dir_pin = 29; // direction pin 
const int left_pwm_pin = 40; // speed control pin 

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

bool comingBack = false; 
int donut = 0; // no sensors indicate turn around 
const int YES = 8; 
bool carIsMoving = true; 

//bool print_directions = true;
//const int LED_RF = 41;
//const int num_samples = 5;

// PID Constants - Needs tuning

// Start with Ziegler-Nichols

float Kp = 0.05;
float Kd = 0.3;

int previous_error = 0; // state storage for the last loop iteration for D-Term
const int BASE_SPEED = 40;
const int MAX_MOTOR_PWM = 255;
const int MIN_MOTOR_PWM = 0;

///////////////////////////////////
void setup() {
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH); // LOW to sleep, motor won't run

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH); // LOW to sleep, motor won't run 

//  pinMode(LED_RF, OUTPUT);

  ECE3_Init();

  Serial.begin(9600);
  delay(2000);

  analogWrite(left_pwm_pin, BASE_SPEED);
  analogWrite(right_pwm_pin, BASE_SPEED);
}

/////////////////////////////////////////////
// Weighted error calculation
////////////////////////////////////////////

int dynamicWeightScheme(const uint16_t n[8]){ // on the way there, prefers left
    
    if(comingBack) return rightBiased(n);  // on the way back, prefers right

    return leftBiased(n);  
}

int symmetric_8421(const uint16_t n[8]){
    return (-8*n[0] - 4*n[1] - 2*n[2] - n[3] + n[4] + 2*n[5] + 4*n[6] + 8*n[7]) / 4;  
}

int rightBiased(const uint16_t n[8]) {
  return (-8*n[0] - 4*n[1] - 2*n[2] - 1*n[3] + 1*n[4] + 1*n[5] + 2*n[6] + 8*n[7]) / 4;
}

int leftBiased(const uint16_t n[8]) {
  return (-8*n[0] - 2*n[1] - 1*n[2] - 1*n[3] + 1*n[4] + 2*n[5] + 4*n[6] + 8*n[7]) / 4;
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
   donut = 0;

  for (int i = 0; i < 8; i++){
    long current_reading = sensorValues[i]; // temporary long
    if(current_reading >= 2200) donut++;
    
    current_reading -= mins[i];
    if(current_reading < 0){ current_reading = 0; }

    long range = maxs[i] - mins[i]; // is this line necessary while car is driving? We know the range before start up. 
    if(range <=0){
      range = 1;
    }

    current_reading = (current_reading * 1000) / range;

    if(current_reading > 1000){
      current_reading = 1000;
    }

    sensorValues[i] = (uint16_t)current_reading;
  }

  return dynamicWeightScheme(sensorValues);
}

void turnAround(int delayMs){
  stopCar();
  
  delay(1000);
  
  digitalWrite(right_dir_pin, LOW); 
  digitalWrite(left_dir_pin, HIGH);

  analogWrite(left_pwm_pin, 60);
  analogWrite(right_pwm_pin, 60);

  delay(delayMs);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, HIGH);

   stopCar();
  
  delay(1000);

  comingBack = false;
  donut = 0;
}

void stopCar(){
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}

void adjustWheelSpeed(float correction){
  // if err > 0 (robot is to the right of the line), then turn left, by increasing right motor speed and decreasing left motor speed.
  // if err < 0 (robot is to the left of the line), turn turn right, by inc left motor spd and then dec. right motor sped.
  int leftSpd_adj;
  int rightSpd_adj;

    digitalWrite(left_dir_pin, LOW);
    digitalWrite(right_dir_pin, LOW);
  

  leftSpd_adj = BASE_SPEED - (int) correction;
  if (leftSpd_adj < 0){ // only flip if we need to
    leftSpd_adj = abs(leftSpd_adj);
    digitalWrite(left_dir_pin, HIGH);
    
    }
    
  rightSpd_adj = BASE_SPEED + (int) correction;

if (rightSpd_adj < 0){
    rightSpd_adj = abs(rightSpd_adj);
    digitalWrite(right_dir_pin, HIGH);
    }
    
  leftSpd_adj = constrain(leftSpd_adj, MIN_MOTOR_PWM, MAX_MOTOR_PWM);
  rightSpd_adj = constrain(rightSpd_adj, MIN_MOTOR_PWM, MAX_MOTOR_PWM);

  analogWrite(left_pwm_pin, leftSpd_adj);
  analogWrite(right_pwm_pin, rightSpd_adj);

}

void stopCarPermanently(){
  stopCar();
  delay(100000);
  }

/////////////////////////////////////////////
// Main loop 
////////////////////////////////////////////

void loop() {
  
    donut = 0;
    int current_error = getError(); // get the sensor readings and convert them into an error term
    int derivative_term = current_error - previous_error; // will be executed every clock cycles, so / 1 would be fine
    float correction = (Kp * current_error) + (Kd * derivative_term);
    previous_error = current_error;
    adjustWheelSpeed(correction); // adjust the wheel speed in response to the error term
    
    if(!comingBack && donut == YES){ // this loop will run once when we hit the end to turn around
      turnAround(1000);
      comingBack = true; 
    }
    delay(1); // allows for better Kd tuning, can be removed for fast.

    if(comingBack && donut == YES){ // the second time the full black line is detected, we've finished the track
      carIsMoving = false; 
      stopCarPermanently();  
           
    } 
}
