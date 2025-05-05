#include <ECE3.h>

#define left_nslp_pin 31 
#define left_dir_pin 29
#define left_pwm_pin 40

#define right_nslp_pin 11 
#define right_dir_pin 30
#define right_pwm_pin 39

uint16_t sensorValues[8];

int base = 140;

float leftSpd = base;
float rightSpd = base;
int turndelay = 240;


float previous_error = 0;
float integral = 0;

bool lastTurnAround = false;
bool hasTurned = false;

void setup()
{
  ECE3_Init();

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);

  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);

}

void loop(){
    // write to sensorValues
    ECE3_read_IR(sensorValues);
}

int combinedValues()
{
  float newVal = 0;
  float arr[8]  = { -1.875, -1.75, -1.5, -1, 1, 1.5, 1.75, 1.875}; // update it according to the readings.
  for (int i = 0; i < 8; i++)
  {
    newVal += arr[i] * sensorValues[i];
  }
  return newVal;
}
