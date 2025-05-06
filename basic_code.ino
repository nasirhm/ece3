uint16_t sensorValues[8]; 

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

bool print_directions = tue;
const int LED_RF = 41;
const int num_samples = 5;

///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();
// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  int leftSpd = 70;
  int rightSpd = 70;
  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(righ_pwm_pin, rightSpd);
}

int getError()
{  
  int mins[8] = {667, 575, 553, 691, 553, 599, 621, 714}; // minimum sensor reading values for each of 8 sensors
  int maxs[8] = {1833, 1925, 1947, 1807, 1947, 1901, 1879, 1786}; // max values
  int err[8] = {};

  for(int i = 0; i < 8; i++){
    err[i] = maxs[i] - mins[i];      
  }

  
  return error;
}

void loop() {
  // put your main code here, to run repeatedly: 
  int leftSpd = 70;
  int rightSpd = 70;
  
  ECE3_read_IR(sensorValues);
  analogWrite(left_pwm_pin,leftSpd);    
}
