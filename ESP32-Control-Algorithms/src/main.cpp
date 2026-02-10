#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

#define CLK_PIN 25 // ESP32 pin GPIO25 connected to the rotary encoder's CLK pin
#define DT_PIN  26 // ESP32 pin GPIO26 connected to the rotary encoder's DT pin

const int ENCODER_CPR = 12; // counts per revolution (CPR) of the rotary encoder

const int SETPOINT_ANGLE = 90; // desired angle to reach

int counter = 0;
float angle = 0;
int CLK_state;
int prev_CLK_state;

int motor1Pin1 = 27; 
int motor1Pin2 = 28; 
int enable1Pin = 14; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

void setup() {
  Serial.begin(9600);

  // configure encoder pins as inputs
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  // configure LEDC PWM
  ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);
  // read the initial state of the rotary encoder's CLK pin
  prev_CLK_state = digitalRead(CLK_PIN);}

void loop() {
  CLK_state = digitalRead(CLK_PIN);
  if (CLK_state != prev_CLK_state && CLK_state == HIGH) {
    if (digitalRead(DT_PIN) == HIGH) {
      counter--;
    } else {
      counter++;
    }
  }
  float angle = getAngle();
  prev_CLK_state = CLK_state;

  Serial.print("Counter: ");
  Serial.println(counter);
  Serial.print("Angle: ");
  Serial.println(angle);

  int dutyCycle = returnDutyCycle(angle);
  Serial.print("Duty Cycle: ");
  Serial.println(dutyCycle);

  if(dutyCycle > 0){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(enable1Pin, dutyCycle); 
  }else if(dutyCycle < 0){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    ledcWrite(enable1Pin, -dutyCycle);
  }
  



}


// put function definitions here:
float getAngle() {
  return (float)(counter % ENCODER_CPR) * (360.0 / (float)ENCODER_CPR);
}

int returnDutyCycle(float angle){
  float error = SETPOINT_ANGLE - angle;
  float Kp = 1.0; // proportional gain
  if (error > 180) {
    error = error - 360; // adjust error for angles greater than 180 degrees
  } else if (error < -180) {
    error = error + 360; // adjust error for angles less than -180 degrees
  }

  int dutyCycle = (int)(Kp * error);
  dutyCycle = constrain(dutyCycle, -255, 255); // limit duty cycle
  return dutyCycle;
}