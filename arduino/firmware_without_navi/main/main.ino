#include "encoder_driver.h"
#include "motor_driver.h"

//PINs table
#define LEFT 0
#define RIGHT 1
#define LEFT_ENC_PIN_A 2  //pin 2
#define LEFT_ENC_PIN_B 20  //pin 20
#define RIGHT_ENC_PIN_A 3  //pin 3
#define RIGHT_ENC_PIN_B 21  //pin 21
#define RIGHT_MOTOR_BACKWARD 10
#define LEFT_MOTOR_BACKWARD  9
#define RIGHT_MOTOR_FORWARD  11
#define LEFT_MOTOR_FORWARD   8

/* Serial port baud rate */
#define BAUDRATE     9600

/* define the string to reply request */
char reply[200];

/* PWM signal */
unsigned int PWM = 255;

long LEFT_TICKS = 0L;
long RIGHT_TICKS = 0L;

bool LEFT_DIRECTION = true;
bool RIGHT_DIRECTION = true;

// Variable to hold an input character
char chr;

//setup
void setup() {

  //set pins for interruption
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A),leftCounter_A,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A),rightCounter_A,CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B),leftCounter_B,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B),rightCounter_B,CHANGE);
  //set motor pins
  pinMode(RIGHT_MOTOR_BACKWARD,OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD,OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD,OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD,OUTPUT);
  //start Serial port
  Serial.begin(BAUDRATE);
}

void loop() {
  /* receive command from jetson nano */
  while (Serial.available() > 0) {
    
    // Read the character in port
    chr = Serial.read();

    // resolve received command
    if (chr == 'r'){
      sprintf(reply,"%ld|%ld$",LEFT_TICKS,RIGHT_TICKS);
      Serial.print(reply);
      continue;
    }
    if (chr == 'A'){
      setMotor(LEFT, PWM);
      Serial.print('$');
      continue;
    }
    else if (chr == 'D'){
      setMotor(RIGHT, PWM);
      Serial.print('$');
      continue;
    }
    else if (chr == 'a'){
      setMotor(LEFT, 0);
      Serial.print('$');
      continue;
    }
    else if (chr == 'd'){
      setMotor(RIGHT, 0);
      Serial.print('$');
      continue;
    }
    else if(chr == 'Q') {
      LEFT_DIRECTION = !LEFT_DIRECTION;
      continue;
    }
    else if (chr == 'E') {
      RIGHT_DIRECTION = !RIGHT_DIRECTION;
      continue;
    }
    else if (chr == 'u') {
      if(PWM < 250){
          PWM += 5;
      }
      else{
          PWM = 255;
      }
      continue;
    }
    else if (chr == 'n') {
      if(PWM > 5){
          PWM -= 5;
      }
      else{
          PWM = 0;
      }
      continue;
    }
  }
}
