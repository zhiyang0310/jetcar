#include "commands.h"
#include "pin_table.h"
#include "encoder_driver.h"
#include "motor_driver.h"

//*********************Parameters*********************//

//Serial port baud rate
#define BAUDRATE 9600
//The PID loop rate
#define PID_RATE 30 // Hz
//The MAX PWM
#define MAX_PWM 255
/* Stop the robot if it hasn't received a movement command
 in this number of milliseconds */
#define AUTO_STOP_INTERVAL 1000

/////////////////////////////////////////////////////////

//*********************Variables*********************//

////////Variables to receive commands///////////
//Variable to hold an input character
char chr;
//A pair of varibles to parse serial commands
int arg = 0;
int index = 0;
// Variable to hold the current single-character command
char cmd;
// Character arrays to hold the first and second arguments
char argv1[20];
char argv2[20];
// The arguments converted to integers
int arg1;
int arg2;
//record wheels ticks
long LEFT_TICKS = 0L;
long RIGHT_TICKS = 0L;

/////////////////////////////////////////////////////////

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
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      run_cmd();
      reset_cmd();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
}
