#include "commands.h"
#include "motor_driver.h"

#define LEFT            0
#define RIGHT           1

//define self-driving mode
bool self_driving = false;

void reset_cmd() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

void run_cmd(){
    if(cmd == SELF_DRIVING){self_driving = true;Serial.print("ok$");}
    else if(cmd == MANUAL_DRIVING){self_driving = false;Serial.print("ok$");}
    else if(cmd == READ_ENCODERS){Serial.print(LEFT_TICKS);Serial.print('|');Serial.print(RIGHT_TICKS);Serial.print("$");}
    else if(cmd == MOTOR_SPEED_L){arg1 = atol(argv1);setMotorSpeed(LEFT, arg1);Serial.print("ok$");}
    else if(cmd == MOTOR_SPEED_R){arg1 = atol(argv1);setMotorSpeed(RIGHT, arg1);Serial.print("ok$");}
    else if(cmd == MOTOR_SPEEDS){arg1 = atoi(argv1);arg2 = atoi(argv2);setMotorSpeeds(arg1, arg2);Serial.print("ok$");}
    else if(cmd == PING){Serial.print("hello");}
    else if(cmd == RESET_ENCODERS){LEFT_TICKS = 0L;RIGHT_TICKS = 0L;Serial.print("ok$");}
    else if(cmd == UPDATE_PID_L){}
}
