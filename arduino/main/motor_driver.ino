#include "pin_table.h"
#include "motor_driver.h"

#define LEFT            0
#define RIGHT           1

void setMotorSpeed(int i, int spd) {
    
    if (i == RIGHT) { 
      if (spd>=0) { 
        analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0);
        }
      else{ 
        analogWrite(RIGHT_MOTOR_BACKWARD, -spd); analogWrite(RIGHT_MOTOR_FORWARD, 0);
        }
    }
    else {
      if(spd>=0) { 
        analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); 
        }
      else{ 
        analogWrite(LEFT_MOTOR_BACKWARD, -spd); analogWrite(LEFT_MOTOR_FORWARD, 0);
        }
    }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed){
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
