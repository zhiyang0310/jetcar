#include "motor_driver.h"

#define RIGHT_MOTOR_BACKWARD 10
#define LEFT_MOTOR_BACKWARD  9
#define RIGHT_MOTOR_FORWARD  11
#define LEFT_MOTOR_FORWARD   8

void setMotor(int i, int spd) {
    
    if (i == RIGHT) { 
      if (RIGHT_DIRECTION) { 
        analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0);
        }
      else{ 
        analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0);
        }
    }
    else {
      if(LEFT_DIRECTION) { 
        analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); 
        }
      else{ 
        analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0);
        }
    }
}
