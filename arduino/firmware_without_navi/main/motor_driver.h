#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 6
  #define LEFT_MOTOR_BACKWARD  10
  #define RIGHT_MOTOR_FORWARD  5
  #define LEFT_MOTOR_FORWARD   9
  //#define RIGHT_MOTOR_ENABLE 12
  //#define LEFT_MOTOR_ENABLE 13
#endif

void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
