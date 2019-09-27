#ifdef ARDUINO_ENC_COUNTER
  #define LEFT_ENC_PIN 2  //pin 2
  #define RIGHT_ENC_PIN 3  //pin 3
  long LEFT_TICKS = 0L;
  long RIGHT_TICKS = 0L;
#endif

void leftCounter();
void rightCounter();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
