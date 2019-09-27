
void leftCounter_A(){
  if(digitalRead(LEFT_ENC_PIN_A) == digitalRead(LEFT_ENC_PIN_B))
      LEFT_TICKS += 1;
  else
      LEFT_TICKS -= 1;
  //Serial.print(digitalRead(LEFT_ENC_PIN_A));
  //Serial.print(digitalRead(LEFT_ENC_PIN_B));
  //Serial.print("-----");
}

void leftCounter_B(){
  if(digitalRead(LEFT_ENC_PIN_A) == digitalRead(LEFT_ENC_PIN_B))
      LEFT_TICKS -= 1;
  else
      LEFT_TICKS += 1;
  //Serial.print(digitalRead(LEFT_ENC_PIN_A));
  //Serial.print(digitalRead(LEFT_ENC_PIN_B));
  //Serial.print("-----");
}

void rightCounter_A(){
  if(digitalRead(RIGHT_ENC_PIN_A) == digitalRead(RIGHT_ENC_PIN_B))
      RIGHT_TICKS -= 1;
  else
      RIGHT_TICKS += 1;
  //Serial.print(digitalRead(RIGHT_ENC_PIN_A));
  //Serial.print(digitalRead(RIGHT_ENC_PIN_B));
  //Serial.print("-----");
}

void rightCounter_B(){
  if(digitalRead(RIGHT_ENC_PIN_A) == digitalRead(RIGHT_ENC_PIN_B))
      RIGHT_TICKS += 1;
  else
      RIGHT_TICKS -= 1;
  //Serial.print(digitalRead(RIGHT_ENC_PIN_A));
  //Serial.print(digitalRead(RIGHT_ENC_PIN_B));
  //Serial.print("-----");
}

long readEncoder(int i){
  if(i==LEFT){
    return LEFT_TICKS;
  }
  else{
    return RIGHT_TICKS;
  }
}

void resetEncoder(int i){
  if(i==LEFT){
    LEFT_TICKS = 0L;
  }
  else{
    RIGHT_TICKS = 0L;
  }
}

void resetEncoders(){
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
