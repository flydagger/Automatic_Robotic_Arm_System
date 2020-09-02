/*
 * communicate with GUI module
 * control motor
 */
#include <Servo.h>

Servo myservo;

int pos = 0;

void setup()
{
  Serial.begin(9600); // send and receive at 9600 baud
}

//bool flag = false;

void loop()
{
//  if(flag == false){
//    flag = true;
//    Serial.print("0");
//  }else{
//    flag = false;
//    Serial.print("1");
//  }
//  if (Serial.available()) {
//    int inByte = Serial.read();
//    Serial.print(inByte-48, DEC);
//  }
  if(Serial.available()){
    int command = Serial.read() - 48;
    if(command == 1){  // 1 means grasp component
      myservo.attach(9);
      for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
      myservo.detach();
      if(myservo.read() == 180){  // check whether servo arrives position 180
        Serial.print(1, DEC);  // 1 means succeed to grasp
      }else{
        Serial.print(0, DEC);  // 0 means fail to grasp
      }
    }else if(command == 2){  // 2 means release component
      myservo.attach(9);
      for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
      myservo.detach();
      if(myservo.read() == 0){ // check whether servo arrives position 0
        Serial.print(1, DEC);  // 1 means succeed to release
      }else{
        Serial.print(0, DEC);  // 0 means fail to release
      }
    }else if(command == 0){
      myservo.detach();
      Serial.print(1, DEC);
    }
  }
}
