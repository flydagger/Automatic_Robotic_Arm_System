/*
 * communicate with GUI module
 * control motor
 */
#include <Servo.h>

Servo myservo;

int pos = 0;
char message[7];
char start_degree_char[4];
char end_degree_char[4];
int start_degree = 0;
int end_degree = 0;

void setup()
{
  Serial.begin(9600); // send and receive at 9600 baud
  memset(message, '\0', 7);
  memset(start_degree_char, '\0', 4);
  memset(end_degree_char, '\0', 4);
}

//bool flag = false;

void loop(){
  if(Serial.available()){
    Serial.readBytes(message, 6);
    strncpy(start_degree_char, message, 3);
    strncpy(end_degree_char, message+3, 3);
    start_degree = atoi(start_degree_char);
    end_degree = atoi(end_degree_char);
    if(start_degree < 0 || start_degree > 180 || end_degree < 0 || end_degree > 180 || start_degree == end_degree){
      Serial.print(0, DEC);
    }else{
      myservo.attach(9);
      if(start_degree < end_degree){
        for (pos = start_degree; pos <= end_degree; pos += 1) {
          myservo.write(pos);
          delay(15);
        }      
      }else if(start_degree > end_degree){
        for (pos = start_degree; pos >= end_degree; pos -= 1) {
          myservo.write(pos);
          delay(15);
        }      
      if(myservo.read() == end_degree){
        Serial.print(1, DEC);
      }else{
        Serial.print(0, DEC);
      }
      myservo.detach();
      }
    memset(message, '\0', 7);
    memset(start_degree_char, '\0', 4);
    memset(end_degree_char, '\0', 4);
    }
  }
}

