/*
 * TB6600 Driver with Bipolar Motor
 * Control stepper motor by serial port communication
 * Receive command from GUI model
 * Command includes direction and number of steps
 * Author: Yixiang Fan
 * Data: 29-01-2019
 * Version: 2
*/
int enablepin = 3;
int dirpin = 4;
int pulsepin = 5;
int clockwise = 0;
int counterclockwise = 1;
char message[7];
int command = 0;
int steps = 0;

void setup() {
   memset(message, '\0', 7); 
   Serial.begin(9600); // send and receive at 9600 baud
   pinMode(enablepin,OUTPUT);
   pinMode(dirpin,OUTPUT);
   pinMode(pulsepin,OUTPUT);
   //rotation, delay,steps


  /* 20000+5000+200+100=25300 steps nearly equals to 180 degree
  *  25300/180*170 = 23894 steps nearly equals to 170 degree
  *  Half a round, from right up to left 170 degree and return to right up, takes 50 seconds.
  *  So a whole round, from right up to left 170 degree, to right up, to right 170 degree
  *  and then return to right up, takes 100 seconds.
  */
 
}
void loop(){
  if(Serial.available()){
    Serial.readBytes(message, 6);
    command = message[0] - 48;
    steps = atoi(message+1);
//    Serial.print(23456, DEC);
    if(steps != 0){
      if(command == 1){  // 1 means clockwise
        spin(clockwise,0,steps);  
      }else if(command == 2){  // 2 means counterclockwise
        spin(counterclockwise,0,steps);
      }else{
        Serial.print(0, DEC);  
      }
      Serial.print(1, DEC);
    }else{
      Serial.print(0, DEC);
    }
    memset(message, '\0', 7); 
  }
}
void spin (int direction, int dly,int steps){
   digitalWrite(enablepin,HIGH); 
   digitalWrite(dirpin,direction);
   for (int step=1;step<steps+1;step++){
      digitalWrite(pulsepin,LOW);
      delay(1);
      digitalWrite(pulsepin,HIGH);
      delay(dly);
   }
//digitalWrite(enablepin,LOW);
}

