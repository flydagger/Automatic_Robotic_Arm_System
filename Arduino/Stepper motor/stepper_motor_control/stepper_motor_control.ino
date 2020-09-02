/*
TB6600 Driver with Bipolar Motor
How to Control Stepper Motors
Written by Michael Wright, Y2kLeader.com 
*/
int enablepin=3;
int dirpin=4;
int pulsepin=5;
int clockwise=0;
int counterclockwise=1;
void setup() {
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
//  int var = 0;
//  while(var++ < 200){
//    spin(counterclockwise,0,23894);
//    delay(1000);
//    spin(clockwise,0,23894);
//    delay(1000);
//    spin(clockwise,0,23894);
//    delay(1000);
//    spin(counterclockwise,0,23894);
//    delay(1000);
//  }
   
}
void loop(){
  if(Serial.available()){
    int command = Serial.read() - 48;
    if(command == 1){  // 1 means clockwise
      spin(clockwise,0,13000);
      Serial.print(1, DEC);
    }else if(command == 2){  // 2 means counterclockwise
      spin(counterclockwise,0,13000);
      Serial.print(1, DEC);
    }
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