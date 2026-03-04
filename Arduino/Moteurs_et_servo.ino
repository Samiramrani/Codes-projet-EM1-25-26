#include <Servo.h>

Servo myservo; // create servo object to control a servo
// twelve servo objects can be created on most boards

int PWM1 = 10;
int DIR1 = 11;
int PWM2 = 9;
int DIR2 = 8;
int pos = 0; // variable to store the servo position

int goForward(int d){
digitalWrite(DIR1,HIGH);
analogWrite(PWM1,100);
digitalWrite(DIR2,HIGH);
analogWrite(PWM2,100);
delay(d*1000/26.5);
}
int pivotLeft(){
digitalWrite(DIR1,LOW);
analogWrite(PWM1,0);
digitalWrite(DIR2,HIGH);
analogWrite(PWM2,100);
delay(1000);
}
int pivotRight(){
digitalWrite(DIR1,HIGH);
analogWrite(PWM1,100);
digitalWrite(DIR2,LOW);
analogWrite(PWM2,0);
delay(1000);
}
int goBackward(int d){
digitalWrite(DIR1,LOW);
analogWrite(PWM1,100);
digitalWrite(DIR2,LOW);
analogWrite(PWM2,100);
delay(d*1000/26.5);
}
int goRight(int a){
digitalWrite(DIR1,LOW);
analogWrite(PWM1,100);
digitalWrite(DIR2,HIGH);
analogWrite(PWM2,100);
delay((48,7*500/26,5)*a/360);
}
int goLeft(int a){
digitalWrite(DIR1,LOW);
analogWrite(PWM1,100);
digitalWrite(DIR2,HIGH);
analogWrite(PWM2,100);
delay((48,7*500/26,5)*a/360);
}

int bicDown(){
for (pos > 100; pos < 200; pos+=1){
myservo.write(pos);
}
}

int bicUp(){
for (pos = 0; pos <= 100; pos += 1) { // goes from 0 degrees to 180 degrees
// in steps of 1 degree
myservo.write(pos); // tell servo to go to position in variable 'pos'
delay(30);
} // waits 15ms for the servo to reach the position
for (pos > 100; pos < 200; pos+=1){
myservo.write(pos);
}
}

void setup()
{
pinMode(DIR1, OUTPUT);
pinMode(DIR2, OUTPUT);
myservo.attach(3);
}
void loop()
{
delay(5000);
//bicDown();
goForward(20);
//bicUp();
goRight(90);
goForward(10);
goRight(90);
goForward(15);
goRight(90);
goForward(5);
//bicDown();
goForward(20);
}
