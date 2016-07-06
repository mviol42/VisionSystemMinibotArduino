#include <EnableInterrupt.h>

int pinDirectionA = 12;
int pinPWMA = 3;
int pinDirectionB = 13;
int pinPWMB = 11;
int encoderPinA = A5;
int encoderPinB = A4;
int delayTime = 5000;
volatile uint32_t   countA = 0;
volatile uint32_t  countB = 0;

unsigned long time = 0;

void setup ()
{
  Serial.begin(9600);
  pinMode(pinDirectionA, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinDirectionB, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  enableInterrupt(encoderPinA, encoderA, CHANGE);
  enableInterrupt(encoderPinB, encoderB, CHANGE);  
}

void loop () {
  int currentCountA = countA;
  int currentCountB = countB;
  unsigned long currentTime = millis();
  if (time < currentTime) {
    time = currentTime + 1000;
    Serial.print("a=");Serial.print(currentCountA);Serial.print(", b=");Serial.println(currentCountB);
  }
  digitalWrite(pinDirectionA, LOW);
  digitalWrite(pinDirectionB, HIGH);
  if(currentCountA > 20000){
    analogWrite(pinPWMA, 0);
    analogWrite(pinPWMB, 0);
  }
  else {
    analogWrite(pinPWMA, 127);
    analogWrite(pinPWMB, 127);
  }
}

void encoderA(){
  countA++;
}

void encoderB(){
  countB++;
}
