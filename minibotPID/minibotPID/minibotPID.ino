#include <EnableInterrupt.h>
//A is left, b is right
//all distances are in mm., all velocities are in mm./s
int pinDirectionA = 12;
int pinPWMA = 3;
int pinDirectionB = 13;
int pinPWMB = 11;
int encoderPinA = A5;
int encoderPinB = A4;
int delayTime = 5000;
int potentiometerPin = A0;
long newtime;
long oldtime = 0;
int LEDPin = 10;
double errorA = 0;
double errorB = 0;
int desiredVelA = 10;
int desiredVelB = 10;
int desiredVel = 10;
double wheelBase = 140;
double wheelRadius = 31;
double turnRate = 0;
double motorA = 0;
double motorB = 0;
double kP = 5;
long time = 0;
double EPS = 5.0;
double maxVel = 600.0; // max vel in mm/sec

volatile int32_t  countA = 0;
volatile int32_t  countB = 0;
volatile long nowA = 0;
volatile long nowB = 0;
volatile long nextTimeA = 0;
volatile long nextTimeB = 0;

int deltaTimeA = 100;
int deltaTimeB = 100;
volatile int lastCountA = 0;
volatile int lastCountB = 0;
volatile double actualVelA = 0;
volatile double actualVelB = 0;

void setup ()
{
  Serial.begin(9600);
  pinMode(pinDirectionA, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinDirectionB, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(potentiometerPin, INPUT);
  enableInterrupt(encoderPinA, encoderA, CHANGE);
  enableInterrupt(encoderPinB, encoderB, CHANGE);
}



void loop () {
  // value between 0..1023, map onto 0..maxVel in mm/sec
  int potValue = analogRead(potentiometerPin);
  desiredVel = potValue * maxVel / 1023;
  
  // add analog read for desired velocity range 0..1023, scale to reasonable double rotaion rate  
  int currentCountA = countA;
  int currentCountB = countB;

  // in wheel shaft rotation speed rad/sec
  double aVelA = actualVelA;
  double aVelB = actualVelB;

  // in wheel shaft rotation speed rad/sec
  double va = calculateVelA(desiredVel, turnRate);
  double vb = calculateVelB(desiredVel, turnRate);

  double errorA = va - aVelA;
  double errorB = vb - aVelB;
  


  // map onto 0 to 255, see constrain function
  motorA = motorA - kP * errorA;
  motorA = constrain(motorA, 0, 255);
  motorB = motorB - kP * errorB;
  motorB = constrain(motorB, 0, 255);
  analogWrite(pinPWMA, motorA);
  analogWrite(pinPWMB, motorB);
  digitalWrite(pinDirectionA, HIGH);
  digitalWrite(pinDirectionB, HIGH);

  long currentTime = millis();
  if (time < currentTime) {
    time = currentTime + 1000;
    // output more data?  which parameters are we interested in?
    Serial.print("v=");Serial.print(desiredVel);
    Serial.print(", a=");Serial.print(currentCountA);
    Serial.print(", lca=");Serial.print(lastCountA);
    Serial.print(", b=");Serial.print(currentCountB);
    Serial.print(", va=");Serial.print(va);
    Serial.print(", ava=");Serial.print(aVelA);
    Serial.print(", avb=");Serial.print(aVelB);
    Serial.print(", na=");Serial.print(nowA);
    Serial.print(", nta=");Serial.print(nextTimeA);
    Serial.print(", ma=");Serial.print(motorA);
    Serial.print(", mb=");Serial.print(motorB);
    Serial.print(", ea=");Serial.print(errorA);
    Serial.print(", eb=");Serial.print(errorB);
    Serial.println("");
  }
  
  if(abs(kP * errorA) < EPS && abs(kP * errorB) < EPS){
      digitalWrite(LEDPin, HIGH);
  } else {
      digitalWrite(LEDPin, LOW);
  }
}

double calculateVelA (double dVel, double tRate){
  return (2 * dVel + wheelBase * tRate) / (2 * wheelRadius);
}

double calculateVelB (double dVel, double tRate){
  return (2 * dVel - wheelBase * tRate) / (2 * wheelRadius);
}


void encoderA(){
  countA++;
  nowA = millis();
  if(nextTimeA - nowA <= 0) {
    // compute delta ticks for A
    actualVelA = (countA - lastCountA) * 1000 / (deltaTimeA);
    nextTimeA = nowA + deltaTimeA;
    lastCountA = countA;
  }
}

void encoderB(){
  countB++; 
  nowB = millis();
  if(nextTimeB - nowB <= 0) {
    // compute delta ticks for B
    actualVelB = (countB - lastCountB) * 1000 / (deltaTimeB);
    nextTimeB = nowB + deltaTimeB;
    lastCountB = countB;
  }
}
