
#define NEEDFORSPEED
#define INTERRUPT_FLAG_PINA5_S32 countA
#define INTERRUPT_FLAG_PINA4_S32 countB

#include <EnableInterrupt.h>
//A is left, b is right
//all distances are in mm., all velocities are in mm./s

//pin numbers
int pinDirectionA = 12;
int pinPWMA = 3;
int pinDirectionB = 13;
int pinPWMB = 11;
int encoderPinA = A5;
int encoderPinB = A4;
int LEDPin = 10;
int minibotVelPin = A0;

//Time
int delayTime = 500;
long newtime;
long oldtime = 0;
long printTime = 0;

//Variables for calculations
double turnRate = 0;
double aSA = 0;
double aSB = 0;
double pidaSA = 0;
double pidaSB = 0;
double minibotMaxVelMm = 600.0; // max vel in mm/sec
int desiredAxleSpeedA = 10;
int desiredAxleSpeedB = 10;
int desiredMinibotVel = 10;
double errorA = 0;
double errorB = 0;
int convertMStoS = 1000;

double EPS = 5.0;
double kP = 2.5;

//physical characteristics
double wheelBase = 140;
double wheelRadius = 31;

//interupt things
long now = 0;
long nextTime = 0;

int deltaTime = 500;
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
  pinMode(minibotVelPin, INPUT);
  enableInterruptFast(encoderPinA, CHANGE);
  enableInterruptFast(encoderPinB, CHANGE);
  Serial.println("now, v, lca, a, aSA, dSA, ea, pa, ma");
}



void loop () {
  // VAlue between 0..1023, map onto 0..MaxVelocity in mm/sec
  int minibotVelInput = analogRead(minibotVelPin);
  desiredMinibotVel = minibotVelInput * minibotMaxVelMm / 1023;
  
  // in wheel shaft rotation speed rad/sec
  double dSA = calculateAxleSpeedA(desiredMinibotVel, turnRate);
  double dSB = calculateAxleSpeedB(desiredMinibotVel, turnRate);

  int32_t currentCountA = countA;
  int32_t currentCountB = countB;

  //Calculate axle speeds from encoder
  now = millis();
  if(nextTime - now <= 0) {
    // compute delta ticks for A
    aSA = (currentCountA - lastCountA) * convertMStoS / (deltaTime);
    aSB = (currentCountB - lastCountB) * convertMStoS / (deltaTime);
    lastCountA = currentCountA;
    lastCountB = currentCountB;
    nextTime = now + deltaTime;
  }

    double errorA = dSA - aSA;
    double errorB = dSB - aSB;


  // map onto 0 to 255, see constrain function
  pidaSA = pidaSA + kP * errorA;
  int motorA = constrain(pidaSA, 0, 255);
  pidaSA = pidaSB + kP * errorB;
  int motorB = constrain(pidaSA, 0, 255);
  analogWrite(pinPWMA, motorA);
  analogWrite(pinPWMB, motorB);
  digitalWrite(pinDirectionA, HIGH);
  digitalWrite(pinDirectionB, HIGH);


  if (printTime < now) {
    printTime = now + 1000;
    // output more data?  which parameters are we interested in?
    Serial.print(now);
    Serial.print(", ");Serial.print(desiredMinibotVel);

    //a
    Serial.print(", ");Serial.print(lastCountA);
    Serial.print(", ");Serial.print(currentCountA);
    Serial.print(", ");Serial.print(aSA);
    Serial.print(", ");Serial.print(dSA);
    Serial.print(", ");Serial.print(errorA);
    Serial.print(", ");Serial.print(pidaSA);
    Serial.print(", ");Serial.print(motorA);


    //b
//    Serial.print(", lcB=");Serial.print(lastCountB);
//    Serial.print(", aSB=");Serial.print(aSB);
//    Serial.print(", dSB=");Serial.print(dSB);
//    Serial.print(", b=");Serial.print(currentCountB);
//    Serial.print(", nta=");Serial.print(nextTime);
//    Serial.print(", pb=");Serial.print(pidaSB);
//    Serial.print(", mb=");Serial.print(motorB);
//    Serial.print(", eb=");Serial.print(errorB);
    Serial.println("");
  }
  
  if(abs(kP * errorA) < EPS && abs(kP * errorB) < EPS){
      digitalWrite(LEDPin, HIGH);
  } else {
      digitalWrite(LEDPin, LOW);
  }
}

double calculateAxleSpeedA (double dVel, double tRate){
  return (2 * dVel + wheelBase * tRate) / (2 * wheelRadius);
}

double calculateAxleSpeedB (double dVel, double tRate){
  return (2 * dVel - wheelBase * tRate) / (2 * wheelRadius);
}

