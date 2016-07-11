
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

//print loop, number of cycles between prints
int printTime = 0;
int printNow = 100;
boolean headerPrinted = false;

//Variables for calculations
double turnRate = 0;
double aSA = 0;
double aSB = 0;
// use fadeAsA to calculate errorA = fadeAsA - dSA
double fadeAsA = 0;
double fadeAsB = 0;
double fade = 0.5;  // fadeAsA =  fade * aSA + (1 - fade) * fadeAsA

double pidaSA = 0;
double pidaSB = 0;
int desiredAxleSpeedA = 10;
int desiredAxleSpeedB = 10;
int desiredMinibotVel = 10;
double errorA = 0;
double errorB = 0;
int convertMStoS = 1000;


//physical characteristics
double minibotMaxVelMm = 30000.0; // max vel in mm/sec
double wheelBase = 140;
double wheelRadius = 31;

// pid timing loop
double EPS = 15.0;
long now = 0;
long nextTime = 0;
long lastTime = 0;
int deltaTime = 50;

double kP = 2.5;

long lastCountA = 0;
long lastCountB = 0;
double actualVelA = 0;
double actualVelB = 0;

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

}



void loop () {
  now = millis();
  if(nextTime - now <= 0) {

    // VAlue between 0..1023, map onto 0..MaxVelocity in mm/sec
    int minibotVelInput = analogRead(minibotVelPin);
    desiredMinibotVel = minibotVelInput * minibotMaxVelMm / 1023;
  
    // in wheel shaft rotation speed rad/sec
    double dSA = calculateAxleSpeedA(desiredMinibotVel, turnRate);
    double dSB = calculateAxleSpeedB(desiredMinibotVel, turnRate);

    long currentCountA = countA;
    long currentCountB = countB;

   
    //Calculate axle speeds from encoder
    // compute delta ticks 
    int deltaTickA = currentCountA - lastCountA;
    int deltaTickB = currentCountB - lastCountB;
    long elapsedTime = now - lastTime;
    aSA = (currentCountA - lastCountA) * convertMStoS / (deltaTime);
    aSB = (currentCountB - lastCountB) * convertMStoS / (deltaTime);
    fadeAsA =  fade * aSA + (1 - fade) * fadeAsA;
    fadeAsB =  fade * aSB + (1 - fade) * fadeAsB;
    double errorA = dSA - fadeAsA;
    double errorB = dSB - fadeAsB;

    // map onto 0 to 255, see constrain function
    pidaSA = pidaSA + kP * errorA;
    int motorA = constrain(pidaSA, 0, 255);
    pidaSB = pidaSB + kP * errorB;
    int motorB = constrain(pidaSB, 0, 255);
    analogWrite(pinPWMA, motorA);
    analogWrite(pinPWMB, motorB);
    digitalWrite(pinDirectionA, HIGH);
    digitalWrite(pinDirectionB, HIGH);

    if(abs(kP * errorA) < EPS && abs(kP * errorB) < EPS){
        digitalWrite(LEDPin, HIGH);
    } else {
        digitalWrite(LEDPin, LOW);
    }

   if (printTime == printNow) {
      if (!headerPrinted) {
        Serial.println("now, et, v, asa, fsa, dsa, dta, ea, pa, ma, asb, fsb, dsb, dtb, eb, pb, mb");
        headerPrinted = true;
      }
      Serial.print(now);
      Serial.print(", ");Serial.print(elapsedTime);
      Serial.print(", ");Serial.print(desiredMinibotVel);

      //a
      Serial.print(", ");Serial.print(aSA);
      Serial.print(", ");Serial.print(fadeAsA);
      Serial.print(", ");Serial.print(dSA);
      Serial.print(", ");Serial.print(deltaTickA);
      Serial.print(", ");Serial.print(errorA);
      Serial.print(", ");Serial.print(pidaSA);
      Serial.print(", ");Serial.print(motorA);

      //b
      Serial.print(", ");Serial.print(aSB);
      Serial.print(", ");Serial.print(fadeAsB);
      Serial.print(", ");Serial.print(dSB);
      Serial.print(", ");Serial.print(deltaTickB);
      Serial.print(", ");Serial.print(errorB);
      Serial.print(", ");Serial.print(pidaSB);
      Serial.print(", ");Serial.print(motorB);
      Serial.println("");   
      printTime = 0;

    }

    lastCountA = currentCountA;
    lastCountB = currentCountB;
    nextTime = now + deltaTime;
    lastTime = now;
    printTime++;
  }
  
}

double calculateAxleSpeedA (double dVel, double tRate){
//  return (2 * dVel + wheelBase * tRate) / (2 * wheelRadius);
  return (dVel ) / (2 * 3.14159 *  wheelRadius);
}

double calculateAxleSpeedB (double dVel, double tRate){
//  return (2 * dVel - wheelBase * tRate) / (2 * wheelRadius);
  return (dVel ) / (2 * 3.14159 *  wheelRadius);
}

