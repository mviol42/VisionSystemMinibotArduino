#include <LiquidCrystal.h>

#define NEEDFORSPEED
#define INTERRUPT_FLAG_PINA5_S32 countA
#define INTERRUPT_FLAG_PINA4_S32 countB
#include <EnableInterrupt.h>

int pinDirectionA = 12;
int pinPWMA = 3;
int pinDirectionB = 13;
int pinPWMB = 11;
int encoderPinA = A5;
int encoderPinB = A4;
int delayTime = 5000;
const int numCols = 16;
const int numRows = 2;
//LiquidCrystal lcd(12, 11, 10, 9, 8, 7);
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
  enableInterruptFast(encoderPinA, CHANGE);
  enableInterruptFast(encoderPinB, CHANGE); 
//  lcd.begin(numCols, numRows);
//  lcd.clear();
//  lcd.print("hello world");
//  lcd.setCursor(0, 7); 
//  lcd.print("b");
}

void loop () {
  long currentCountA = countA;
  long currentCountB = countB;
  unsigned long currentTime = millis();
  if (time < currentTime) {
    time = currentTime + 1000;
//    lcd.setCursor(0, 1);
//    lcd.write(millis() / 1000);
        
    Serial.print("a=");Serial.print(currentCountA);Serial.print(", b=");Serial.println(currentCountB);
  }
  digitalWrite(pinDirectionA, LOW);
  digitalWrite(pinDirectionB, HIGH);
  if(currentCountA > 2000000){
    analogWrite(pinPWMA, 0);
    analogWrite(pinPWMB, 0);
  }
  else {
    analogWrite(pinPWMA, 200);
     analogWrite(pinPWMB, 200);
  }
}

