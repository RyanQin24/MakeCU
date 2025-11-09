#include <Arduino.h>

int TrigpinFront = 7;
int EchopinFront = 8;

int TrigpinLeft = 9;
int EchopinLeft = 10;

int TrigpinRight = 11;
int EchopinRight = 12;

void setup() {
  Serial.begin(9600);
  pinMode(TrigpinFront, OUTPUT);
  pinMode(EchopinFront, INPUT);

  pinMode(TrigpinLeft, OUTPUT);
  pinMode(EchopinLeft, INPUT);

  pinMode(TrigpinRight, OUTPUT);
  pinMode(EchopinRight, INPUT);
}

void loop() {
  digitalWrite(TrigpinFront, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigpinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigpinFront, LOW);
  long Frontduration = pulseIn(EchopinFront, HIGH, 25000); // timeout 25ms (~4m)

  digitalWrite(TrigpinLeft, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigpinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigpinLeft, LOW);
  long Leftduration = pulseIn(EchopinLeft, HIGH, 25000); // timeout 25ms (~4m)

  digitalWrite(TrigpinRight, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigpinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigpinRight, LOW);
  long Rightduration = pulseIn(EchopinRight, HIGH, 25000); // timeout 25ms (~4m)
  long Frontdistance = 428;
  long Leftdistance = 428;
  long Rightdistance = 428;
  if(Frontduration > 0){
    Frontdistance = Frontduration * 0.0343 / 2;
  }
  if(Leftduration > 0){
    Leftdistance = Leftduration * 0.0343 / 2;
  }
  if(Rightduration > 0){
    Rightdistance = Rightduration * 0.0343 / 2;
  }
  Serial.println(String(Frontdistance) + "," + String(Leftdistance) + "," + String(Rightdistance));
 
  delay(500);
}