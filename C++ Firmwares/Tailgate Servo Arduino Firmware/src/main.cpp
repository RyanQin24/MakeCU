#include <Arduino.h>
#include <Servo.h>

int LDServoExd = 50;
int LDServoRt = 90;
int leftIO = 3;

int RDServoExd = 90 - LDServoExd;
int RDServoRt = LDServoExd - 90;
int rightIO = 5;

int LockExd = 90;
int LockRt = 0;
int lockIO = 6;

int inc = 5;
int acdly = 5;

bool extendlogic = false;
bool retractlogic = false;

Servo leftLift;
Servo rightLift;
Servo lock;

void setup() {
  Serial.begin(9600);
  leftLift.attach(leftIO); 
  rightLift.attach(rightIO);   
  lock.attach(lockIO);  
  leftLift.write(LDServoRt); 
  rightLift.write(RDServoRt);   
  lock.write(LockRt);  
  pinMode(13,OUTPUT);
  digitalWrite(13, LOW);
}

void loop() {
    
    if (Serial.available() > 0) {
        char c = Serial.read();
        switch (c){
            case 'e':
                extendlogic = true;  
                break;
            case 'r':
                retractlogic = true;
                leftLift.write(LDServoRt); 
                rightLift.write(RDServoRt);   
                break;
             case 'u':
                lock.write(LockExd);  
                break;
             case 'l':
                lock.write(LockRt);  
                break;
        }
        if(extendlogic){
            for(int i = LDServoRt; i > LDServoExd;  i -= inc){
                if (i < LDServoExd){
                    leftLift.write(LDServoExd);
                    rightLift.write(RDServoExd);
                }else{
                    leftLift.write(i);
                    rightLift.write(90 - i);
                }
                delay(acdly);
            }
            extendlogic = false;
        }else if (retractlogic){

        }
    }
}
