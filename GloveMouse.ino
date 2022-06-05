#include "Recognizer.h"
#include "Rotation.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

Rotation rot;
Recognizer r(3);
Event ev;
int8_t mode = 1;
volatile bool mpuFlag = false;
MPU6050 mpu;
uint8_t fifoBuffer[45];
const int8_t CHOOSE = 0;

void setup() {
  Serial.begin(115200);
  pinMode(3, INPUT_PULLUP);
  rot.init();
  attachInterrupt(0, dmpReady, RISING);
}

void dmpReady() {
  rot.dmpReady();
}

void loop() {
  r.loop();
  r.getEvent(ev);
  if(!ev.isNone()){
    if(ev.q == Recognizer::PRESS){
      rot.newSeries();
    }
    Serial.print(ev.q);
    Serial.print(" ");
    Serial.println(ev.cnt);
  }
  if(r.isPressed()){
    float data[3];
    if(!rot.getRotation(data))
      return;
    Serial.print(-1);
    Serial.print(" ");
    Serial.print(data[0]);
    Serial.print(" ");
    Serial.print(data[1]);
    Serial.print(" ");
    Serial.println(data[2]);
  }
}
