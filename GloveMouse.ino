#undef DEBUG

#include "Recognizer.h"
#include "Rotation.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//#include "SoftwareSerial.h"

//SoftwareSerial Serial1(9,8);

Rotation rot;
Recognizer r3(3), r4(4);
Event ev;
int8_t mode = 1;
volatile bool mpuFlag = false;
MPU6050 mpu;
uint8_t fifoBuffer[45];
const int8_t CHOOSE = 0;
uint8_t period = 10;
uint32_t last_gyro_send = 0;

void f2ba(float f, byte *dataArray) {
  for(int i = 0; i < 4; i++)
    dataArray[i] = ((uint8_t*)&f)[i];
}

float ba2f(byte *dataArray){
  float a;
  for(int i = 0; i < 4; i++)
    ((uint8_t*)&a)[i] = dataArray[i];
  return a;
}

void setup() {
  Serial.begin(115200);
  
//  Serial1.begin(115200);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  rot.init();
  attachInterrupt(0, dmpReady, RISING);
}

void dmpReady() {
  rot.dmpReady();
}

void loop() {
  r3.loop();
  r3.getEvent(ev);
  if(!ev.isNone()){
    if(ev.q == Recognizer::PRESS){
      rot.newSeries();
    }
    Serial.write(ev.q);
    Serial.write(ev.cnt);
    Serial.write((uint8_t)3);
    Serial.flush();
    
//    Serial1.print(ev.q);
//    Serial1.print(" ");
//    Serial1.print(ev.cnt);
//    Serial1.print(" ");
//    Serial1.println(3);
  }
  r4.loop();
  r4.getEvent(ev);
  if(!ev.isNone()){
    if(ev.q == Recognizer::PRESS){
      rot.newSeries();
    }
    Serial.write(ev.q);
    Serial.write(ev.cnt);
    Serial.write((uint8_t)4);
    Serial.flush();
    
//    Serial1.print(ev.q);
//    Serial1.print(" ");
//    Serial1.print(ev.cnt);
//    Serial1.print(" ");
//    Serial1.println(4);
  }
  uint32_t now = millis();
  if(r3.isPressed() && now-last_gyro_send >= period){
    last_gyro_send = now;
    Quaternion data;
    if(!rot.getRotation(data))
      return;
    Serial.write(Recognizer::MOVE);
    float w = data.w;
    float x = data.x;
    float y = data.y;
    float z = data.z;
    byte arr[4];
    f2ba(w, arr);
    Serial.write(arr, 4);
    f2ba(x, arr);
    Serial.write(arr, 4);
    f2ba(y, arr);
    Serial.write(arr, 4);
    f2ba(z, arr);
    Serial.write(arr, 4);
    Serial.flush();

//    Serial1.print(Recognizer::NONE);
//    Serial1.print(w, 4);
//    Serial1.print(" ");
//    Serial1.print(x, 4);
//    Serial1.print(" ");
//    Serial1.print(y, 4);
//    Serial1.print(" ");
//    Serial1.println(z, 4);
  }
  delay(1);

}
