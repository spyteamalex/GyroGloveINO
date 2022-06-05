#ifndef Rotation_h
#define Rotation_h
#include "I2Cdev.h"
#include "Arduino.h"
#include "MPU6050_6Axis_MotionApps20.h"

class Rotation{
  private:
  volatile bool mpuFlag = false;
  MPU6050 mpu;
  uint8_t fifoBuffer[45];
  Quaternion Q1, Q2;
  bool newSeriesReq = false;

  public:
  Rotation(){}
  
  void init() {
    Wire.begin();
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
  }
  void dmpReady() {
    mpuFlag = true;
  }

  bool updateQuaternion(){
    if (mpuFlag && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      Quaternion q;
      VectorFloat gravity;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpuFlag = false;
      Q2 = q;
      return true;
    }
    return false;
  }

  //Первый пакет станет Q1
  void newSeries(){
    newSeriesReq = true;
  }

  //Q2 = Q1 * rot
  //Q1^-1 * Q2 = rot
  //(~Q1 * Q2)/|Q1|^2 = rot 
  //где ~Q1 - сопряженное с Q1
  bool getRotation(Quaternion &q) {
    bool flg = updateQuaternion();
    if(newSeriesReq){
      if(!flg){
        return false;
      }
      Q1 = Q2; 
      newSeriesReq = false;
    }
    Quaternion rot = Q1.getConjugate().getProduct(Q2);
    float a = Q1.getMagnitude();
    a *= a;
    rot = Quaternion(rot.w/a, rot.x/a, rot.y/a, rot.z/a);
    q.w = rot.w;
    q.x = rot.x;
    q.y = rot.y;
    q.z = rot.z;
    return true;
  }
};
#endif
