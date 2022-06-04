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

  void updateQuaternion(){
    if (mpuFlag && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      Quaternion q;
      VectorFloat gravity;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpuFlag = false;
      Q2 = q;
    }
  }

  void updateDefault(){
    updateQuaternion();
    Q1 = Q2;
  }

  //Q2 = Q1 * rot
  //Q1^-1 * Q2 = rot
  //(~Q1 * Q2)/|Q1|^2 = rot 
  //где ~Q1 - сопряженное с Q1
  void getRotation(float* vf) {
    updateQuaternion();
    Quaternion rot = Q1.getConjugate().getProduct(Q2);
    float a = Q1.getMagnitude();
    a *= a;
    rot = Quaternion(rot.w/a, rot.x/a, rot.y/a, rot.z/a);
    mpu.dmpGetEuler(vf, &rot);
  }
};
#endif
