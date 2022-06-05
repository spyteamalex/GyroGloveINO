#ifndef Angles_h
#define Angles_h
#include "I2Cdev.h"
#include "Arduino.h"
#include "MPU6050_6Axis_MotionApps20.h"

enum RotSeq{zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy,xzx};

void twoaxisrot(float r11, float r12, float r21, float r31, float r32, float res[]){
  res[0] = atan2( r11, r12 );
  res[1] = acos ( r21 );
  res[2] = atan2( r31, r32 );
}

void threeaxisrot(float r11, float r12, float r21, float r31, float r32, float res[]){
  res[0] = atan2( r31, r32 );
  res[1] = asin ( r21 );
  res[2] = atan2( r11, r12 );
}

void quaternion2Euler(const Quaternion &q, float* res, RotSeq rotSeq = zxy)
{
    switch(rotSeq){
    case zyx:
      threeaxisrot( 2*(q.x*q.y + q.w*q.z),
                     q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
                    -2*(q.x*q.z - q.w*q.y),
                     2*(q.y*q.z + q.w*q.x),
                     q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
                     res);
      break;

    case zyz:
      twoaxisrot( 2*(q.y*q.z - q.w*q.x),
                   2*(q.x*q.z + q.w*q.y),
                   q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
                   2*(q.y*q.z + q.w*q.x),
                  -2*(q.x*q.z - q.w*q.y),
                  res);
      break;

    default:
    case zxy:
      threeaxisrot( -2*(q.x*q.y - q.w*q.z),
                      q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
                      2*(q.y*q.z + q.w*q.x),
                     -2*(q.x*q.z - q.w*q.y),
                      q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
                      res);
      break;
      
    case zxz:
      twoaxisrot( 2*(q.x*q.z + q.w*q.y),
                  -2*(q.y*q.z - q.w*q.x),
                   q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
                   2*(q.x*q.z - q.w*q.y),
                   2*(q.y*q.z + q.w*q.x),
                   res);
      break;

    case yxz:
      threeaxisrot( 2*(q.x*q.z + q.w*q.y),
                     q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
                    -2*(q.y*q.z - q.w*q.x),
                     2*(q.x*q.y + q.w*q.z),
                     q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
                     res);
      break;

    case yxy:
      twoaxisrot( 2*(q.x*q.y - q.w*q.z),
                   2*(q.y*q.z + q.w*q.x),
                   q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
                   2*(q.x*q.y + q.w*q.z),
                  -2*(q.y*q.z - q.w*q.x),
                  res);
      break;

    case yzx:
      threeaxisrot( -2*(q.x*q.z - q.w*q.y),
                      q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
                      2*(q.x*q.y + q.w*q.z),
                     -2*(q.y*q.z - q.w*q.x),
                      q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
                      res);
      break;

    case yzy:
      twoaxisrot( 2*(q.y*q.z + q.w*q.x),
                  -2*(q.x*q.y - q.w*q.z),
                   q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
                   2*(q.y*q.z - q.w*q.x),
                   2*(q.x*q.y + q.w*q.z),
                   res);
      break;

    case xyz:
      threeaxisrot( -2*(q.y*q.z - q.w*q.x),
                    q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
                    2*(q.x*q.z + q.w*q.y),
                   -2*(q.x*q.y - q.w*q.z),
                    q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
                    res);
      break;

    case xyx:
      twoaxisrot( 2*(q.x*q.y + q.w*q.z),
                  -2*(q.x*q.z - q.w*q.y),
                   q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
                   2*(q.x*q.y - q.w*q.z),
                   2*(q.x*q.z + q.w*q.y),
                   res);
      break;

    case xzy:
      threeaxisrot( 2*(q.y*q.z + q.w*q.x),
                     q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
                    -2*(q.x*q.y - q.w*q.z),
                     2*(q.x*q.z + q.w*q.y),
                     q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
                     res);
      break;

    case xzx:
      twoaxisrot( 2*(q.x*q.z - q.w*q.y),
                   2*(q.x*q.y + q.w*q.z),
                   q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
                   2*(q.x*q.z + q.w*q.y),
                  -2*(q.x*q.y - q.w*q.z),
                  res);
      break;
   }
}
#endif
