#ifndef MySoftwareSerial_h
#define MySoftwareSerial_h
#include "Arduino.h"
#include <SoftwareSerial.h>

class MySoftwareSerial: SoftwareSerial
{
public:
  MySoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false){}
  write(byte *b, uint8_t len){
    int ans = 0;
    for(int i = 0; i < len; i++)
      ans+=write(b[i]);
    return ans;
  }
};

#endif
