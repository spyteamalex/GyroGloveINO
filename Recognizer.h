#ifndef Recognizer_h
#define Recognizer_h
#include "Arduino.h"
struct Event{
    uint8_t q = 0;
    uint8_t cnt = 0;
    Event(){}
    Event(uint8_t q, uint8_t cnt){
      this->q = q;
      this->cnt = cnt;
    }
    bool isNone(){
      return q == 0;
    }
  };

class Recognizer{
  public: 
    static const uint8_t NONE = 0;
    static const uint8_t CLICK = 1;
    static const uint8_t PRESS = 2;
    static const uint8_t RELEASE = 3;
    static const uint8_t MOVE = 4;
   
  private:
    uint8_t port = 0;
    uint8_t q = NONE;
    uint8_t q_cnt = 0;
    
    bool last_state = 0;
    uint32_t last_ev = 0;
    bool prs = false;
    uint8_t cnt = 0;
    const uint32_t CLICK_GAP = 250;
    const uint32_t MIN_GAP = 30;
    const uint8_t MAX_CNT = 4;
    
  public: 
    bool getState(){
      return !digitalRead(port);
    }
    bool isPressed(){
      return prs;
    }
    Recognizer(uint8_t p){
      port = p;
    }
    void loop() {
      bool state = getState();
      uint32_t t = millis();
      if(!prs && cnt && t-last_ev > CLICK_GAP){
        if(state){
          q = PRESS;
          q_cnt = cnt;
          prs = true;
        }else{
          q = CLICK;
          q_cnt = cnt;
          cnt = 0;
        }
      }else if(!last_state && state && t-last_ev > MIN_GAP){
        cnt++;
        cnt = min(cnt, MAX_CNT);
        last_state = state;
        last_ev = t;
      }else if(last_state && !state && t-last_ev > MIN_GAP){
        last_state = state;
        last_ev = t;
        if(prs){
          q = RELEASE;
          q_cnt = cnt;
          prs = false;  
          cnt = 0;        
        }
//        else if(cnt >= MAX_CNT){
//          q = CLICK;
//          q_cnt = cnt;
//          cnt = 0;
//        }
      }
    }
    void getEvent(Event &e){
      e.q = q;
      e.cnt = q_cnt;
      q = 0;
      q_cnt = 0;
    }
};

#endif
