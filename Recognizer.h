#ifndef Recognizer_h
#define Recognizer_h
#include "Arduino.h"
struct Event{
    int8_t q = 0;
    int8_t cnt = 0;
    Event(){}
    Event(int8_t q, int8_t cnt){
      this->q = q;
      this->cnt = cnt;
    }
    bool isNone(){
      return q == 0;
    }
  };

class Recognizer{
  public: 
    static const int8_t NONE = 0;
    static const int8_t CLICK = 1;
    static const int8_t PRESS = 2;
    static const int8_t RELEASE = 3;
   
  private:
    int8_t port = 0;
    int8_t q = NONE;
    int8_t q_cnt = 0;
    
    bool last_state = 0;
    uint32_t last_ev = 0;
    bool prs = false;
    int8_t cnt = 0;
    const int32_t CLICK_GAP = 250;
    const int32_t MIN_GAP = 10;
    const int8_t MAX_CNT = 3;
    
  public: 
    bool getState(){
      return !digitalRead(port);
    }
    bool isPressed(){
      return prs;
    }
    Recognizer(int8_t p){
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
        }else if(cnt >= MAX_CNT){
          q = CLICK;
          q_cnt = cnt;
          cnt = 0;
        }
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
