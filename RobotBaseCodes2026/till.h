#include <stdint.h>
#ifndef TILL_H
#define TILL_H

#include "state.h"
#include "pid.h"



class Till : public State {
  public:
    Till(Tiller* tiller) : State(State::TILL, tiller), count_(0), last_millis_(0) {
      _gyro_pid = nullptr;
      _y_pid = nullptr;
      
    }
    ~Till() {};
    enum SIDE_SENSOR{
      left,
      right,
    };
    SIDE_SENSOR _target_sensor;
    float _target_y;
    SIDE_SENSOR findYRef();
    void begin() override;
    // void begin(void* data) override;
    void end() override;
    void poll() override;
  
  private:
    int count_;
    unsigned long last_millis_;
    PID<uint16_t>* _gyro_pid;
    PID<uint16_t>* _y_pid;
};


#endif // TILL_H