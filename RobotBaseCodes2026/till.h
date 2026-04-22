#include <stdint.h>
#ifndef TILL_H
#define TILL_H

#include "state.h"
#include "pid.h"

#define TILL_SPEED 50

class Till : public State {
  public:
    Till(Tiller* tiller) : State(State::TILL, tiller), count_(0) {
      _gyro_pid = nullptr;
      _y_pid = nullptr;
      endzone_count_ = 0;
      tilling_speed_ = 50;
    }
    ~Till() {};
    enum SIDE_SENSOR{
      left,
      right,
    };
    SIDE_SENSOR _target_sensor;
    float _target_y;
    SIDE_SENSOR findYRef();
    void begin() override {};
    void begin(TillData data) override;
    void end() override;
    void poll() override;
  
  private:
    int count_;
    PID<float>* _gyro_pid;
    PID<float>* _y_pid;
    int endzone_count_;
    int8_t tilling_speed_; // 50 foward, -50 backward
};


#endif // TILL_H