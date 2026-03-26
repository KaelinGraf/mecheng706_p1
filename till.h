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
      ultrasonic_count_ = 0;
      
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
    void end() override;
    void poll() override;
  
  private:
    float process_noise_ = 0.4; 
    float sensor_noise_ = 2;  

    int count_;
    unsigned long last_millis_;
    PID<float>* _gyro_pid;
    PID<float>* _y_pid;
    int ultrasonic_count_;
    float _prev_y_est;
    float _last_y_var;
    unsigned long _last_y_millis;

    float getFilteredY(); 
};


#endif // TILL_H