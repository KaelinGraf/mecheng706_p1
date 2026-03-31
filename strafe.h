#ifndef STRAFE_H
#define STRAFE_H

#include "state.h"
#include "pid.h"

class Strafe : public State {
  public:
    Strafe(Tiller* tiller) : State(State::STRAFE, tiller) {
      _angle_pid = new PID<float>(15.0, 0.0, 0.0, 0.0, false, -100.0, 100.0);
      _x_pid = new PID<float>(10.0, 0.0, 0.0, 0.0, false, -100.0, 100.0);
      _y_pid = new PID<float>(10.0, 0.0, 0.0, 0.0, false, -150.0, 150.0);
    }
    ~Strafe() {
      delete _angle_pid;
      delete _x_pid;
      delete _y_pid;
    };

    void begin() override;
    void end() override;
    void poll() override;

  private:
    PID<float>* _angle_pid;
    PID<float>* _x_pid;
    PID<float>* _y_pid;
    float _target_y;
    bool _use_left;            // locked sensor choice for this strafe
    bool _using_home_sensor;   // true if using home-wall sensor
};

#endif // STRAFE_H
