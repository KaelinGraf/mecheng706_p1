#ifndef STRAFE_H
#define STRAFE_H

#include "state.h"
#include "pid.h"

class Strafe : public State {
  public:
    Strafe(Tiller* tiller) : State(State::STRAFE, tiller) {
      _gyro_pid = new PID<float>(100.0, 20.0, 0.0, 0.0, true, -100.0, 100.0); 

      _angle_pid = new PID<float>(20.0, 1.7, 0.0, 0.0, true, -100.0, 100.0);
      _x_pid = new PID<float>(4.0, 0.0, 0.0, 0.0, false, -100.0, 100.0);
      _y_pid = new PID<float>(6.7, 0.7, 0.0, 0.0, true, -100.0, 100.0);
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
    PID<float>* _gyro_pid;
    float _target_y;
    bool _use_left;            // locked sensor choice for this strafe
    bool _using_home_sensor;   // true if using home-wall sensor
    int exit_count = 0;
};

#endif // STRAFE_H
