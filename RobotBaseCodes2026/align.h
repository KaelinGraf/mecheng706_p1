#ifndef ALIGN_H
#define ALIGN_H

#include "state.h"
#include "pid.h"

class Align : public State {
  public:
    Align(Tiller* tiller) : State(State::ALIGN, tiller) {
      // Re-using the PID tunings from the homing state
      _angle_pid = new PID<float>(12.0, 0.005, 0.0, 0.0, false, -100.0, 100.0);
      _x_pid = new PID<float>(6.0, 0.005, 0.0, 0.0, true, -100.0, 100.0);
    }
    ~Align() {
      delete _angle_pid;
      delete _x_pid;
    }

    void begin() override;
    void end() override;
    void poll() override;

  private:
    PID<float>* _angle_pid;
    PID<float>* _x_pid;
    float _target_x_dist;
    int _stable_count = 0;
};

#endif // ALIGN_H