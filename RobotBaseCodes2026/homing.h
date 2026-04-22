#ifndef HOMING_H
#define HOMING_H

#include "state.h"
#include "pid.h"


class Homing : public State {
  public:
    enum HomingStage {
      HC_DRIVE_TO_WALL,
      HC_ALIGN_PERP,
      HC_CHECK_US,
      HC_ROTATE_MOVE,
      HC_STRAFE_ALIGN,
      HC_DONE,
    };
    Homing(Tiller* tiller) : State(State::HOMING, tiller),
      _angle_pid(nullptr), _x_pid(nullptr), _y_pid(nullptr), _rotate_pid(nullptr),
      _rotate_target(-1.0), _us_phase(0),_hs() {}
    ~Homing() {
      if (_angle_pid) delete _angle_pid;
      if (_x_pid) delete _x_pid;
      if (_y_pid) delete _y_pid;
      if (_rotate_pid) delete _rotate_pid;
    };

    void begin() override;
    void end() override;
    void poll() override;

  private:
    PID<float>* _angle_pid;
    PID<float>* _x_pid;
    PID<float>* _y_pid;
    PID<float>* _rotate_pid;
    float _rotate_target;  // target angle in radians (±PI/2)
    int _us_phase;     // 0 = rotating anticlockwise, 1 = rotating clockwise
    HomingStage _hs;
    int _gyro_error_count = 0;
};




#endif // HOMING_H