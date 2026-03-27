#ifndef FIND_CORNER_H
#define FIND_CORNER_H

#include "state.h"
#include "pid.h"

class FindCorner : public State {
  public:
    enum HomingStage {
      HC_DRIVE_TO_WALL,
      HC_ALIGN_PERP,
      HC_CHECK_US,
      HC_ROTATE_MOVE,
      HC_STRAFE_ALIGN,
      HC_DONE,
      HC_ABORT
    };

    FindCorner(Tiller* tiller) : State(State::FIND_CORNER, tiller),
      _angle_pid(nullptr), _x_pid(nullptr), _y_pid(nullptr), _rotate_pid(nullptr),
      _rotate_target(0.0), _rotate_phase(0) {}
    ~FindCorner() {
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
    int _rotate_phase;     // 0 = rotating, 1 = driving forward
    int _target_wall;      // 0 = none, 1 = left, 2 = right
};


#endif // FIND_CORNER_H