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

    FindCorner(Tiller* tiller) : State(State::FIND_CORNER, tiller), _angle_pid(nullptr), _x_pid(nullptr) {}
    ~FindCorner() {
      if (_angle_pid) delete _angle_pid;
      if (_x_pid) delete _x_pid;
    };

    void begin() override;
    void end() override;
    void poll() override;

  private:
    PID<float>* _angle_pid;
    PID<float>* _x_pid;
};


#endif // FIND_CORNER_H