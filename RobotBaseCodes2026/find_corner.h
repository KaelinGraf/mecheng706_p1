#ifndef FIND_CORNER_H
#define FIND_CORNER_H

#include "state.h"

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

    FindCorner(Tiller* tiller) : State(State::FIND_CORNER, tiller) {}
    ~FindCorner() {};

    void begin() override;
    void end() override;
    void poll() override;
};


#endif // FIND_CORNER_H