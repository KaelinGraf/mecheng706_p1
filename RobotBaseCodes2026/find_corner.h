#ifndef FIND_CORNER_H
#define FIND_CORNER_H

#include "state.h"

class FindCorner : public State {
  public:
    FindCorner(Fsm* fsm) : State(State::FIND_CORNER, fsm) {}
    ~FindCorner() {};

    void begin() override;
    void end() override;
    StateResult *poll() override;
};


#endif // FIND_CORNER_H