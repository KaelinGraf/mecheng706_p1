#ifndef FIND_CORNER_H
#define FIND_CORNER_H

#include "state.h"

class FindCorner : public State {
  public:
    FindCorner(Tiller* tiller) : State(State::FIND_CORNER, tiller) {}
    ~FindCorner() {};

    void begin() override;
    void end() override;
    void poll() override;
};


#endif // FIND_CORNER_H