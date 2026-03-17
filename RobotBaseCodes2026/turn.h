#ifndef TURN_H
#define TURN_H

#include "state.h"

class Turn : public State {
  public:
    Turn(Tiller* tiller) : State(State::TILL, tiller), count_(0), last_millis_(0) {};
    ~Turn() {};

    void begin() override;
    void end() override;
    void poll() override;
  
  private:
    int count_;
    unsigned long last_millis_;
};


#endif // TURN_H