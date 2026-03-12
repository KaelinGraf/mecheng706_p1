#ifndef TILL_H
#define TILL_H

#include "state.h"

class Till : public State {
  public:
    Till(Fsm* fsm) : State(State::TILL, fsm), count_(0), last_millis_(0) {}
    ~Till() {};

    void begin() override;
    void end() override;
    StateResult* poll() override;
  
  private:
    int count_;
    unsigned long last_millis_;
};


#endif // TILL_H