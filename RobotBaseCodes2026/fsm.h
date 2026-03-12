#include "state.h"

class Fsm {
private:
    State *current_state_;
    State *states_[State::NUM_STATES];

public:
    Fsm();
    ~Fsm();

    bool switchState(State::Name newState);
    StateResult *pollState();
};