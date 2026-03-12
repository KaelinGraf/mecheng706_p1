#include "HardwareSerial.h"
#include "fsm.h"
#include "find_corner.h"
#include "till.h"
#include "turn.h"

Fsm::Fsm() {
  current_state_= new FindCorner(this);
  current_state_->begin();

  states_[State::FIND_CORNER] = new FindCorner(this);
  states_[State::TILL] = new Till(this);
  states_[State::TURN] = new Turn(this);
}

bool Fsm::switchState(State::Name newState) {
// 1. End the current state
    if (current_state_) {
        current_state_->end();
    }

    // 2. Look up the new state in our array (O(1) time complexity, no switch!)
    current_state_ = states_[newState];

    // 3. Begin the new state
    if (current_state_) {
        current_state_->begin();
        return true;
    }

    Serial.print("unknown state");
    return false;
}

StateResult *Fsm::pollState() {
  if (current_state_) {
    return current_state_->poll();
  } else {
    return nullptr;
  }
};