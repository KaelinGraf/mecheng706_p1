#include "Arduino.h"
#include "find_corner.h"
#include "fsm.h"

void FindCorner::begin() {
  Serial.println("finding Corner...");
}

void FindCorner::end() {}

StateResult *FindCorner::poll() {
  Serial.println("finding Corner...");
  fsm_->switchState(State::TILL);

  return nullptr;
}