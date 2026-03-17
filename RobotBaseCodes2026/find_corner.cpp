#include "Arduino.h"
#include "find_corner.h"
#include "tiller.h"

void FindCorner::begin() {
  Serial.println("finding Corner...");
}

void FindCorner::end() {}

void FindCorner::poll() {
  Serial.println("finding Corner...");
  tiller_->switchState(State::TILL);
}