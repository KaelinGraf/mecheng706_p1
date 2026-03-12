#include "Arduino.h"
#include "turn.h"
#include "fsm.h"

void Turn::begin() {
  Serial.println("turning");
  last_millis_ = millis();
  count_ = 0;
}

void Turn::end() {
  Serial.println("stopped turning");
}

StateResult *Turn::poll() {
  const long t = millis();
  if ((t - last_millis_) > 100) {
    count_++;
    last_millis_ = t;
  }

  if (count_ > 30) {
    fsm_->switchState(State::TILL);
  }
  return nullptr;
}