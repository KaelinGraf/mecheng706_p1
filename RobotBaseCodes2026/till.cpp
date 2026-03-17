#include "Arduino.h"
#include "till.h"
#include "tiller.h"

void Till::begin() {
  Serial.println("tilling");
  last_millis_ = millis();
  count_ = 0;
}

void Till::end() {
  Serial.println("stopped tilling");
}

void Till::poll() {
  const long t = millis();
  if ((t - last_millis_) > 100) {
    count_++;
    last_millis_ = t;
  }

  if (count_ > 30) {
    tiller_->switchState(State::TURN);
  }
}