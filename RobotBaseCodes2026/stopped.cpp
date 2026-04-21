#include "Arduino.h"
#include "stopped.h"
#include "tiller.h"

void Stopped::begin() {
  tiller_->println("Stopped");
}

void Stopped::end() {
}

void Stopped::poll() {
  const long t = millis();

  if (t - last_millis_ > 1000) {
    tiller_->println("STOPPED---------");
  }
}