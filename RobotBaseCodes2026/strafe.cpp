#include "HardwareSerial.h"
#include "Arduino.h"
#include "strafe.h"
#include "tiller.h"
#include "till.h" // For TILL state transition

void Strafe::begin() {
  tiller_->_ultrasonic->runUltrasonic();
  Serial.println("strafing");
  tiller_->inc_y_tgt();
  
  _target_y = tiller_->get_y_tgt();

  tiller_->print("Strafe: setpoint="); tiller_->print(_target_y);
  tiller_->print(" distance=");          tiller_->print(tiller_->_ultrasonic->readSensor());
  tiller_->print(" sensor=");          tiller_->println("ULTRASONIC");
}

void Strafe::end() {
  Serial.println("stopped strafing");
}

void Strafe::poll() {
  float close_ir = tiller_->_front_left_ir->readSensor();
  float ultrasonic = tiller_->_ultrasonic->readSensor();
  
  float dist_avg = 15.0;
  float angle_err = 0.0;
  
  if (close_ir > 0.0 && ultrasonic > 0.0) {
    dist_avg = close_ir;
  }
  
  float vx = _x_pid->update(dist_avg - 15.0);
  float vtheta = _angle_pid->update(angle_err - 0.0);
  
  float y_error = 0.0;
  float vy = 0.0;
  
  if (ultrasonic > 0.0) {
    y_error = _target_y - ultrasonic;
    //vy = _y_pid->update(y_error);   // ultrasonic on left → strafe left
  } else {
    // Sensor invalid — blind strafe toward the target wall
    vy = 50.0;
  }
  
  // Diagnostics (throttled to avoid flooding serial)
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    tiller_->print("Strafe: side=");    tiller_->print("ULTRASONIC on left side");
    tiller_->print(" tgt=");            tiller_->print(_target_y);
    tiller_->print(" err=");            tiller_->print(y_error);
    tiller_->print(" vy=");             tiller_->println(vy);
    last_print = millis();
  }
  
  //tiller_->_motors->writeAllMotors(vx, vy, vtheta);
  
  // Check if we hit row target distance
  if (ultrasonic > 0.0 && fabs(y_error) < 2.0) {
      tiller_->println("Strafe: target reached, switching to TILL");
      tiller_->switchState(State::TILL);
  }
}
