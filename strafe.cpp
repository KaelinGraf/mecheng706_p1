#include "HardwareSerial.h"
#include "Arduino.h"
#include "strafe.h"
#include "tiller.h"
#include "till.h" // For TILL state transition

void Strafe::begin() {
  Serial.println("strafing");
  tiller_->inc_y_tgt();
  
  // y_target is already the sensor distance (mirrored after midpoint)
  _target_y = tiller_->get_y_tgt();
  _using_home_sensor = !tiller_->useFarSensor();

  int turns = tiller_->getTurnCount();
  bool home_is_left;
  if (tiller_->getHomeWallSensor() == 0) {
    home_is_left = (turns % 2 == 0);
  } else {
    home_is_left = (turns % 2 != 0);
  }

  // Pick the correct physical sensor
  if (_using_home_sensor) {
    _use_left = home_is_left;
  } else {
    _use_left = !home_is_left;
  }

  tiller_->print("Strafe: setpoint="); tiller_->print(_target_y);
  tiller_->print(" useFar=");          tiller_->print(!_using_home_sensor);
  tiller_->print(" sensor=");          tiller_->println(_use_left ? "LEFT" : "RIGHT");
}

void Strafe::end() {
  Serial.println("stopped strafing");
}

void Strafe::poll() {
  float left_ir = tiller_->_front_left_ir->readSensor();
  float right_ir = tiller_->_front_right_ir->readSensor();
  
  float dist_avg = 15.0;
  float angle_err = 0.0;
  
  if (left_ir > 0.0 && right_ir > 0.0) {
    dist_avg = (left_ir + right_ir) / 2.0;
    angle_err = left_ir - right_ir;
  }

  float vx = _x_pid->update(dist_avg - 15.0);
  float vtheta = _angle_pid->update(angle_err - 0.0);

  // Read the locked-in sensor
  float side_reading = _use_left ? tiller_->_side_left_ir->readSensor() 
                                 : tiller_->_side_right_ir->readSensor();
  
  float y_error = 0.0;
  float vy = 0.0;
  
  if (side_reading > 0.0) {
    y_error = _target_y - side_reading;
    if (_use_left) {
      vy = -_y_pid->update(y_error);  // left wall: positive error → strafe right
    } else {
      vy = _y_pid->update(y_error);   // right wall: positive error → strafe left
    }
  } else {
    // Sensor invalid — blind strafe toward the target wall
    vy = _use_left ? -50.0 : 50.0;
  }

  // Diagnostics (throttled to avoid flooding serial)
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    tiller_->print("Strafe: side=");    tiller_->print(side_reading);
    tiller_->print(" tgt=");            tiller_->print(_target_y);
    tiller_->print(" err=");            tiller_->print(y_error);
    tiller_->print(" vy=");             tiller_->println(vy);
    last_print = millis();
  }
  
  tiller_->_motors->writeAllMotors(vx, vy, vtheta);
  
  // Check if we hit row target distance
  if (side_reading > 0.0 && fabs(y_error) < 2.0) {
      tiller_->println("Strafe: target reached, switching to TILL");
      tiller_->switchState(State::TILL);
  }
}
