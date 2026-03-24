#include "HardwareSerial.h"
#include "Arduino.h"
#include "strafe.h"
#include "tiller.h"
#include "till.h" // For TILL state transition

void Strafe::begin() {
  Serial.println("strafing");
  tiller_->inc_y_tgt();
  _target_y = tiller_->get_y_tgt();
  
  tiller_->print("Strafe target y: ");
  tiller_->println(_target_y);
}

void Strafe::end() {
  Serial.println("stopped strafing");
}

void Strafe::poll() {
  float left_ir = tiller_->_front_left_ir->readSensor();
  float right_ir = tiller_->_front_right_ir->readSensor();
  
  float dist_avg = 15.0; // Assume we are at target if missing data
  float angle_err = 0.0;
  
  if (left_ir > 0.0 && right_ir > 0.0) {
    dist_avg = (left_ir + right_ir) / 2.0;
    angle_err = left_ir - right_ir; // positive if left is further -> needs positive vtheta correction
  }

  // To remain 15cm from the wall directly in front:
  float vx = _x_pid->update(dist_avg - 15.0); // positive if too far -> go forward
  
  // To remain perfectly squared:
  float vtheta = _angle_pid->update(angle_err - 0.0); // positive if left is further -> turn CCW

  float side_left = tiller_->_side_left_ir->readSensor();
  float side_right = tiller_->_side_right_ir->readSensor();
  
  bool wall_on_left = (side_left > 0.0 && (side_right <= 0.0 || side_left < side_right));
  
  float y_error = 0.0;
  float vy = 0.0;
  
  if (wall_on_left) {
      y_error = _target_y - side_left; 
      // If error is positive, need to move right -> positive vy
      vy = _y_pid->update(y_error);
  } else if (side_right > 0.0) {
      y_error = _target_y - side_right;
      // If error is positive, need to move left -> negative vy
      vy = -_y_pid->update(y_error);
  } else {
      // Blind strafing default?
      vy = 50.0; 
  }
  
  tiller_->_motors->writeAllMotors(vx, vy, vtheta);
  
  // Check if we hit row target distance (y error near 0)
  if (abs(y_error) < 1.0 && (side_left > 0.0 || side_right > 0.0)) {
      tiller_->switchState(State::TILL);
  }
}
