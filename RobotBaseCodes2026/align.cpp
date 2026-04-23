#include "HardwareSerial.h"
#include "Arduino.h"
#include "align.h"
#include "tiller.h"

void Align::begin() {
  //---tiller_->println("Align: squaring up to wall with rear sensors");
  
  _angle_pid->resetPID();
  _x_pid->resetPID();
  _stable_count = 0;

  // Lock in the current distance so the robot just rotates in place to square up
  float fl = tiller_->_rear_left_ir->getAvg();
  float fr = tiller_->_rear_right_ir->getAvg();
  
  if (fl > 0 && fr > 0) {
      _target_x_dist = (fl + fr) / 2.0;
  } else {
      _target_x_dist = 8.0; // Safe fallback distance
  }
}

void Align::end() {
  //---tiller_->println("Align: finished, gyro reset.");
}

void Align::poll() {
  float fl = tiller_->_rear_left_ir->getAvg();
  float fr = tiller_->_rear_right_ir->getAvg();

  float angle_err = 0.0;
  float dist_avg = _target_x_dist;

  if (fl > 0.0 && fr > 0.0) {
      dist_avg = (fl + fr) / 2.0;
      angle_err = fl - fr; // Difference gives orientation error
  } else if (fl > 0.0) {
      dist_avg = fl;
  } else if (fr > 0.0) {
      dist_avg = fr;
  }

  float vx = _x_pid->update(dist_avg - _target_x_dist);
  float vtheta = _angle_pid->update(angle_err);

  // Apply efforts (signs match your homing.cpp kinematics for rear sensors)
  tiller_->_motors->writeAllMotors(-vx, 0, -vtheta);

  // Check if aligned (within 1.5 cm distance and ~1.0 difference between sensors)
  if (fl > 0.0 && fr > 0.0 &&
      fabs(dist_avg - _target_x_dist) < 1.5 &&
      fabs(angle_err) < 1.0) {
      _stable_count++;
  } else {
      _stable_count = 0;
  }

  // Require it to hold the aligned position for a few frames before exiting
  if (_stable_count >= 10) { 
      tiller_->_motors->writeAllMotors(0, 0, 0);
      tiller_->_gyro->resetAngle(); // Dead reckon complete
      
      // IMPORTANT: Change this to whatever state you want to enter next
      tiller_->switchState(State::TILL,{tiller_->get_y_tgt(),true,false}); 
  }
}