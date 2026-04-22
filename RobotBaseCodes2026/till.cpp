#include "HardwareSerial.h"
#include "Arduino.h"
#include "till.h"
#include "tiller.h"


void Till::begin(TillData data) {
  tiller_->println("tilling");

  count_ = 0;

  _target_y = data.distance;

  tiller_->print("y from wall: ");
  tiller_->print(_target_y);
  tiller_->println(" cm");
  tiller_->print("drive foward? ");
  tiller_->println(data.drive_foward ? "yes" : "no");

  tiller_->_gyro->resetAngle();

  tilling_speed_ = data.drive_foward ? TILL_SPEED : -TILL_SPEED; // move between driving foward and backward
  

  tiller_->_gyro->resetAngle();
  endzone_count_ = 0;

  _gyro_pid = new PID<float>(100.0, 20.0, 0.0, 0.0, true, -100.0, 100.0); 
  _y_pid = new PID<float>(3.0, 0.05, 1.0, 0.0, true, -100.0, 100.0);
}

void Till::end() {
  tiller_->println("stopped tilling");
  tiller_->incTurnCount();
  delete _gyro_pid;
  _gyro_pid = nullptr;
  delete _y_pid;
  _y_pid = nullptr;
}

void Till::poll() {
  float angle_control_effort;
  float y_control_effort;
  float current_y;
  float y_error;
  float last_y;

  float heading = tiller_->_gyro->getAngle();
  last_y = current_y;

  tiller_->_ultrasonic->readSensor();
  current_y = tiller_->_ultrasonic->getAvg();

  if (current_y < 0) current_y = _target_y; // fallback

  y_error = current_y - _target_y;

  tiller_->print("dist error: "); tiller_->println(y_error);
  tiller_->print("heading: ");    tiller_->println(heading);
  angle_control_effort = _gyro_pid->update(heading);
  y_control_effort = -_y_pid->update(y_error); // move right (negative Vy) to increase distance to left wall
  // y_control_effort = 0;
  
  tiller_->_motors->writeAllMotors(tilling_speed_, y_control_effort, angle_control_effort);

  float left_dist;
  float right_dist;
  if (tilling_speed_ > 0) {
    // foward
    left_dist = tiller_->_front_left_ir->readSensor() - 9.5;
    right_dist = tiller_->_front_right_ir->readSensor() - 9.5;
  } else {
    left_dist = tiller_->_rear_left_ir->readSensor() - 1.5;
    right_dist = tiller_->_rear_right_ir->readSensor() - 1.5;
  }

  float u_dist = left_dist > 0 && right_dist > 0 
    ? (left_dist + right_dist) / 2 
    : max(left_dist, right_dist);

  if (u_dist > 0.0 && u_dist < 28.0) {
    endzone_count_++;
  } else {
    endzone_count_ = 0;
  }

  if (endzone_count_ >= 5) {
      tiller_->switchState(State::STRAFE);
  }
}



