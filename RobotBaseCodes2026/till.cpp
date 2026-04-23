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
  tiller_->print("From homing? ");
  tiller_->println(data.from_homing?"yes":"no");
  tiller_->print("turn count: "); tiller_->println(tiller_->getTurnCount());
  if(data.from_homing){
    tiller_->_gyro->resetAngle();

  }
  tiller_->_ultrasonic->resetBuffer();
  tiller_->_front_left_ir->resetBuffer();
  tiller_->_front_right_ir->resetBuffer();
  tiller_->_rear_left_ir->resetBuffer();
  tiller_->_rear_right_ir->resetBuffer();
  tilling_speed_ = data.drive_foward ? TILL_SPEED : -TILL_SPEED; // move between driving foward and backward
  
  _from_homing = data.from_homing;

  
  // if (tiller_->getTurnCount() % 2 ==0) {
  //   for (int i = 0; i < 2; i++) {
  //     tiller_->_front_left_ir->getAvg();
  //     tiller_->_front_right_ir->getAvg();
  //   }
  // } else {
  //   for (int i = 0; i < 2; i++) {
  //     tiller_->_rear_left_ir->getAvg();
  //     tiller_->_rear_right_ir->getAvg();
  //   }
  // }tiller_->_ultrasonic->getAvg();
  // tiller_->_ultrasonic->getAvg();
  endzone_count_ = 0;

  _gyro_pid = new PID<float>(300.0, 25.0, 0.0, 0.0, true, -60.0, 60.0); 
  _y_pid = new PID<float>(6.0, 0.5, 0.0, 0.0, true, -100.0, 100.0);
}

void Till::end() {
  tiller_->println("stopped tilling");
  tiller_->print("Y before inc: ");
  tiller_->println(tiller_->get_y_tgt());
  if(!_from_homing){
    tiller_->incTurnCount();
  }
  tiller_->print("Y after inc: ");
  tiller_->println(tiller_->get_y_tgt());

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

  // tiller_->print("dist error: "); tiller_->println(y_error);
  // tiller_->print("heading: ");    tiller_->println(heading);
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

  float u_dist = (left_dist + right_dist) / 2;

  if (u_dist > 0.0 && u_dist < 8.0 && left_dist > 0 && right_dist > 0) {
    endzone_count_++;
  } else {
    endzone_count_ = 0;
  }

  if (endzone_count_ >= 5) {
      tiller_->_motors->writeAllMotors(0.0,0.0,0.0);

      if (tiller_->isLastRun()) {
        tiller_->switchState(State::STOPPED);
      } else if(_from_homing){
        tiller_->switchState(State::TILL,{_target_y,true,false});
      }else{
        tiller_->switchState(State::STRAFE);
      }
  }
}



