#include "HardwareSerial.h"
#include "Arduino.h"
#include "till.h"
#include "tiller.h"


void Till::begin() {
  Serial.println("tilling");
  last_millis_ = millis();
  count_ = 0;
  _target_sensor= findYRef();

  tiller_->print("target sens: ");
  tiller_->println(_target_sensor);

  _target_y = tiller_->get_y_tgt();
  tiller_->print("target: ");
  tiller_->print(_target_y);
  tiller_->println("cm");

  tiller_->_gyro->resetAngle();
  ultrasonic_count_ = 0;

  _gyro_pid = new PID<float>(50.0, 0.0, 0.0, 0.0, false, -100.0, 100.0);
  _y_pid = new PID<float>(10.0, 0.0, 0.0, 0.0, false, -150.0, 150.0);
  
}

void Till::end() {
  Serial.println("stopped tilling");
}

void Till::poll() {

  float angle_control_effort;
  float y_control_effort;
  float current_y;
  float y_error;
  
  float heading = tiller_->_gyro->getAngle();

  current_y = (_target_sensor == SIDE_SENSOR::left) ? tiller_->_side_left_ir->readSensor() : tiller_->_side_right_ir->readSensor();
  if (current_y < 0) current_y = _target_y; // fallback

  y_error = _target_y - current_y;

  angle_control_effort = _gyro_pid->update(0.0 - heading);
  
  if (_target_sensor == SIDE_SENSOR::left) {
      y_control_effort = _y_pid->update(y_error); // move right to increase distance to left wall
  } else {
      y_control_effort = -_y_pid->update(y_error); // move left to increase distance to right wall
  }
  
  float vx = -50.0; // Constant backward speed
  tiller_->_motors->writeAllMotors(vx, y_control_effort, angle_control_effort);
  
  float u_dist = tiller_->_ultrasonic->readSensor();
  if (u_dist > 0.0 && u_dist < 30.0) {
    tiller_->print("ultrasonic dist");

    tiller_->print(u_dist);
      ultrasonic_count_++;
  } else {
      ultrasonic_count_ = 0;
  }
  
  if (ultrasonic_count_ >= 5) {
      tiller_->switchState(State::TURN);
  }
}



Till::SIDE_SENSOR Till::findYRef(){
  float left_ir_read = tiller_->_side_left_ir->readSensor();
  float right_ir_read = tiller_->_side_right_ir->readSensor();

  if (left_ir_read <= 0.0){
    return SIDE_SENSOR::right;
  }
  else if (right_ir_read <= 0.0 ){
    return SIDE_SENSOR::left;
  }
  else{
    // TODO we want to check both out of range case first
    return (left_ir_read <= right_ir_read) ? SIDE_SENSOR::left : SIDE_SENSOR::right;
  }
}

