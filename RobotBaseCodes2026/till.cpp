#include "HardwareSerial.h"
#include "Arduino.h"
#include "till.h"
#include "tiller.h"


void Till::begin(void* target_y) {
  Serial.println("tilling");
  last_millis_ = millis();
  count_ = 0;
  _target_sensor= findYRef();
  _target_y = (static_cast<float*>(target_y));
  _gyro_pid = new PID<uint16_t>(gyro_omega_0_kp,gyro_omega_0_ki,gyro_omega_0_kd,0.0,false,-100.0,100.0);
  _y_pid = new PID<uint16_t>(y_dist_pid_kp,y_dist_pid_ki,y_dist_pid_kd, float(neutral),false,-100.0,100.0);
  
}

void Till::end() {
  Serial.println("stopped tilling");
}

void Till::poll() {
  uint16_t angle_control_effort;
  uint16_t y_control_effort;
  float error;
  float omega;
  float current_y;
  float y_error;
  omega = tiller_->_gyro->readSensor();
  current_y =  (_target_sensor == SIDE_SENSOR::left)? tiller_->_side_left_ir->readSensor() : tiller_->_side_right_ir->readSensor();
  y_error = *_target_y - current_y;

  angle_control_effort = _gyro_pid->update(omega);
  y_control_effort = _y_pid->update(y_error);
  tiller_->_motors->writeAllMotors(0, y_control_effort, angle_control_effort);
  
  // if (count_ > 30) {
  //   tiller_->switchState(State::TURN);
  // }
}



enum Till::SIDE_SENSOR Till::findYRef(){
  float left_ir_read = tiller_->_side_left_ir->readSensor();
  float right_ir_read = tiller_->_side_right_ir->readSensor();
  if (left_ir_read == -1.0){
    return SIDE_SENSOR::right;
  }
  else if (right_ir_read == -1.0 ){
    return SIDE_SENSOR::left;
  }
  else{
    return (left_ir_read <= right_ir_read) ? SIDE_SENSOR::left : SIDE_SENSOR::right;
  }
}

