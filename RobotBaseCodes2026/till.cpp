#include "Arduino.h"
#include "till.h"
#include "tiller.h"

void Till::begin() {
  Serial.println("tilling");
  last_millis_ = millis();
  count_ = 0;
  _target_sensor= findYRef();
  _target_y = (_target_sensor == SIDE_SENSOR::left)? tiller_->_side_left_ir->readSensor() : tiller_->_side_right_ir->readSensor();
  _gyro_pid = new PID<uint16_t>(gyro_omega_0_kp,gyro_omega_0_ki,gyro_omega_0_kd,float(neutral),false,min_duty_motor,max_duty_motor,neutral);
  
}

void Till::end() {
  Serial.println("stopped tilling");
}

void Till::poll() {
  uint16_t angle_control_effort;
  float error;
  float omega;
  omega = tiller_->_gyro->readSensor();
  angle_control_effort = _gyro_pid->update(omega);
  



  if (count_ > 30) {
    tiller_->switchState(State::TURN);
  }
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

