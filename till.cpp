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
  _y_pid = new PID<float>(20.0, 0.0, 0.0, 0.0, false, -100.0, 100.0);

  // kalman stuff here
  float initial_y = (_target_sensor == SIDE_SENSOR::left) ? tiller_->_side_left_ir->readSensor() : tiller_->_side_right_ir->readSensor();
  _prev_y_est = (initial_y > 0.0) ? initial_y : _target_y;
  _last_y_var = 1.0;
  _last_y_millis = millis();
}

void Till::end() {
  Serial.println("stopped tilling");
}

void Till::poll() {

  float angle_control_effort;
  float y_control_effort;
  float current_y;
  float y_error;
  bool useKalmanFilter = true;
  
  float heading = tiller_->_gyro->getAngle();

  current_y = getYDist(useKalmanFilter);

  y_error = _target_y - current_y;

  angle_control_effort = _gyro_pid->update(0.0 + heading);
  
  if (_target_sensor == SIDE_SENSOR::left) {
      y_control_effort = -_y_pid->update(y_error); // move right (negative Vy) to increase distance to left wall
  } else {
      y_control_effort = _y_pid->update(y_error); // move left (positive Vy) to increase distance to right wall
  }
  
  float vx = -50.0; // Constant backward speed (dominant axis)
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

float Till::getFilteredY() {
  // Get raw reading based on active target sensor
  float raw_data = (_target_sensor == SIDE_SENSOR::left) ? tiller_->_side_left_ir->readSensor() : tiller_->_side_right_ir->readSensor();

  // Handle out-of-range fallback
  if (raw_data < 0.0) {
    _last_y_millis = millis();
    return _prev_y_est;
  }

  // Prediction
  float dt = (millis() - _last_y_millis) / 1000.0;
  float a_priori_est = _prev_y_est; 
  float a_priori_var = _last_y_var + (process_noise_ * dt);

  // Update
  float kalman_gain = a_priori_var / (a_priori_var + sensor_noise_);
  float a_post_est = a_priori_est + kalman_gain * (raw_data - a_priori_est);
  float a_post_var = (1.0 - kalman_gain) * a_priori_var;

  _prev_y_est = a_post_est;
  _last_y_var = a_post_var;
  _last_y_millis = millis();
  
  return _prev_y_est;
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

