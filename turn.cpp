#include "Arduino.h"
#include "turn.h"
#include "tiller.h"

void Turn::begin() {
  Serial.println("turning");
  last_millis_ = millis();
  count_ = 0;
  _phase = 0;
  
  if (!_turn_pid) _turn_pid = new PID<float>(60.0, 2.0, 0.0, 0.0, true, -100.0, 100.0);
  if (!_x_pid) _x_pid = new PID<float>(4.0, 0.0, 0.0, 0.0, false, -100.0, 100.0);
  if (!_angle_pid) _angle_pid = new PID<float>(8.0, 0.0, 0.0, 0.0, false, -100.0, 100.0);
  
  tiller_->_gyro->resetAngle();
}

void Turn::end() {
  Serial.println("stopped turning");
}

void Turn::poll() {
    float vx = 0.0, vy = 0.0, vtheta = 0.0;
    
    if (_phase == 0) {
        // TURN_180 phase
        float current_angle = tiller_->_gyro->getAngle();
        float target_angle = PI; 
        float error = target_angle - fabs(current_angle);
        
        vtheta = _turn_pid->update(error);
        
        if (abs(error) < 0.1) {
            _phase = 1; 
        }
    } else if (_phase == 1) {
        // SQUARE_UP phase
        float left = tiller_->_front_left_ir->readSensor();
        float right = tiller_->_front_right_ir->readSensor();
        
        float dist_avg = 15.0;
        float angle_err = 0.0;
        if (left > 0.0 && right > 0.0) {
            dist_avg = (left + right) / 2.0;
            angle_err = left - right;
        }
        
        vx = _x_pid->update(dist_avg - 15.0); 
        vtheta = _angle_pid->update(angle_err - 0.0); 
        
        if (abs(dist_avg - 15.0) < 2.0 && abs(angle_err) < 1.5) {
             tiller_->_gyro->resetAngle(); 
             tiller_->switchState(State::STRAFE);
             return;
        }
    }
    
    tiller_->_motors->writeAllMotors(vx, vy, vtheta);
}