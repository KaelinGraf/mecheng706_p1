#include "HardwareSerial.h"
#include "Arduino.h"
#include "strafe.h"
#include "tiller.h"
#include "till.h" // For TILL state transition

void Strafe::begin() {
  tiller_->_ultrasonic->runUltrasonic();
  tiller_->println("strafing");
  tiller_->inc_y_tgt();
  
  _target_y = tiller_->get_y_tgt();

  tiller_->print("Strafe: setpoint="); tiller_->print(_target_y);
  tiller_->print(" distance=");          tiller_->print(tiller_->_ultrasonic->readSensor());
  tiller_->print(" sensor= ");          tiller_->print("ULTRASONIC");
  tiller_->print(" turn=");          tiller_->println(tiller_->getTurnCount());
}

void Strafe::end() {
  Serial.println("stopped strafing");
}

void Strafe::poll() {

  float ultrasonic = tiller_->_ultrasonic->readSensor();
  
  float dist_avg = 15.0;
  float angle_err = 0.0;
  float wall_dist = (tiller_->getTurnCount() % 2 ==0) ? 20.0:5.0;
  float dist_val_left = -1.0;
  float dist_val_right = -1.0;
  int count = 0;

  //loop till 2 valid readings
  while((dist_val_left == -1.0 || dist_val_right == -1.0) || count < 40){
    dist_val_left = (tiller_->getTurnCount() % 2 ==0)?tiller_->_front_left_ir->readSensor() : tiller_->_rear_left_ir->readSensor();
    dist_val_right = (tiller_->getTurnCount() % 2 == 0)?tiller_->_front_right_ir->readSensor() : tiller_->_rear_right_ir->readSensor();
    count ++;
  }
  if(dist_val_left != -1.0 && dist_val_right != -1.0) {
    dist_avg = (dist_val_left + dist_val_right)/2.0;
    angle_err = (dist_val_right - dist_val_left);
  }
  
  float vx = _x_pid->update(wall_dist - dist_avg);
  float vtheta = _angle_pid->update(angle_err);
  
  float y_error = 0.0;
  float vy = 0.0;
  
  if (ultrasonic > 0.0) {
    y_error = ultrasonic - _target_y;
    vy = _y_pid->update(-y_error);   // ultrasonic on left → strafe left
  } else {
    // Sensor invalid — blind strafe toward the target wall
    vy = 50.0;
  }

  vy = 0.0;

  // Diagnostics (throttled to avoid flooding serial)
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    tiller_->print("Strafe: side=");    tiller_->print("ULTRASONIC on left side");
    tiller_->print(" tgt=");            tiller_->print(_target_y);
    tiller_->print(" err=");            tiller_->print(y_error);
    tiller_->print(" vy=");             tiller_->print(vy);
    tiller_->print(" vx=");             tiller_->println(vx);
    last_print = millis();
  }
  
  tiller_->_motors->writeAllMotors(vx, vy, vtheta);
  
  // Check if we hit row target distance
  if (ultrasonic > 0.0 && fabs(y_error) < 2.0) {
      tiller_->println("Strafe: target reached, switching to TILL");
      tiller_->_motors->writeAllMotors(0,0,0);
      //tiller_->switchState(State::TILL, {_target_y, false});
  }
}


