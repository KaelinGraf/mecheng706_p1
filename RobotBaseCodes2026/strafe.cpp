#include "HardwareSerial.h"
#include "Arduino.h"
#include "strafe.h"
#include "tiller.h"
#include "till.h" // For TILL state transition

void Strafe::begin() {
  tiller_->_ultrasonic->runUltrasonic();
  tiller_->println("strafing");
  tiller_->print("turn count: "); tiller_->println(tiller_->getTurnCount());
  
  _target_y = tiller_->get_y_tgt();
  tiller_->print("target y");
  tiller_->println(_target_y);


  if (tiller_->getTurnCount() % 2 ==1) {
    for (int i = 0; i < 2; i++) {
      tiller_->_front_left_ir->resetBuffer();
      tiller_->_front_right_ir->resetBuffer();
    }
  } else {
    for (int i = 0; i < 2; i++) {
      tiller_->_rear_left_ir->resetBuffer();
      tiller_->_rear_right_ir->resetBuffer();
    }
    
  }tiller_->_ultrasonic->resetBuffer();
  _gyro_pid->resetPID();
  _angle_pid->resetPID();
  _x_pid->resetPID();
  _y_pid->resetPID(); 

  // tiller_->print("Strafe: setpoint="); tiller_->print(_target_y);
  // tiller_->print(" distance=");          tiller_->print(tiller_->_ultrasonic->readSensor());
  // tiller_->print(" sensor= ");          tiller_->print("ULTRASONIC");
  // tiller_->print(" turn=");          tiller_->println(tiller_->getTurnCount());
}

void Strafe::end() {
  Serial.println("stopped strafing");
  tiller_->print("Ultrasonic: ");
  tiller_->println(tiller_->_ultrasonic->getAvg());
}

void Strafe::poll(){  
  float ultrasonic = tiller_->_ultrasonic->readBlocking();
  float wall_dist = (tiller_->getTurnCount() % 2 ==0) ? 5.0:14.0;
  float dist_val_left = (tiller_->getTurnCount() % 2 ==1) ? tiller_->_front_left_ir->getAvg() : tiller_->_rear_left_ir->getAvg();
  float dist_val_right = (tiller_->getTurnCount() % 2 == 1) ? tiller_->_front_right_ir->getAvg() : tiller_->_rear_right_ir->getAvg();

  //tiller_->print("left dist:  "); tiller_->println(dist_val_left);
  //tiller_->print("right dist: "); tiller_->println(dist_val_right);

  if(ultrasonic > 0 && dist_val_left > 0 && dist_val_right > 0){
    float angle = tiller_->_gyro->getAngle();
    float vtheta = _gyro_pid->update(angle);
    float vy = _y_pid->update((_target_y - ultrasonic));  // CHANGED flipped sign
    float vx = _x_pid->update(((dist_val_left + dist_val_right)/2.0)-wall_dist);
    if(tiller_->getTurnCount() % 2 !=1){
      vx *= -1.0;
    }
    tiller_->_motors->writeAllMotors(vx, vy, vtheta);
  }

  tiller_->print("y err:  "); tiller_->println(_target_y - ultrasonic);
  tiller_->print("Ultrasonic"); tiller_->println(ultrasonic);
  
  if(
    (fabs((ultrasonic-_target_y)) <= 2.0) &&
    (fabs(((dist_val_left + dist_val_right) / 2.0) - wall_dist) <= 2.0)
  ){
    exit_count ++;
  }else{
    exit_count = 0;
  }
  if(exit_count >= 3){
    tiller_->println("STRAFE FINISH");
    tiller_->switchState(State::TILL, {_target_y,tiller_->getTurnCount() % 2 !=1, false});
  }

}


