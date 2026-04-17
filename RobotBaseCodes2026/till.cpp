#include "HardwareSerial.h"
#include "Arduino.h"
#include "till.h"
#include "tiller.h"


void Till::begin() {
  Serial.println("tilling");
  last_millis_ = millis();
  count_ = 0;

  // findYRef picks the sensor AND sets _target_y and _using_far_sensor
  _target_sensor = findYRef();

  tiller_->print("target sens: ");
  tiller_->println(_target_sensor == SIDE_SENSOR::left ? "LEFT" : "RIGHT");
  tiller_->print("using far sensor: ");
  tiller_->println(_using_far_sensor ? "YES" : "NO");
  tiller_->print("y from home wall: ");
  tiller_->print(tiller_->get_y_tgt());
  tiller_->println(" cm");
  tiller_->print("sensor setpoint: ");
  tiller_->print(_target_y);
  tiller_->println(" cm");

  tiller_->_gyro->resetAngle();
  ultrasonic_count_ = 0;

  _gyro_pid = new PID<float>(10.0, 0.0, 0.0, 0.0, false, -100.0, 100.0);
  _y_pid = new PID<float>(4.0, 0.0, 1.0, 0.0, false, -100.0, 100.0);
}

void Till::end() {
  Serial.println("stopped tilling");
}

void Till::poll() {

  float angle_control_effort;
  float y_control_effort;
  float current_y;
  float y_error;
  float last_y;
  
  float heading = tiller_->_gyro->getAngle();
  last_y = current_y;
  current_y = (_target_sensor == SIDE_SENSOR::left) ? tiller_->_side_left_ir->readSensor() : tiller_->_side_right_ir->readSensor();

  if (current_y < 0) current_y = _target_y; // fallback
  if (fabs(current_y-last_y) > 5.0);
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



Till::SIDE_SENSOR Till::findYRef(){
  // --- First call: establish which sensor faces the home wall ---
  if (tiller_->getHomeWallSensor() == -1) {
    float left_val  = tiller_->_side_left_ir->readSensorFiltered(5, 50);
    float right_val = tiller_->_side_right_ir->readSensorFiltered(5, 50);

    tiller_->print("findYRef init: L="); tiller_->print(left_val);
    tiller_->print("  R=");              tiller_->println(right_val);

    // The closer sensor is the one facing the home wall (we just homed to it)
    bool left_ok  = (left_val > 0.0);
    bool right_ok = (right_val > 0.0);

    if (left_ok && (!right_ok || left_val <= right_val)) {
      tiller_->setHomeWallSensor(0);  // left faces home wall
      tiller_->println("findYRef: home wall sensor = LEFT");
    } else {
      tiller_->setHomeWallSensor(1);  // right faces home wall
      tiller_->println("findYRef: home wall sensor = RIGHT");
    }
  }

  // --- Determine which sensor currently faces the home wall ---
  int turns = tiller_->getTurnCount();
  bool home_is_left;
  if (tiller_->getHomeWallSensor() == 0) {
    home_is_left = (turns % 2 == 0);
  } else {
    home_is_left = (turns % 2 != 0);
  }

  // --- Pick sensor based on pre-computed use_far flag ---
  // y_target is already the sensor distance (mirrored after midpoint)
  _using_far_sensor = tiller_->useFarSensor();
  _target_y = tiller_->get_y_tgt();

  SIDE_SENSOR chosen;
  if (!_using_far_sensor) {
    chosen = home_is_left ? SIDE_SENSOR::left : SIDE_SENSOR::right;
  } else {
    chosen = home_is_left ? SIDE_SENSOR::right : SIDE_SENSOR::left;
  }

  tiller_->print("findYRef: turns="); tiller_->print(turns);
  tiller_->print(" homeIsLeft=");     tiller_->print(home_is_left);
  tiller_->print(" useFar=");         tiller_->print(_using_far_sensor);
  tiller_->print(" tgt=");            tiller_->print(_target_y);
  tiller_->print(" sensor=");         tiller_->println(chosen == SIDE_SENSOR::left ? "LEFT" : "RIGHT");

  return chosen;
}
