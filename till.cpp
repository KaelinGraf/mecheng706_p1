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
  const int N_SAMPLES = 10;
  const int DELAY_MS  = 5;  // ms between samples

  float left_samples[N_SAMPLES];
  float right_samples[N_SAMPLES];
  int   left_valid  = 0;
  int   right_valid = 0;

  // Collect multiple readings from both sensors
  for (int i = 0; i < N_SAMPLES; i++) {
    float l = tiller_->_side_left_ir->readSensor();
    float r = tiller_->_side_right_ir->readSensor();

    if (l > 0.0) { left_samples[left_valid++]   = l; }
    if (r > 0.0) { right_samples[right_valid++] = r; }

    delay(DELAY_MS);
  }

  tiller_->print("findYRef: left valid=");  tiller_->print(left_valid);
  tiller_->print("  right valid=");         tiller_->println(right_valid);

  // If one sensor has too few valid readings, use the other
  const int MIN_VALID = N_SAMPLES / 2;  // need at least half valid
  bool left_ok  = (left_valid  >= MIN_VALID);
  bool right_ok = (right_valid >= MIN_VALID);

  if (!left_ok && !right_ok) {
    tiller_->println("findYRef: both sensors unreliable, defaulting left");
    return SIDE_SENSOR::left;
  }
  if (!left_ok)  return SIDE_SENSOR::right;
  if (!right_ok) return SIDE_SENSOR::left;

  // Compute mean and standard deviation for each sensor
  auto computeStats = [](float* samples, int n, float &mean, float &stddev) {
    float sum = 0.0;
    for (int i = 0; i < n; i++) sum += samples[i];
    mean = sum / n;

    float sq_sum = 0.0;
    for (int i = 0; i < n; i++) {
      float diff = samples[i] - mean;
      sq_sum += diff * diff;
    }
    stddev = sqrt(sq_sum / n);
  };

  float left_mean, left_std, right_mean, right_std;
  computeStats(left_samples,  left_valid,  left_mean,  left_std);
  computeStats(right_samples, right_valid, right_mean, right_std);

  tiller_->print("findYRef: L mean="); tiller_->print(left_mean);
  tiller_->print(" std=");             tiller_->print(left_std);
  tiller_->print("  R mean=");         tiller_->print(right_mean);
  tiller_->print(" std=");             tiller_->println(right_std);

  // Choose the sensor with lower noise (standard deviation)
  if (left_std < right_std) {
    tiller_->println("findYRef: chose LEFT (lower noise)");
    return SIDE_SENSOR::left;
  } else if (right_std < left_std) {
    tiller_->println("findYRef: chose RIGHT (lower noise)");
    return SIDE_SENSOR::right;
  } else {
    // Equal noise — pick whichever is closer (more reliable at short range)
    tiller_->println("findYRef: equal noise, choosing by distance");
    return (left_mean <= right_mean) ? SIDE_SENSOR::left : SIDE_SENSOR::right;
  }
}

