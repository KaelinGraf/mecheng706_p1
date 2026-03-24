#include "find_corner.h"
#include "Arduino.h"
#include "mappings.h"
#include "sensors.h"
#include "servo_control.h"
#include "tiller.h"

// Homing internal parameters (tunable)
static const float TARGET_DISTANCE_CM = 15.0f;  // desired distance from wall
static const float ALIGN_TOLERANCE_CM = 2.0f;   // front sensors equal within this
static const float FRONT_DETECT_CM = 30.0f;     // initial detection threshold
static const float US_SHORT_THRESHOLD_CM = 100.0f;  // threshold to classify short side

// timing constants for open-loop moves (ms) - Approximate need to be tuned.
static const unsigned long FORWARD_BURST_MS = 300;
static const unsigned long ROTATE_90_MS = 900;  // approximate rotation time for 90deg
static const unsigned long STRAFE_BURST_MS = 350;

struct HomingState {
  FindCorner::HomingStage stage = FindCorner::HC_DRIVE_TO_WALL;
  unsigned long last_millis = 0;
  int attempts = 0;
};

void FindCorner::begin() {
  tiller_->println("finding Corner...");
  // Stop motors at start using the new driveMotors architecture
  tiller_->_motors->writeAllMotors(0, 0, 0); 
  tiller_->println("FindCorner: begin homing");

  if (_angle_pid) delete _angle_pid;
  if (_x_pid) delete _x_pid;
  if (_rotate_pid) delete _rotate_pid;
  _angle_pid = new PID<float>(8.0, 0.00005, 0.0, 0.0, true, -100.0, 100.0);
  _x_pid = new PID<float>(4.0, 0.00005, 0.0, 0.0, true, -100.0, 100.0);
  _rotate_pid = new PID<float>(50.0, 0.0, 0.0, 0.0, false, -100.0, 100.0);
  _rotate_phase = 0;
}

void FindCorner::end() {
  tiller_->println("FindCorner: ended");
}

// JUST PRINT SENSOR READINGS
// void FindCorner::poll() {
//   // Use a static variable to remember the last time we printed
//   static unsigned long last_print_time = 0;
//   unsigned long current_time = millis();

//   // Check if 3000ms (3 seconds) have passed
//   if (current_time - last_print_time >= 3000) {
//     last_print_time = current_time; // Reset the timer

//     tiller_->println("--- SENSOR READINGS ---");

//     // Ultrasonic
//     float usval = tiller_->_ultrasonic->readSensor();
//     tiller_->print("Ultrasonic: ");
//     tiller_->print(usval);
//     tiller_->println(" cm");
    
//     // Short Range IR - Front Left
//     float sr_fl = tiller_->_front_left_ir->readSensor();
//     tiller_->print("SR Front Left: ");
//     tiller_->print(sr_fl);
//     tiller_->println(" cm");

//     // Short Range IR - Front Right (Fixed the sensor call and pointer typo)
//     float sr_fr = tiller_->_front_right_ir->readSensor();
//     tiller_->print("SR Front Right: ");
//     tiller_->print(sr_fr);
//     tiller_->println(" cm");

//     // Long Range IR - Side Left
//     float lr_sl = tiller_->_side_left_ir->readSensor();
//     tiller_->print("LR Side Left: ");
//     tiller_->print(lr_sl);
//     tiller_->println(" cm");

//     // Long Range IR - Side Right
//     float lr_sr = tiller_->_side_right_ir->readSensor();
//     tiller_->print("LR Side Right: ");
//     tiller_->print(lr_sr);
//     tiller_->println(" cm");

//     // Gyroscope
//     float gyroZ = tiller_->_gyro->readSensor(false);
//     tiller_->print("Gyro Z: ");
//     tiller_->println(gyroZ);
    
//     tiller_->println("-----------------------");
//   }
// }

void FindCorner::poll() {
  static HomingState hs;

  // --- lamdas for writing to motors (is this right??)
  auto forward = [&](int effort) { tiller_->_motors->writeAllMotors(effort, 0, 0); };
  auto reverse = [&](int effort) { tiller_->_motors->writeAllMotors(-effort, 0, 0); };
  auto strafeLeft = [&](int effort) { tiller_->_motors->writeAllMotors(0, -effort, 0); };
  auto strafeRight = [&](int effort) { tiller_->_motors->writeAllMotors(0, effort, 0); };
  auto rotateCW = [&](int effort) { tiller_->_motors->writeAllMotors(0, 0, -effort); };
  auto rotateCCW = [&](int effort) { tiller_->_motors->writeAllMotors(0, 0, effort); };
  auto stopDrive = [&]() { tiller_->_motors->writeAllMotors(0, 0, 0); };

  // Non-blocking-ish polling loop. Each poll advances the homing stage.
  switch (hs.stage) {
    case HC_DRIVE_TO_WALL: {
      tiller_->println("Homing: driving forward to find wall");
      forward(220); // Using lambda
      unsigned long t0 = millis();
      // drive in short bursts and sample front IRs
      while ((millis() - t0) < 3000) {
        float fl = tiller_->_front_left_ir->readSensor();
        float fr = tiller_->_front_right_ir->readSensor();
        
        if ((fl > 0 && fl < FRONT_DETECT_CM) || (fr > 0 && fr < FRONT_DETECT_CM)) {
          stopDrive();
          hs.stage = HC_ALIGN_PERP;
          hs.last_millis = millis();
          tiller_->println("Homing: wall detected by front sensor");
          return;
        }
        delay(80);
      }
      // If no wall found, stop and abort after a few attempts
      stopDrive();
      hs.attempts++;
      if (hs.attempts > 3) {
        tiller_->println("Homing: no wall found after attempts, aborting");
        hs.stage = HC_ABORT;
      }
      return;
    }

    case HC_ALIGN_PERP: {
      // PID-based continuous alignment (polled each cycle)
      float fl = tiller_->_front_left_ir->readSensor();
      float fr = tiller_->_front_right_ir->readSensor();

      float dist_avg = TARGET_DISTANCE_CM; // fallback if sensors invalid
      float angle_err = 0.0;

      if (fl > 0.0 && fr > 0.0) {
        dist_avg = (fl + fr) / 2.0;
        angle_err = fl - fr;
      } else if (fl > 0.0) {
        dist_avg = fl;
      } else if (fr > 0.0) {
        dist_avg = fr;
      }

      float vx = _x_pid->update(dist_avg - TARGET_DISTANCE_CM);
      float vtheta = _angle_pid->update(angle_err);

      tiller_->_motors->writeAllMotors(vx, 0, vtheta);

      // Transition when both distance and angle errors are within tolerance
      if (fl > 0.0 && fr > 0.0 &&
          fabs(dist_avg - TARGET_DISTANCE_CM) < ALIGN_TOLERANCE_CM &&
          fabs(angle_err) < 1.5) {
        tiller_->_motors->writeAllMotors(0, 0, 0);
        tiller_->_gyro->resetAngle();  // Reset gyro so rotation PID starts from 0
        hs.stage = HC_CHECK_US;
        tiller_->println("Homing: perpendicular and at target distance");
      }
      return;
    }

    case HC_CHECK_US: {
      tiller_->println("Homing: checking ultrasonic to classify side length");
      float usval = tiller_->_ultrasonic->readSensor();
      tiller_->print("Ultrasonic (rear): ");
      tiller_->println(usval);
      
      if (usval > 0 && usval < US_SHORT_THRESHOLD_CM) {
        tiller_->println("Homing: detected short side — will rotate to closest side IR");
        hs.stage = HC_ROTATE_MOVE;
      } else if (usval >= US_SHORT_THRESHOLD_CM) {
        tiller_->println("Homing: detected long side — will strafe to closest wall");
        hs.stage = HC_STRAFE_ALIGN;
      } else {
        tiller_->println("Homing: ultrasonic invalid, proceeding with strafe fallback");
        hs.stage = HC_STRAFE_ALIGN;
      }
      return;
    }

    case HC_ROTATE_MOVE: {
      if (_rotate_phase == 0) {
        // Phase 0: Determine direction and PID-rotate 90 degrees
        // On first entry, determine rotation direction from side IRs
        if (_rotate_target == 0.0) {
          float l = tiller_->_side_left_ir->readSensor();
          float r = tiller_->_side_right_ir->readSensor();
          bool l_valid = (l > 0 && l < 1000);
          bool r_valid = (r > 0 && r < 1000);

          // Rotate toward the closer wall
          if (l_valid && (!r_valid || l < r)) {
            _rotate_target = PI / 2.0;   // CCW 90 deg (positive = CCW)
            tiller_->println("Homing: rotating CCW toward left wall");
          } else {
            _rotate_target = -PI / 2.0;  // CW 90 deg
            tiller_->println("Homing: rotating CW toward right wall");
          }
          tiller_->_gyro->resetAngle();
          _rotate_pid->resetPID();
        }

        float current_angle = tiller_->_gyro->getAngle();
        float error = _rotate_target - current_angle;
        float vtheta = _rotate_pid->update(error);
        
        static unsigned long last_gyro_print = 0;
        if (millis() - last_gyro_print > 100) {
          tiller_->print("Gyro Angle: ");
          tiller_->print(current_angle);
          tiller_->print(" | Target: ");
          tiller_->print(_rotate_target);
          tiller_->print(" | Error: ");
          tiller_->println(error);
          last_gyro_print = millis();
        }

        tiller_->_motors->writeAllMotors(0, 0, vtheta);

        if (fabs(error) < 0.08) {  // ~4.5 degrees tolerance
          tiller_->_motors->writeAllMotors(0, 0, 0);
          _rotate_phase = 1;
          tiller_->println("Homing: rotation complete, driving to wall");
        }
      } else {
        // Phase 1: Drive forward until side sensor reads target distance
        float side = tiller_->_side_left_ir->readSensor();
        if (side <= 0) {
          side = tiller_->_side_right_ir->readSensor();
        }

        if (side > 0 && fabs(side - TARGET_DISTANCE_CM) <= ALIGN_TOLERANCE_CM) {
          tiller_->_motors->writeAllMotors(0, 0, 0);
          tiller_->println("Homing: side reached target distance");
          hs.stage = HC_DONE;
          return;
        }

        // Keep driving forward
        forward(180);
      }
      return;
    }

    case HC_STRAFE_ALIGN: {
      // PID-based continuous strafe (polled each cycle)
      float l = tiller_->_side_left_ir->readSensor();
      float r = tiller_->_side_right_ir->readSensor();
      bool l_valid = (l > 0 && l < 1000);
      bool r_valid = (r > 0 && r < 1000);

      // Maintain heading with gyro
      float heading = tiller_->_gyro->getAngle();
      float vtheta = _angle_pid->update(0.0 - heading);

      float vy = 0.0;
      bool at_target = false;

      if (l_valid && (!r_valid || l < r)) {
        // Strafe toward left wall using PID
        float y_error = TARGET_DISTANCE_CM - l;
        vy = -_x_pid->update(y_error);
        at_target = (fabs(l - TARGET_DISTANCE_CM) <= ALIGN_TOLERANCE_CM);
      } else if (r_valid) {
        // Strafe toward right wall using PID
        float y_error = TARGET_DISTANCE_CM - r;
        vy = _x_pid->update(y_error);
        at_target = (fabs(r - TARGET_DISTANCE_CM) <= ALIGN_TOLERANCE_CM);
      } else {
        // Both IRs out of range — blind strafe right until a sensor picks up
        vy = 100.0;
      }

      tiller_->_motors->writeAllMotors(0, vy, vtheta);

      if (at_target) {
        tiller_->_motors->writeAllMotors(0, 0, 0);
        hs.stage = HC_DONE;
        tiller_->println("Homing: strafe reached target distance");
      }
      return;
    }

    case HC_DONE: {
      stopDrive();
      tiller_->println("Homing complete — setting home (0,0,0)");
      tiller_->switchState(State::TILL);
      return;
    }

    case HC_ABORT: {
      stopDrive();
      tiller_->println("Homing aborted — switching to TILL");
      tiller_->switchState(State::TILL);
      return;
    }
  }

  return;
}