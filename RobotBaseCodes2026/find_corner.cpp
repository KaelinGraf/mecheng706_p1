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
  tiller_->println("FindCorner: begin homing (open-loop)");
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
  auto rotateCW = [&](int effort) { tiller_->_motors->writeAllMotors(0, 0, effort); };
  auto rotateCCW = [&](int effort) { tiller_->_motors->writeAllMotors(0, 0, -effort); };
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
      tiller_->println("Homing: aligning perpendicular to wall");
      for (int i = 0; i < 6; ++i) {
        float fl = tiller_->_front_left_ir->readSensor();
        float fr = tiller_->_front_right_ir->readSensor();
        
        tiller_->print("FL=");
        tiller_->print(fl);
        tiller_->print(" FR=");
        tiller_->println(fr);
        
        if (fl > 0 && fr > 0) {
          float diff = fl - fr;
          if (fabs(diff) <= ALIGN_TOLERANCE_CM) {
            // both roughly equal — move to target distance
            if (fl > TARGET_DISTANCE_CM + ALIGN_TOLERANCE_CM) {
              forward(180);
              delay(FORWARD_BURST_MS);
              stopDrive();
            } else if (fl < TARGET_DISTANCE_CM - ALIGN_TOLERANCE_CM) {
              reverse(180);
              delay(FORWARD_BURST_MS);
              stopDrive();
            }
            // check again
            float fl2 = tiller_->_front_left_ir->readSensor();
            if (fl2 > 0 && fabs(fl2 - TARGET_DISTANCE_CM) <= ALIGN_TOLERANCE_CM) {
              hs.stage = HC_CHECK_US;
              tiller_->println("Homing: perpendicular and at target distance");
              return;
            }
          } else {
            // rotate a small amount toward the nearer side
            if (diff > 0) {
              rotateCCW(160);
              delay(200);
              stopDrive();
            } else {
              rotateCW(160);
              delay(200);
              stopDrive();
            }
          }
        } else {
          // If one sensor invalid, try small forward burst to get a reading
          forward(150);
          delay(200);
          stopDrive();
        }
        delay(150);
      }
      // If alignment attempts failed, abort
      tiller_->println("Homing: alignment attempts exceeded, aborting");
      hs.stage = HC_ABORT;
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
      tiller_->println("Homing: choosing side (left/right) using long-range IRs");
      float l = tiller_->_side_left_ir->readSensor();
      float r = tiller_->_side_right_ir->readSensor();
      
      tiller_->print("Side L=");
      tiller_->print(l);
      tiller_->print(" R=");
      tiller_->println(r);
      
      bool l_valid = (l > 0 && l < 1000); 
      bool r_valid = (r > 0 && r < 1000); 
      
      if (!l_valid && !r_valid) {
        rotateCW(200);
        delay(ROTATE_90_MS);
        stopDrive();
      } else {
        if (l_valid && (!r_valid || l < r)) {
          rotateCCW(200);
          delay(ROTATE_90_MS);
          stopDrive();
        } else {
          rotateCW(200);
          delay(ROTATE_90_MS);
          stopDrive();
        }
      }
      
      tiller_->println("Homing: moving toward chosen wall");
      unsigned long tstart = millis();
      while ((millis() - tstart) < 8000) {
        forward(200);
        delay(300);
        stopDrive();
        
        float side = tiller_->_side_left_ir->readSensor();
        if (side <= 0) {
          side = tiller_->_side_right_ir->readSensor(); 
        }
                                              
        if (side > 0 && fabs(side - TARGET_DISTANCE_CM) <= ALIGN_TOLERANCE_CM) {
          tiller_->println("Homing: side reached target distance");
          hs.stage = HC_DONE;
          return;
        }
      }
      tiller_->println("Homing: timed out moving to side wall, aborting");
      hs.stage = HC_ABORT;
      return;
    }

    case HC_STRAFE_ALIGN: {
      tiller_->println("Homing: strafing to closest wall (long-side case)");
      float l = tiller_->_side_left_ir->readSensor();
      float r = tiller_->_side_right_ir->readSensor();
      
      bool l_valid = (l > 0 && l < 1000); 
      bool r_valid = (r > 0 && r < 1000); 
      
      if (l_valid && (!r_valid || l < r)) {
        unsigned long t0 = millis();
        while ((millis() - t0) < 6000) {
          strafeLeft(200);
          delay(STRAFE_BURST_MS);
          stopDrive();
          float v = tiller_->_side_left_ir->readSensor();
          if (v > 0 && fabs(v - TARGET_DISTANCE_CM) <= ALIGN_TOLERANCE_CM) {
            hs.stage = HC_DONE;
            return;
          }
        }
      } else if (r_valid) {
        unsigned long t0 = millis();
        while ((millis() - t0) < 6000) {
          strafeRight(200);
          delay(STRAFE_BURST_MS);
          stopDrive();
          float v = tiller_->_side_right_ir->readSensor();
          if (v > 0 && fabs(v - TARGET_DISTANCE_CM) <= ALIGN_TOLERANCE_CM) {
            hs.stage = HC_DONE;
            return;
          }
        }
      } else {
        tiller_->println("Homing: no valid side IRs, assuming done");
        hs.stage = HC_DONE;
        return;
      }
      tiller_->println("Homing: strafe timed out, aborting");
      hs.stage = HC_ABORT;
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