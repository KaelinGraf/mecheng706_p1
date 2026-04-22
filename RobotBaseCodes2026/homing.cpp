#include "homing.h"
#include "Arduino.h"
#include "mappings.h"
#include "sensors.h"
#include "servo_control.h"
#include "tiller.h"

static const float TARGET_DISTANCE_CM = 18.0f; // desired distance from wall (Long Range IR blind spot is 10cm, must be >10)
static const float ALIGN_TOLERANCE_CM = 2.0f;   // front sensors equal within this
static const float FRONT_DETECT_CM = 20.0f;     // initial detection threshold
static const float US_SHORT_THRESHOLD_CM = 110.0f;  // threshold to classify short side
static const float US_DIST_CM = 105.0f;


void Homing::begin() {
  tiller_->println("Homing: begin homing");
  // Stop motors at start using the new driveMotors architecture
  tiller_->_motors->writeAllMotors(0, 0, 0); 

  if (_angle_pid) delete _angle_pid;
  if (_x_pid) delete _x_pid;
  if (_y_pid) delete _y_pid;
  if (_rotate_pid) delete _rotate_pid;
  _angle_pid = new PID<float>(12.0, 0.005, 0.0, 0.0, false, -100.0, 100.0);
  _x_pid = new PID<float>(6.0, 0.005, 0.0, 0.0, true, -100.0, 100.0);
  _y_pid = new PID<float>(8.0, 0.005, 0.0, 0.0, true, -100.0, 100.0);
  _rotate_pid = new PID<float>(140.0, 8.0, 0.0, 0.0, true, -200.0, 200.0);
  _us_phase = 0;
  _rotate_target = -1.0;

  // Reset gyro so we have a zero-heading reference for driving straight
  tiller_->_gyro->resetAngle();
}

void Homing::end() {
  tiller_->println("Homing: ended");
}

void Homing::poll(){
    float fl;
    float fr;
    float usval;
    auto forward = [&](int effort) { tiller_->_motors->writeAllMotors(effort, 0, 0); };
    auto reverse = [&](int effort) { tiller_->_motors->writeAllMotors(-effort, 0, 0); };
    auto strafeLeft = [&](int effort) { tiller_->_motors->writeAllMotors(0, -effort, 0); };
    auto strafeRight = [&](int effort) { tiller_->_motors->writeAllMotors(0, effort, 0); };
    auto rotateCW = [&](int effort) { tiller_->_motors->writeAllMotors(0, 0, -effort); };
    auto rotateCCW = [&](int effort) { tiller_->_motors->writeAllMotors(0, 0, effort); };
    auto stopDrive = [&]() { tiller_->_motors->writeAllMotors(0, 0, 0); };
    // tiller_->print("HS: ");
    // tiller_->println(_hs);
    switch (_hs) {
        case HC_DRIVE_TO_WALL:{
            // Drive in reverse with gyro heading correction to prevent arcing
            float heading = tiller_->_gyro->getAngle();
            float vtheta = _angle_pid->update(0.0 - heading); // hold zero heading
            tiller_->_motors->writeAllMotors(-100, 0, -vtheta);
            
            fl = tiller_->_rear_left_ir->readSensor();
            fr = tiller_->_rear_right_ir->readSensor();
                
            // Either sensor detecting wall is sufficient to transition;
            // the align_perp stage will handle straightening out.
            if ((fl > 0 && fl < FRONT_DETECT_CM) || (fr > 0 && fr < FRONT_DETECT_CM)) {
                stopDrive();
                _hs = HC_ALIGN_PERP;
                tiller_->println("Homing: wall detected by rear sensor");
                return;
            }
            return;
        }
        case HC_ALIGN_PERP:{
            //tiller_->print("align perp");
            // Aligning perpendicular to wall using front IR sensors
            // Takes difference of front left and right IRs to estimate angle, average for distance
            fl = tiller_->_rear_left_ir->readSensor();
            fr = tiller_->_rear_right_ir->readSensor();

            float angle_err = 0.0;
            float dist_avg;
            //if neither of the sensors returned a good value, dont update the PID 
            if (fl>0.0 |fr > 0.0){
                if (fl > 0.0 && fr > 0.0) {
                    dist_avg = (fl + fr) / 2.0;
                    angle_err = fl - fr;
                } else if (fl > 0.0) {
                    dist_avg = fl;
                } else if (fr > 0.0) {
                    dist_avg = fr;}
                

                float vx = _x_pid->update(dist_avg - TARGET_DISTANCE_CM);
                float vtheta = _angle_pid->update(angle_err);

                tiller_->_motors->writeAllMotors(-vx, 0, -vtheta);
                // tiller_->println("applying efforts (vx vy vtheta)");
                // tiller_->println(vx);
                // tiller_->println(vtheta);

            }
            // Transition when both distance and angle errors are within tolerance
            if (fl > 0.0 && fr > 0.0 &&
                fabs(dist_avg - TARGET_DISTANCE_CM) < ALIGN_TOLERANCE_CM &&
                fabs(angle_err) < 1.5) {
                tiller_->_motors->writeAllMotors(0, 0, 0);
                tiller_->_gyro->resetAngle();  // Reset gyro so rotation PID starts from 0
                if (_us_phase == 0){
                    _hs = HC_ROTATE_MOVE;
                    tiller_->println("Homing: perpendicular and at target distance, rotating to check US");
                }
                else if (_us_phase == 2){
                    tiller_->println("Homing: perpendicular and at target distance after 2nd rotate, ready to strafe");
                    _hs = HC_STRAFE_ALIGN;
                }
            }
            return;
        }
        case HC_CHECK_US:{
            // Checks ultrasonic to classify side length, transitions to either till or strafe depending on result
            tiller_->_ultrasonic->runUltrasonic();
            usval = -1.0;
            while (usval < 50.0 || usval >= 300.0){
                //read tll valid reading (must be between 50 and 300 cm, otherwise likely a spurious reading)
                usval = tiller_->_ultrasonic->readSensor();
            }
            if (usval <= US_SHORT_THRESHOLD_CM){
                tiller_->println("Homing: detected short side, till to wall");
                _hs = HC_DONE; //this starts the transition to till
            }
            else if (usval > US_SHORT_THRESHOLD_CM){
                tiller_->println("Homing: detected long side, rotating back and strafing to wall");
                _hs = HC_ROTATE_MOVE; //_us_phase equals 1 here so that it rotates the other direction in the rotate move stage
            }

            return;
        }
        case HC_ROTATE_MOVE:{
            // Rotate move depending on us_phase. Phase 0 is rotate anticlockwise, phase 1 is rotate clockwise. Transition when gyro angle reaches target.
            //first time in this state, note _rotate_target is reset to -1.0 when leaving the state too
            if (_rotate_target == -1.0) {
                 _rotate_target = (_us_phase == 0) ? PI/2 : -PI/2;
            
                tiller_->_gyro->resetAngle();
                _rotate_pid->resetPID();
            }
            float current_angle = tiller_->_gyro->getAngle();
            float error = _rotate_target - current_angle;
            float vtheta = _rotate_pid->update(error);
            tiller_->_motors->writeAllMotors(0, 0, -vtheta);
            if(fabs(error) < 0.05) {
                tiller_->_motors->writeAllMotors(0, 0, 0);
                _rotate_target = -1.0; // reset for next time
                if (_us_phase == 0) {
                    _us_phase = 1; // rotate other direction next time
                    //transition to US to check side length
                    _hs = HC_CHECK_US;
                    tiller_->println("Homing: rotation complete, checking ultrasonic");

                }
                else if (_us_phase == 1){
                    //if we had to rotate twice, this means we are on the long side and thus need to align then strafe
                    _us_phase = 2;
                    _hs = HC_ALIGN_PERP;
                    tiller_->println("Homing: rotated back, re-aligning to begin strafing");
                }
                
            }
            return;
        }

        case HC_STRAFE_ALIGN:{
            // Strafe align using front short range IR for angle and x dist, and ultrasonic for Y distance. Transition when all are within tolerance.
            // PID-based continuous 3DOF strafe alignment into the corner
            fl = tiller_->_rear_left_ir->readSensor();
            fr = tiller_->_rear_right_ir->readSensor();
            
            float dist_avg = TARGET_DISTANCE_CM;
            float angle_err = 0.0;
            if (fl > 0.0 && fr > 0.0) {
                dist_avg = (fl + fr) / 2.0;
                angle_err = fl - fr;
            }
            float vy = 0.0;
            float vx = 0.0;
            float vtheta = 0.0;
            if (fl > 0.0 && fr > 0.0) {
                vx = _x_pid->update(dist_avg - TARGET_DISTANCE_CM);
                vtheta = _angle_pid->update(angle_err);
            } else {
                // Maintain heading with gyro if somehow front sensors are lost
                float heading = tiller_->_gyro->getAngle();
                vtheta = _angle_pid->update(0.0 - heading); // 0.0 heading was zeroed in HC_ALIGN_PERP
            }
            usval = -1.0;
            while (usval < 0.0 || usval >= 300.0){
                //ensure sensible reading is read from ultrasonic (between 0 and 300 cm, otherwise likely a spurious reading)
                usval = tiller_->_ultrasonic->readSensor();
            }
            if (usval > 0.0 && usval < 300.0){
                vy = _y_pid->update(US_DIST_CM - usval);
            }
            tiller_->println("strafe printouts");
            tiller_->println(angle_err);
            tiller_->println(-vx);
            tiller_->println(usval);


            tiller_->_motors->writeAllMotors(-vx, vy, vtheta);

            if (fl > 0.0 && fr > 0.0 &&
                fabs(dist_avg - TARGET_DISTANCE_CM) <= ALIGN_TOLERANCE_CM &&
                fabs(angle_err) < 1.5 &&
                fabs(US_DIST_CM - usval) <= ALIGN_TOLERANCE_CM) {
                tiller_->_motors->writeAllMotors(0, 0, 0);
                _hs = HC_DONE;
                tiller_->println("Homing: 3DOF strafe reached target corner");
            }
            return;
        }
        
        case HC_DONE:
            stopDrive();
            if (_us_phase == 2){
                //TO BE HERE, WE NEED TO "TILL" TILL WALL WITHOUT STRAFING
                tiller_->println("Homing: tilling to wall");
                tiller_->switchState(State::TILL, {US_DIST_CM, true});
            }else{
                tiller_->println("Homing: homing complete, ready to start tiling");
                tiller_->switchState(State::TILL, {US_DIST_CM, false});
            }

            break;
    }
}