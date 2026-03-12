#include <stdint.h>
#include <Servo.h>
#include "mappings.h"
const int left_front_multis[3] = {1,1,-1};
const int left_rear_multis[3] = {1,-1,-1};
const int right_front_multis[3] = {1,-1,1};
const int right_rear_multis[3] = {1,1,1};
class Motor;
// Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
// Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
// Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
// Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
// Servo turret_motor;      // create servo object to control turret servo


enum motorName{
  left_font,
  left_rear,
  right_rear,
  right_front,
};
class Motor{
  private:
    Servo _motor;
    const int* _control_multipliers; //array of i.e [1,-1,1] for front right motor (multipliers for control effort Vx Vy Vtheta)


  public:
    Motor(const int* control_multipliers,uint8_t motor_pin):_control_multipliers(control_multipliers){
      _motor.attach(motor_pin);
    }
    Motor(uint8_t motor_pin){
      _motor.attach(motor_pin);
    }
    ~Motor(){
      _motor.detach();
    }
    void writeMotor(uint16_t microseconds);
    void writeMotor(uint16_t vx, uint16_t vy, uint16_t vtheta);
    void stopMotor();
};

class driveMotors{
  private:
    Motor _left_font_motor;
    Motor _left_rear_motor;
    Motor _right_front_motor;
    Motor _right_rear_motor;

  public:
    driveMotors():_left_font_motor(left_front_multis, left_front_pin),_left_rear_motor(left_rear_multis, left_rear_pin),_right_front_motor(right_front_multis, right_front_pin),_right_rear_motor(right_rear_multis, right_rear_pin){
      //_left_font_motor = Motor(left_front_multis, left_front_pin);
      // _left_rear_motor = Motor(left_rear_multis, left_rear_pin);
      // _right_rear_motor = Motor(right_rear_multis, right_rear_pin);
      // _right_front_motor = Motor(right_front_multis, right_front_pin);
    }
    writeMotor(motorName target_motor, uint16_t speed);

};




class turret : public Motor{
  public:
    turret(uint8_t motor_pin):Motor(motor_pin){};
    

};