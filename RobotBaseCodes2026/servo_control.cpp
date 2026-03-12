#include "servo_control.h"

void Motor::writeMotor(uint16_t microseconds){
  _motor.writeMicroseconds(int(clip<uint16_t>(microseconds,min_duty_motor,max_duty_motor)));
}

void Motor::writeMotor(uint16_t vx, uint16_t vy, uint16_t vtheta){
  uint16_t val = vx * _control_multipliers[0] + vy * _control_multipliers[1] + vtheta * _control_multipliers[2];
  _motor.writeMicroseconds(int(clip<uint16_t>(val,min_duty_motor,max_duty_motor)));
}

void turret::writeMotor(int angle){
  _motor.write(angle);
}