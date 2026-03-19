#include "HardwareSerial.h"
#include "tiller.h"
#include "find_corner.h"
#include "till.h"
#include "turn.h"
#include "initialising.h"

Tiller::Tiller(Adafruit_BNO08x* bno08x,sh2_SensorValue_t* sensorValue,HardwareSerial* SerialCom):_front_left_ir(front_left_ir_pin),_front_right_ir(front_right_ir_pin),_side_left_ir(side_left_ir_pin),_side_right_ir(side_right_ir_pin) {
  current_state_= new Initialising(this);
  serialCom_ = SerialCom;
  current_state_->begin();

  // initilise all states, prevents memory managment issues
  states_[State::INITIALISING] = new Initialising(this);
  states_[State::FIND_CORNER]  = new FindCorner(this);
  states_[State::TILL]         = new Till(this);
  states_[State::TURN]         = new Turn(this);

  _gyro = new Gyroscope(bno08x, sensorValue, SerialCom);
}

bool Tiller::switchState(State::Name newState, void* data = nullptr) {
// 1. End the current state
    if (current_state_) {
        current_state_->end();
    }

    if (!is_battery_voltage_OK()) {
      // TODO enter fault
      current_state_ = states_[State::INITIALISING];
      this->println();
      this->println("ERROR: BATTERY NOT OK");
      this->println();
      return false;
    }

    // 2. Look up the new state
    current_state_ = states_[newState];

    // 3. Begin the new state
    if (current_state_) {
      if (data != nullptr){
        current_state_->begin(data);
        return true;
      }
      else{
        current_state_->begin();
        return true;

      }
    }

    this->print("unknown state");
    return false;
}

void Tiller::pollState() {
  if (current_state_) {
    current_state_->poll();
  }
};

bool Tiller::is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  // the voltage of a LiPo cell depends on its chemistry and varies from
  // about 3.5V (discharged) = 717(3.5V Min)
  // https://oscarliang.com/lipo-battery-guide/ to about 4.20-4.25V (fully
  // charged) = 860(4.2V Max) Lipo Cell voltage should never go below 3V,
  // So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    this->print("Lipo level:");
    this->print(Lipo_level_cal);
    this->print("%");
    this->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      this->println(
          "Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      this->println("!Lipo is Overchanged!!!");
    else {
      this->println(
          "Lipo voltage too LOW, any lower and the lipo with be damaged");
      this->print("Please Re-charge Lipo:");
      this->print(Lipo_level_cal);
      this->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}