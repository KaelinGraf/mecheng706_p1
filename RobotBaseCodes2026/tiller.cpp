#include "Arduino.h"
#include "HardwareSerial.h"
#include "tiller.h"
#include "homing.h"
#include "till.h"
#include "turn.h"
#include "strafe.h"
#include "initialising.h"

volatile Ultrasonic* ultrasonicISR = nullptr;
ISR(INT4_vect) {
  if (!ultrasonicISR) {
    Serial.println("error");
    return;} // safety check
  // Check if the pin is HIGH (Rising Edge)
  if (digitalRead(2)) {
    unsigned long last_t1 = ultrasonicISR->getSentTime();
    ultrasonicISR->setLastSent(last_t1);
    ultrasonicISR->setSentTime(micros());
  } 
  // If it's not HIGH, it must be LOW (Falling Edge)
  else {
    ultrasonicISR->setReturnTime(micros());
  }
}

Tiller::Tiller(Adafruit_BNO08x* bno08x,sh2_SensorValue_t* sensorValue,HardwareSerial* SerialCom) {
  serialCom_ = SerialCom;

  // Generate y targets as sensor distances (mirrored after midpoint)
  // First half: increasing distance from home wall (use home sensor)
  // Second half: increasing distance from far wall, stored as decreasing (use far sensor)
  float first = Y_MARGIN;
  float last  = Y_WIDTH - Y_MARGIN;
  float abs_positions[NUM_SNAKES];
  for (int i = 0; i < NUM_SNAKES; i++) {
    abs_positions[i] = first + i * (last - first) / (NUM_SNAKES - 1);
  }
  for (int i = 0; i < NUM_SNAKES; i++) {
    if (abs_positions[i] <= Y_WIDTH / 2.0) {
      y_tgts_[i] = abs_positions[i];              // distance from home wall
      use_far_[i] = false;
    } else {
      y_tgts_[i] = Y_WIDTH - abs_positions[i];    // distance from far wall
      use_far_[i] = true;
    }
  }
  curr_y_idx_ = 0;
  turn_count_ = 0;
  home_wall_sensor_ = -1;

  // Print generated targets
  SerialCom->println("Y targets (sensor distances):");
  for (int i = 0; i < NUM_SNAKES; i++) {
    SerialCom->print("  ["); SerialCom->print(i); SerialCom->print("] ");
    SerialCom->print(y_tgts_[i]); SerialCom->print(" cm (");
    SerialCom->print(use_far_[i] ? "far" : "home");
    SerialCom->println(" sensor)");
  }

  // Initialise hardware first, so states can safely access sensors/motors
  _gyro = new Gyroscope(bno08x, sensorValue, SerialCom);
  _motors = new driveMotors();
  _front_left_ir = new LongRangeIR(front_left_ir_pin);
  _front_right_ir = new LongRangeIR(front_right_ir_pin);
  _rear_left_ir = new ShortRangeIR(rear_left_ir_pin);
  _rear_right_ir = new ShortRangeIR(rear_right_ir_pin);
  _ultrasonic = new Ultrasonic(ECHO_PIN, TRIG_PIN, MAX_DIST);
  ultrasonicISR = _ultrasonic;
  _motors->attatchAll();

  // Initialise all states, prevents memory management issues
  states_[State::INITIALISING] = new Initialising(this);
  states_[State::HOMING]  = new Homing(this);
  states_[State::TILL]         = new Till(this);
  states_[State::TURN]         = new Turn(this);
  states_[State::STRAFE]       = new Strafe(this);

  // Begin initial state AFTER all hardware and states are ready
  current_state_ = states_[State::INITIALISING];
  current_state_->begin();
}

bool Tiller::switchState(State::Name newState, TillData data = {-1.0, false}) {
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
      if (data.distance != -1.0){
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
  _gyro->readSensor();
  if (current_state_) {
    current_state_->poll();
  }
};

void Tiller::testSensors(){
  print("gyro: rad, itegral");
  println(_gyro->readSensor());
  println(_gyro->getAngle());

  print("ultrasonic");
  println(_ultrasonic->readSensor());

  println("IR senors:");
  print("Front Left:  "); println(_front_left_ir->readSensor());
  print("Front Right: "); println(_front_right_ir->readSensor());
  print("Rear Left:   "); println(_rear_left_ir->readSensor());
  print("Rear Right:  "); println(_rear_right_ir->readSensor());
  delay(200);

}

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