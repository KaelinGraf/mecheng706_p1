#include"sensors.h"


float Sensor::readSensor(){
  return applyCalibration(float(analogRead(_read_pin)));
}
void Sensor::setReadPin(uint8_t new_pin){
  _read_pin = new_pin;
}
uint8_t Sensor::getReadPin(){
  return _read_pin;
}

Sensor::Sensor(uint8_t read_pin){
  _read_pin = read_pin;
}

Ultrasonic::Ultrasonic(uint8_t echo_pin, uint8_t trigger_pin, uint8_t max_dist, HardwareSerial* SerialCom) 
: Sensor(uint8_t(255)),  _echo_pin(echo_pin), _trigger_pin(trigger_pin), _max_dist(max_dist),_serial_com(SerialCom){
};
float Ultrasonic::readSensor() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pin, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(_echo_pin) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (_max_dist + 1000)) {
      if (DIAGNOSTICS){
        _serial_com->println("HC-SR04: NOT found");
      }
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(_echo_pin) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (_max_dist + 1000)) {
      if (DIAGNOSTICS){
        _serial_com->println("HC-SR04: Out of range");
      }
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (DIAGNOSTICS){
    if (pulse_width > _max_dist) {
      _serial_com->println("HC-SR04: Out of range");
    } else {
      _serial_com->print("HC-SR04:");
      _serial_com->print(cm);
      _serial_com->println("cm");
    }
  }
  return cm;
};




