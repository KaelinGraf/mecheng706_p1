#include "Arduino.h"
#include"sensors.h"
#include <avr/interrupt.h>

Ultrasonic *us = new Ultrasonic(ECHO_PIN, TRIG_PIN, MAX_DIST, &Serial);
ISR(INT4_vect) {
  us->setReturnTime(micros());
}

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


float ShortRangeIR::readSensor(){
  if ((millis() - _last_millis)<=SHORTRANGE_LATENCY){ //in the event of "double read" 
    return _prev_reading;
  }
  _prev_reading = applyCalibration(readVoltage(_read_pin));
  _last_millis = millis();
  return _prev_reading;
}

float ShortRangeIR::applyCalibration(float adc_voltage){
  //Impliments the calibration profile shown on the datasheet (only for the linear region)
  //Outside of the linear region should be treated as out-of-bounds (Returns -1)
  //Datasheet maps V = (1/[L + 0.42])m + c, between voltage ranges of 0.3V - 3V
  const float c = 0.1097f;
  const float m = 11.33f;
  float x = 0.0;
  if (adc_voltage < _min_voltage){
    return -1.0;
  }
  else if (adc_voltage > _max_voltage){
    return -1.0;
  }
  x = (adc_voltage - c) / m;
  return (1/x) - 0.42;
}

float LongRangeIR::readSensor(){
  if ((millis() - _last_millis)<=LONGRANGE_LATENCY){ //in the event of "double read" 
    return _prev_reading;
  }
  _prev_reading = applyCalibration(readVoltage(_read_pin));
  _last_millis = millis();
  return _prev_reading;
}

float LongRangeIR::applyCalibration(float adc_voltage){
  //Impliments the calibration profile shown on the datasheet (only for the linear region)
  //Outside of the linear region should be treated as out-of-bounds (Returns -1)
  //Datasheet maps V = (1/[L])m + c, thus L = 1/V between voltage ranges of 0.3V - 3V
  const float m = 18.744f;
  const float c = 0.3196f;
  if (adc_voltage < _min_voltage){
    return -1.0;
  }
  else if (adc_voltage > _max_voltage){
    return -1.0;
  }
  return (1/((adc_voltage - c) / m));
  
}

void Ultrasonic::initUltrasonic(){
  runUltrasonic();
}

void Ultrasonic::runUltrasonic(){
  digitalWrite(_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pin, LOW);
  setSentTime(micros());
}

Ultrasonic::Ultrasonic(uint8_t echo_pin, uint8_t trigger_pin, uint8_t max_dist, HardwareSerial* SerialCom) 
: Sensor(uint8_t(255)),  _echo_pin(echo_pin), _trigger_pin(trigger_pin), _max_dist(max_dist),_serial_com(SerialCom){
  initUltrasonic();
};
float Ultrasonic::readSensor() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  /*
  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(_echo_pin) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (_max_dist + 1000)) {
      if (DIAGNOSTICS){
        _serial_com->println("HC-SR04: NOT found");
      }
      return -1;
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
  */

  t2 = getReturnTime();
  t1 = getSentTime();
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
  runUltrasonic();
  return cm;
};


float Gyroscope::readSensor(){
  if (_bno08x->wasReset()) {
    _bno08x->enableReport(SH2_GYROSCOPE_UNCALIBRATED);
  }

  if (_bno08x->getSensorEvent(_sensorValue)) {
    if (_sensorValue->sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
      float gyroZ =
          _sensorValue->un.gyroscope
              .z;  // Current Measured Angular Velocity Around The Z Axis
      _serial_com->print("Gyroscope I2C: ");
      _serial_com->println(gyroZ);
      return gyroZ;

    }
  }
  return -1001.0;
};



