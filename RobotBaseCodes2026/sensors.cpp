#include "Arduino.h"
#include"sensors.h"
float readVoltage(uint8_t pin){
  float adc_value = (analogRead(pin));
  return (adc_value / ADC_RANGE) * V_ADC;

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

float Sensor::readSensorFiltered(int nSamples, int delayMs) {
  // Cap at a reasonable max to avoid stack issues
  const int MAX_SAMPLES = 20;
  if (nSamples > MAX_SAMPLES) nSamples = MAX_SAMPLES;
  if (nSamples < 1) nSamples = 1;

  float samples[MAX_SAMPLES];
  int valid = 0;

  for (int i = 0; i < nSamples; i++) {
    float reading = readSensor();
    // Discard invalid readings: negative, NaN, or Inf
    if (!isnan(reading) && !isinf(reading) && reading > 0.0) {
      // Insertion sort to keep samples ordered (for median)
      int j = valid;
      while (j > 0 && samples[j - 1] > reading) {
        samples[j] = samples[j - 1];
        j--;
      }
      samples[j] = reading;
      valid++;
    }
    if (delayMs > 0 && i < nSamples - 1) {
      delay(delayMs);
    }
  }

  if (valid == 0) return -1.0;
  if (valid == 1) return samples[0];

  // Return the median (robust against outliers and ghost echoes)
  if (valid % 2 == 1) {
    return samples[valid / 2];
  } else {
    return (samples[valid / 2 - 1] + samples[valid / 2]) / 2.0;
  }
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
  long curr_ms = millis();
  if ((curr_ms - _last_millis)<=LONGRANGE_LATENCY){ //in the event of "double read" 
    return _prev_reading;
  }
  _prev_reading = applyCalibration(readVoltage(_read_pin));
  _last_millis = curr_ms;
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
  int first = readSensor();
}

void Ultrasonic::runUltrasonic(){
  digitalWrite(_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pin, LOW);
}

Ultrasonic::Ultrasonic(uint8_t echo_pin, uint8_t trigger_pin, int max_dist) 
: Sensor(uint8_t(255)),  _echo_pin(echo_pin), _trigger_pin(trigger_pin), _max_dist(max_dist){
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

  if (t2 < t1){
    Serial.println("UltraSonic Out of sync, return last cm");
    t1 = getLastSent();
  }

  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (DIAGNOSTICS){
    if (pulse_width > _max_dist) {
      Serial.println("HC-SR04: Out of range");
    } else {
      Serial.print("HC-SR04:");
      Serial.print(cm);
      Serial.println("cm");
    }
  }
  runUltrasonic();
  return cm;
};

float Gyroscope::readSensor(bool apply_filter=false){
  if (_bno08x->wasReset()) {
    _bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED);
  }

  if (_bno08x->getSensorEvent(_sensorValue)) {
    if (_sensorValue->sensorId == SH2_GYROSCOPE_CALIBRATED) {
      float gyroZ =_sensorValue->un.gyroscope.z;  // Current Measured Angular Velocity Around The Z Axis
      
      uint32_t now = micros();
      float dt = (now - _prev_micros) / 1000000.0f;
      _prev_micros = now;
      _rad += gyroZ * dt;
      
      _prev_measurements->push(gyroZ);
      return (apply_filter)? _prev_measurements->average():gyroZ;

    }
    
  }
  return -1001.0;
};



