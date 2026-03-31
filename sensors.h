#ifndef SENSORS_H
#define SENSORS_H

#include "HardwareSerial.h"
#include <stdint.h>
#include "mappings.h"
#include <Arduino.h>
#include <Adafruit_BNO08x.h>  //Need for Gyroscope
#include "utils.h"

//Class methods for reading and calibrating different sensor variants


float readVoltage(uint8_t pin);


class Sensor{
  protected:
    uint8_t _read_pin;
    virtual float applyCalibration(float adc_voltage)=0; //Contains the math for each sensor that maps ADC values to a meaningful value

  public:
    Sensor(uint8_t read_pin);
    virtual float readSensor();
    float readSensorFiltered(int nSamples, int delayMs = 0);
    virtual void setReadPin(uint8_t new_pin);
    virtual uint8_t getReadPin();

};


class ShortRangeIR: public Sensor{
  private:
    uint32_t _last_millis;
    float _prev_reading;
    float _min_voltage = 0.25;
    float _max_voltage = 2.9;


  public:
    ShortRangeIR(uint8_t read_pin) : Sensor(read_pin){
      _last_millis = millis();
      _prev_reading = -1.0;
    }
    float readSensor() override;
    float applyCalibration(float adc_voltage) override;
    //TODO: ADD MOVING AVERAGE FILTER

};


class LongRangeIR: public Sensor{
  private:
    uint32_t _last_millis;
    float _prev_reading;
    float _min_voltage = 0.1;
    float _max_voltage = 3.0;


  public:
    LongRangeIR(uint8_t read_pin) : Sensor(read_pin){
      _last_millis = millis();
      _prev_reading = -1.0;
    }
    float readSensor() override;
    float applyCalibration(float adc_voltage) override;
};

class Ultrasonic: public Sensor{
  private:
    uint8_t _echo_pin = ECHO_PIN;
    uint8_t _trigger_pin = TRIG_PIN;
    unsigned int _max_dist = MAX_DIST;
  
  public:
    Ultrasonic(uint8_t echo_pin, uint8_t trigger_pin, unsigned int max_dist);

    float readSensor() override;
    inline float applyCalibration(float adc_voltage) override {return adc_voltage;};
    
};


class Gyroscope: public Sensor{
  private:
    Adafruit_BNO08x* _bno08x;
    sh2_SensorValue_t* _sensorValue;
    float _rad = 0.0;
    float _last_omega = 0.0;
    uint32_t _prev_micros = 0.0;
    HardwareSerial* _serial_com;
    RingBuffer<float,4>* _prev_measurements;

    
  public:
    Gyroscope(Adafruit_BNO08x* bno08x,sh2_SensorValue_t* sensorValue,HardwareSerial* SerialCom):Sensor(uint8_t(0)),_bno08x(bno08x),_sensorValue(sensorValue),_serial_com(SerialCom){
      _prev_measurements = new RingBuffer<float,4>();
      _prev_micros = micros();
      _serial_com->println("Enabling Gyroscope...");
      if (!_bno08x->begin_I2C() ||
          !_bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
            _serial_com->println("IMU failed");
          }
    };
    float readSensor(bool apply_filter = false);
    inline float applyCalibration(float adc_voltage) override {return adc_voltage;};
    void resetAngle() { _rad = 0.0; _prev_micros = micros(); }
    float getAngle() { return _rad; }

    
};


struct gyroData{
  
  
};


#endif








