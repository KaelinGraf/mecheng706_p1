#include <stdint.h>
#include "mappings.h"
#include <Arduino.h>
//Class methods for reading and calibrating different sensor variants


readVoltage(uint8_t pin){
  float adc_value = (analogRead(pin));
  return (adc_value / ADC_RANGE) * V_ADC;

}


class Sensor{
  protected:
    uint8_t _read_pin;
    virtual float applyCalibration(float adc_voltage)=0; //Contains the math for each sensor that maps ADC values to a meaningful value

  public:
    Sensor(uint8_t read_pin);
    virtual float readSensor();
    virtual void setReadPin(uint8_t new_pin);
    virtual uint8_t getReadPin();

};


class ShortRangeIR: public Sensor{
  private:
    uint8_t _read_pin;
    uint32_t _last_millis;
    float _prev_reading;
    float _min_voltage = 0.3;
    float _max_voltage = 3.0;


  public:
    ShortRangeIR(uint8_t read_pin) : Sensor(read_pin){
      _last_millis = millis()
      _prev_reading = -1.0;
    }
    float readSensor() override;
    float applyCalibration(float adc_voltage) override;
    //TODO: ADD MOVING AVERAGE FILTER

};


class LongRangeIR: public Sensor{
  private:
    uint8_t _read_pin;
    uint32_t _last_millis;
    float _prev_reading;
    float _min_voltage = 0.35;
    float _max_voltage = 3.0;


  public:
    LongRangeIR(uint8_t read_pin) : Sensor(read_pin){
      _last_millis = millis()
      _prev_reading = -1.0;
    }
    float applyCalibration(float adc_voltage) override;
};

class Ultrasonic: public Sensor{
  private:
    uint8_t _echo_pin = ECHO_PIN;
    uint8_t _trigger_pin = TRIG_PIN;
    uint8_t _max_dist = MAX_DIST;
    HardwareSerial* _serial_com;
  
  public:
    Ultrasonic(uint8_t echo_pin, uint8_t trigger_pin, uint8_t max_dist, HardwareSerial* SerialCom);

    float readSensor() override;
    void applyCalibration(){};
    
};