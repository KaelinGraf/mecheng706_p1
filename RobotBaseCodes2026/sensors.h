#include <stdint.h>
#include "mappings.h"
#include <Arduino.h>
//Class methods for reading and calibrating different sensor variants



class Sensor{
  protected:
    uint8_t _read_pin;
    virtual float applyCalibration(float adc_value)=0; //Contains the math for each sensor that maps ADC values to a meaningful value

  public:
    Sensor(uint8_t read_pin);
    virtual float readSensor();
    virtual void setReadPin(uint8_t new_pin);
    virtual uint8_t getReadPin();

};


class ShortRangeIR: public Sensor{
  private:
    uint8_t _read_pin;

  public:
    ShortRangeIR(uint8_t read_pin) : Sensor(read_pin){}

    float applyCalibration(float adc_value) override;

};


class LongRangeIR: public Sensor{
  private:
    uint8_t _read_pin;

  public:
    LongRangeIR(uint8_t read_pin) : Sensor(read_pin){}
    float applyCalibration(float adc_value) override;
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