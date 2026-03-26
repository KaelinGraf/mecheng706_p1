#include "SoftwareSerial.h"
#include <stdint.h>
#include <Arduino.h>
#include "state.h"
#include "sensors.h"
#include "servo_control.h"
#include "pid.h"

#define NUM_Y_TGTS 3

class Tiller {
private:
    State *current_state_;
    State *states_[State::NUM_STATES];
    HardwareSerial *serialCom_;
    SoftwareSerial *btSerial_ = nullptr;
    float y_tgts_[NUM_Y_TGTS] = {15.0, 30.0, 45.0};
    unsigned int curr_y_idx_ = 0;

public:
    ShortRangeIR* _front_left_ir;
    ShortRangeIR* _front_right_ir;
    LongRangeIR* _side_right_ir;
    LongRangeIR* _side_left_ir;
    Gyroscope* _gyro;
    Ultrasonic* _ultrasonic;
    driveMotors* _motors;
    Tiller(Adafruit_BNO08x* bno08x,sh2_SensorValue_t* sensorValue,HardwareSerial* SerialCom);
    ~Tiller();
    void testSensors();
    bool switchState(State::Name newState,void* data = nullptr);
    void pollState();
    inline void setSerialCom(HardwareSerial *serialCom) { serialCom_ = serialCom; };
    inline void setBluetoothSerial(SoftwareSerial *btSerial) { btSerial_ = btSerial; };
    template <typename... Args>
    inline void print(Args... args) { 
        if (serialCom_) serialCom_->print(args...); 
        if (btSerial_) btSerial_->print(args...); 
    }
    template <typename... Args>
    inline void println(Args... args) { 
        if (serialCom_) serialCom_->println(args...); 
        if (btSerial_) btSerial_->println(args...); 
    }
    bool is_battery_voltage_OK();
    inline float get_y_tgt() { return y_tgts_[curr_y_idx_]; }
    inline void inc_y_tgt() { if (curr_y_idx_ < NUM_Y_TGTS-1) curr_y_idx_++; }
};