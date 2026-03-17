#include <stdint.h>
#include <Arduino.h>
#include "state.h"
#include "sensors.h"
#include "servo_control.h"
#include "pid.h"

class Tiller {
private:
    State *current_state_;
    State *states_[State::NUM_STATES];
    HardwareSerial *serialCom_;



public:
    ShortRangeIR* _front_left_ir;
    ShortRangeIR* _front_right_ir;
    LongRangeIR* _side_right_ir;
    LongRangeIR* _side_left_ir;
    Gyroscope* _gyro;
    driveMotors* _motors;
    Tiller(Adafruit_BNO08x* bno08x,sh2_SensorValue_t* sensorValue,HardwareSerial* SerialCom);
    ~Tiller();

    bool switchState(State::Name newState);
    void pollState();
    inline void setSerialCom(HardwareSerial *serialCom) { serialCom_ = serialCom; };
    template <typename... Args>
    inline void print(Args... args) { serialCom_->print(args...); }
    template <typename... Args>
    inline void println(Args... args) { serialCom_->println(args...); }
    bool is_battery_voltage_OK();
};