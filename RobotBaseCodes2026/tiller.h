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
    float y_tgts_[3] = {15.0, 30, 45};
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

    bool switchState(State::Name newState,void* data = nullptr);
    void pollState();
    inline void setSerialCom(HardwareSerial *serialCom) { serialCom_ = serialCom; };
    template <typename... Args>
    inline void print(Args... args) { serialCom_->print(args...); }
    template <typename... Args>
    inline void println(Args... args) { serialCom_->println(args...); }
    bool is_battery_voltage_OK();
    inline float get_y_tgt() { return y_tgts_[curr_y_idx_]; }
    inline void inc_y_tgt() { curr_y_idx_++; }
};