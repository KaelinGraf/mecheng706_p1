#include "SoftwareSerial.h"
#include <stdint.h>
#include <Arduino.h>
#include "state.h"
#include "sensors.h"
#include "servo_control.h"
#include "pid.h"

// Field geometry & tilling configuration
static constexpr float Y_WIDTH = 121.7f;          // field width in cm
static constexpr float Y_MARGIN = 15.0f;           // margin from walls in cm
static constexpr int   NUM_SNAKES = 6;              // number of tilling passes
static constexpr float SENSOR_MAX_RANGE = 80.0f;    // long-range IR max in cm

class Tiller {
private:
    State *current_state_;
    State *states_[State::NUM_STATES];
    HardwareSerial *serialCom_;
    SoftwareSerial *btSerial_ = nullptr;
    float y_tgts_[NUM_SNAKES];          // computed in constructor
    bool  use_far_[NUM_SNAKES];          // true if pass i uses the far-wall sensor
    unsigned int curr_y_idx_ = 0;
    int turn_count_ = 0;                // tracks 180° turns for sensor parity
    int home_wall_sensor_ = -1;         // 0=left, 1=right, -1=not set

public:
    LongRangeIR* _front_left_ir;
    LongRangeIR* _front_right_ir;
    ShortRangeIR* _rear_right_ir;
    ShortRangeIR* _rear_left_ir;
    Gyroscope* _gyro;
    Ultrasonic* _ultrasonic;
    driveMotors* _motors;
    Tiller(Adafruit_BNO08x* bno08x,sh2_SensorValue_t* sensorValue,HardwareSerial* SerialCom);
    ~Tiller();
    void testSensors();
    bool switchState(State::Name newState, TillData data = {-1.0, false});
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
    inline void inc_y_tgt() { if (curr_y_idx_ < NUM_SNAKES-1) curr_y_idx_++; }
    inline int getTurnCount() { return turn_count_; }
    inline void incTurnCount() { turn_count_++; }
    inline int getHomeWallSensor() { return home_wall_sensor_; }
    inline void setHomeWallSensor(int s) { home_wall_sensor_ = s; }
    inline bool useFarSensor() { return use_far_[curr_y_idx_]; }
};