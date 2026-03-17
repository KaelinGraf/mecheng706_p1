#include <Arduino.h>
#include "state.h"

class Tiller {
private:
    State *current_state_;
    State *states_[State::NUM_STATES];
    HardwareSerial *serialCom_;

public:
    Tiller();
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