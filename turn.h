#ifndef TURN_H
#define TURN_H
#include "state.h"
#include "pid.h"

class Turn : public State {
  public:
    Turn(Tiller* tiller) : State(State::TURN, tiller), count_(0), last_millis_(0) {
        _turn_pid = nullptr;
        _x_pid = nullptr;
        _angle_pid = nullptr;
        _phase = 0;
    };
    ~Turn() {
        if (_turn_pid) delete _turn_pid;
        if (_x_pid) delete _x_pid;
        if (_angle_pid) delete _angle_pid;
    };

    void begin() override;
    void end() override;
    void poll() override;
  
  private:
    int count_;
    unsigned long last_millis_;
    int _phase; // 0 for TURN_180, 1 for SQUARE_UP
    PID<float>* _turn_pid;
    PID<float>* _x_pid;
    PID<float>* _angle_pid;
};


#endif // TURN_H