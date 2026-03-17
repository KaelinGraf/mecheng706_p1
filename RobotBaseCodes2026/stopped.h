// #ifndef STOPPED_H
// #define STOPPED_H

// #include "state.h"

// class Stopped : public State {
//   public:
//     Stopped(Tiller* tiller) : State(State::STOPPED, tiller), count_(0), last_millis_(0) {};
//     ~Stopped() {};

//     void begin() override;
//     void end() override;
//     void poll() override;
  
//   private:
//     int count_;
//     unsigned long last_millis_;
// };


// #endif // STOPPED_H