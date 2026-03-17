#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <Arduino.h>

template<typename OutputType = uint8_t>
class PID{
  //use PID<> myPid(values) to instantiate for PWM, else PID<type> myPid(values);
  private:
    float _ki;
    float _kp;
    float _kd;
    float _f;
    float _prev_error=0;
    float _error_integral=0;
    uint32_t _prev_micros = 0;
    bool _integral_enabled=false;
    OutputType _output_min;
    OutputType _output_max;
    OutputType _output_neutral;

  
  public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float f = 0.0f,bool enable_integral=false, OutputType output_min=0, OutputType output_max=255, OutputType output_neutral=0)
        : _kp(kp), _ki(ki), _kd(kd), _f(f),_integral_enabled(enable_integral),_output_min(output_min),_output_max(output_max),_output_neutral(output_neutral) {
    
    }
    void setFeedForward(float f){_f=f;};
    float getFeedForward() {return _f;};
    void enableIntegral(){_integral_enabled=true;}
    void disableIntegral(){_integral_enabled = false;}
    void setOutputLimits(OutputType min, OutputType max){
      _output_min = min;
      _output_max = max;}
    OutputType update(float error);
    void resetPID(){
      _prev_error = 0.0;
      _error_integral = 0.0;
      _prev_micros = micros();
    }


};

template<typename OutputType>
OutputType PID<OutputType>::update(float error){
  uint32_t now = micros();
  float delta_time = (now - _prev_micros)/1000000.0f;
  _prev_micros = now;
  float p_term = _kp * error;
  _error_integral = (_integral_enabled) ? _error_integral + (error * delta_time) : 0.0;
  float i_term = _error_integral * _ki;
  float d_term = _kd * ((error - _prev_error)/delta_time);
  float control_effort = p_term + i_term + d_term;

  if (control_effort > static_cast<float>(_output_max)){
    control_effort = _output_max;
    if(error > 0){
      _error_integral = (_integral_enabled) ? _error_integral - (error * delta_time) : 0.0;
    }
  }
  else if (control_effort < static_cast<float>(_output_min)){
    control_effort = _output_min;
      if(error < 0){
          _error_integral = (_integral_enabled) ? _error_integral - (error * delta_time) : 0.0;

    }
  }
  _prev_error = error;

  return static_cast<OutputType>(control_effort);

}

#endif