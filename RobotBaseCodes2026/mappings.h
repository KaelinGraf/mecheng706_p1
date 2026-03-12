#include <stdint.h>
//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
#define DIAGNOSTICS true

#define TRIG_PIN  48
#define ECHO_PIN  49


// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
#define MAX_DIST  23200

#define LONGRANGE_LATENCY  48 //ms max latency between read updates (loop time cannot exceed this)
#define SHORTRANGE_LATENCY  20 //^

#define V_ADC  5.0
#define ADC_RANGE  1024


#define left_front_pin  46
#define left_rear_pin  47
#define right_rear_pin  50
#define right_front_pin  51



#define max_duty_motor  2300
#define min_duty_motor  700
#define neutral  1500

#define max_duty_turret  2100
#define min_duty_turret  900
#define neutral_turret  1500
