//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const bool DIAGNOSTICS = true;

const int TRIG_PIN = 48;
const int ECHO_PIN = 49;


// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;


