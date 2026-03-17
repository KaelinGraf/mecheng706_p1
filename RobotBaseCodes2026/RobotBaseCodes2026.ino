/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706
  Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    BNO085 https://www.adafruit.com/product/4754
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp
  https://www.sparkfun.com/products/12728 Servo - Generic (Sub-Micro Size)
  https://www.sparkfun.com/products/9065 Vex Motor Controller 29
  https://www.vexrobotics.com/276-2193.html Vex Motors
  https://www.vexrobotics.com/motors.html Turnigy nano-tech 2200mah 2S
  https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html
    HC 12 Module https://www.hc01.com/downloads/HC-12%20english%20datasheets.pdf

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 18/02/2026
  Author: Trishit Ghatak
*/

/*
        DONT CHANGE HEADERS

-----------------------------
*/

#include <Adafruit_BNO08x.h> //Need for Gyroscope
#include "mappings.h"
#include <Servo.h> //Need for Servo pulse output
#include "pid.h"
#include "servo_control.h"
#include "tiller.h"

// Gyroscope initialisation
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
float rad = 0.0;

// #define NO_READ_GYRO  //Uncomment if GYRO is not attached.

#define NO_HC \
  -SR04 // Uncomment if HC-SR04 ultrasonic ranging sensor is not attached.

// #define NO_BATTERY_V_OK //Uncomment if BATTERY_V_OK if you do not care about
// battery damage.

// State machine states
enum STATE
{
  INITIALISING,
  RUNNING,
  STOPPED
};

int speed_val = 100;
int speed_change;

void delaySeconds(int TimedDelaySeconds);
void flashLED(int LedNumber, int TimedDelay);
void serialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3);
void serialOutputPlotter(int32_t Value1, int32_t Value2, int32_t Value3);
void bluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2,
                                  int32_t Value3);
void serialOutput(int32_t Value1, int32_t Value2, int32_t Value3);
void setupWireless();

int pos = 0;
Tiller *tiller = nullptr;
void setup(void)
{
  // turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug
  // info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  tiller = new Tiller();

  delay(1000); // settling time but no really needed
}

void loop(void) // main loop
{
  tiller->pollState();
}

void printBluetooth()
{
  serialOutput(analogRead(A4), analogRead(A4), analogRead(A4));
}

// Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped()
{
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  // disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500)
  { // print massage every 500ms
    previous_millis = millis();
    tiller->println("STOPPED---------");

#ifndef NO_BATTERY_V_OK
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis)
  {
    indexer++;
    if (indexer > 4)
    {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    }
    else
    {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000)
  {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK

#endif

#ifndef NO_HC - SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000))
    {
      tiller->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000))
    {
      tiller->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  // of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST)
  {
    tiller->println("HC-SR04: Out of range");
  }
  else
  {
    tiller->print("HC-SR04:");
    tiller->print(cm);
    tiller->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  tiller->print("Analog Range A4:");
  tiller->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  if (bno08x.wasReset())
  {
    bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED);
  }

  if (bno08x.getSensorEvent(&sensorValue))
  {
    if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED)
    {
      float gyroZ = sensorValue.un.gyroscope.z; // Current Measured Angular Velocity Around The Z Axis
      tiller->print("Gyroscope I2C: ");
      tiller->println(gyroZ);
    }
  }
  return;
}
#endif

// // Serial command pasing
// void read_serial_command() {
//   if (SerialCom->available()) {
//     char val = SerialCom->read();
//     SerialCom->print("Speed:");
//     SerialCom->print(speed_val);
//     SerialCom->print(" ms ");

//     // Perform an action depending on the command
//     switch (val) {
//       case 'w':  // Move Forward
//       case 'W':
//         forward();
//         SerialCom->println("Forward");
//         break;
//       case 's':  // Move Backwards
//       case 'S':
//         reverse();
//         SerialCom->println("Backwards");
//         break;
//       case 'q':  // Turn Left
//       case 'Q':
//         strafe_left();
//         SerialCom->println("Strafe Left");
//         break;
//       case 'e':  // Turn Right
//       case 'E':
//         strafe_right();
//         SerialCom->println("Strafe Right");
//         break;
//       case 'a':  // Turn Right
//       case 'A':
//         ccw();
//         SerialCom->println("ccw");
//         break;
//       case 'd':  // Turn Right
//       case 'D':
//         cw();
//         SerialCom->println("cw");
//         break;
//       case '-':  // Turn Right
//       case '_':
//         speed_change = -100;
//         SerialCom->println("-100");
//         break;
//       case '=':
//       case '+':
//         speed_change = 100;
//         SerialCom->println("+");
//         break;
//       default:
//         stop();
//         SerialCom->println("stop");
//         break;
//     }
//   }
// }

//----------------------Motor moments------------------------
// The Vex Motor Controller 29 use Servo Control signals to determine speed and
// direction, with 0 degrees meaning neutral
// https://en.wikipedia.org/wiki/Servo_control

// void disable_motors() {
//   left_font_motor.detach();   // detach the servo on pin left_front to turn Vex
//                               // Motor Controller 29 Off
//   left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex
//                               // Motor Controller 29 Off
//   right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex
//                               // Motor Controller 29 Off
//   right_font_motor.detach();  // detach the servo on pin right_front to turn Vex
//                               // Motor Controller 29 Off

//   pinMode(left_front, INPUT);
//   pinMode(left_rear, INPUT);
//   pinMode(right_rear, INPUT);
//   pinMode(right_front, INPUT);
// }

// void enable_motors() {
//   left_font_motor.attach(left_front);  // attaches the servo on pin left_front
//                                        // to turn Vex Motor Controller 29 On
//   left_rear_motor.attach(left_rear);   // attaches the servo on pin left_rear to
//                                        // turn Vex Motor Controller 29 On
//   right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear
//                                         // to turn Vex Motor Controller 29 On
//   right_font_motor.attach(
//       right_front);  // attaches the servo on pin right_front to turn Vex Motor
//                      // Controller 29 On
// }
// void stop()  // Stop
// {
//   left_font_motor.writeMicroseconds(1500);
//   left_rear_motor.writeMicroseconds(1500);
//   right_rear_motor.writeMicroseconds(1500);
//   right_font_motor.writeMicroseconds(1500);
// }

// void forward() {
//   left_font_motor.writeMicroseconds(1500 + speed_val);
//   left_rear_motor.writeMicroseconds(1500 + speed_val);
//   right_rear_motor.writeMicroseconds(1500 - speed_val);
//   right_font_motor.writeMicroseconds(1500 - speed_val);
// }

// void reverse() {
//   left_font_motor.writeMicroseconds(1500 - speed_val);
//   left_rear_motor.writeMicroseconds(1500 - speed_val);
//   right_rear_motor.writeMicroseconds(1500 + speed_val);
//   right_font_motor.writeMicroseconds(1500 + speed_val);
// }

// void ccw() {
//   left_font_motor.writeMicroseconds(1500 - speed_val);
//   left_rear_motor.writeMicroseconds(1500 - speed_val);
//   right_rear_motor.writeMicroseconds(1500 - speed_val);
//   right_font_motor.writeMicroseconds(1500 - speed_val);
// }

// void cw() {
//   left_font_motor.writeMicroseconds(1500 + speed_val);
//   left_rear_motor.writeMicroseconds(1500 + speed_val);
//   right_rear_motor.writeMicroseconds(1500 + speed_val);
//   right_font_motor.writeMicroseconds(1500 + speed_val);
// }

// void strafe_left() {
//   left_font_motor.writeMicroseconds(1500 - speed_val);
//   left_rear_motor.writeMicroseconds(1500 + speed_val);
//   right_rear_motor.writeMicroseconds(1500 + speed_val);
//   right_font_motor.writeMicroseconds(1500 - speed_val);
// }

// void strafe_right() {
//   left_font_motor.writeMicroseconds(1500 + speed_val);
//   left_rear_motor.writeMicroseconds(1500 - speed_val);
//   right_rear_motor.writeMicroseconds(1500 - speed_val);
//   right_font_motor.writeMicroseconds(1500 + speed_val);
// }
