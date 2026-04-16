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
#include <SoftwareSerial.h>
#include "pid.h"
#include "servo_control.h"
#include "tiller.h"

// Bluetooth Setup matching WirelessSetup2026.ino
#define BLUETOOTH_RX 19
#define BLUETOOTH_TX 18
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Gyroscope initialisation
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float rad = 0.0;

// #define NO_READ_GYRO  //Uncomment if GYRO is not attached.

#define NO_HC \
  -SR04 // Uncomment if HC-SR04 ultrasonic ranging sensor is not attached.

// #define NO_BATTERY_V_OK //Uncomment if BATTERY_V_OK if you do not care about battery damage.

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
  EICRB |= (1 << ISC40);   // ISC41 = 0, ISC40 = 1 → ANY CHANGE
  EICRB &= ~(1 << ISC41);
  EIMSK |= (1 << INT4);
  
  Serial.begin(115200);
  // turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug
  // info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  BluetoothSerial.begin(115200);
  
  tiller = new Tiller(&bno08x,&sensorValue, &Serial);
  tiller->setBluetoothSerial(&BluetoothSerial); // Enable dual-printing to Bluetooth

  delay(1000); // settling time but no really needed
}

void loop(void) // main loop
{
  tiller->pollState();
  //tiller->testSensors();
}

void printBluetooth()
{
  serialOutput(analogRead(A4), analogRead(A4), analogRead(A4));
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
