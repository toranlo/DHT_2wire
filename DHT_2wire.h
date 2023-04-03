/*!
 *  @file DHT_2wire.h
 *
 *  This is a library for DHT series of low cost temperature/humidity sensors
 *  using two wire interface (sensor self-power).
 *  DOES NOT REQUIRE any adittional libraries to use this class.
 *
 *  Writen by toranlo.
 *
 *  Based on DHT.h from Adafruit Industries.
 *  https://github.com/adafruit/DHT-sensor-library
 *
 *  MIT license, all text above must be included in any redistribution
 *
 */

#ifndef DHT2WIRE_H
#define DHT2WIRE_H

#include "Arduino.h"

/* Uncomment to enable printing out nice debug messages. */
//#define DHT_DEBUG

#define DEBUG_PRINTER                                                          \
  Serial /**< Define where debug output will be printed.                       \
          */

/* Setup debug printing macros. */
#ifdef DHT_DEBUG
#define DEBUG_PRINT(...)                                                       \
  { DEBUG_PRINTER.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(...)                                                     \
  { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...)                                                       \
  {} /**< Debug Print Placeholder if Debug is disabled */
#define DEBUG_PRINTLN(...)                                                     \
  {} /**< Debug Print Line Placeholder if Debug is disabled */
#endif

/* Define types of sensors. */
static const uint8_t DHT11{11};  /**< DHT TYPE 11 */
static const uint8_t DHT12{12};  /**< DHY TYPE 12 */
static const uint8_t DHT21{21};  /**< DHT TYPE 21 */
static const uint8_t DHT22{22};  /**< DHT TYPE 22 */
static const uint8_t AM2301{21}; /**< AM2301 */

#if defined(TARGET_NAME) && (TARGET_NAME == ARDUINO_NANO33BLE)
#ifndef microsecondsToClockCycles
/*!
 * As of 7 Sep 2020 the Arduino Nano 33 BLE boards do not have
 * microsecondsToClockCycles defined.
 */
#define microsecondsToClockCycles(a) ((a) * (SystemCoreClock / 1000000L))
#endif
#endif

/*!
 *  @brief  Class that stores state and functions for DHT
 */
class DHT_2WIRE {
public:
  DHT_2WIRE(uint8_t pin, uint8_t type, uint8_t count = 6);
  void begin(uint8_t usec = 55);
  float readTemperature(bool S = false, bool force = false);
  float convertCtoF(float);
  float convertFtoC(float);
  float computeHeatIndex(bool isFahrenheit = true);
  float computeHeatIndex(float temperature, float percentHumidity,
                         bool isFahrenheit = true);
  float readHumidity(bool force = false);
  bool read(bool force = false);

private:
  uint8_t data[5];
  uint8_t _pin, _type;
#ifdef __AVR
  // Use direct GPIO access on an 8-bit AVR so keep track of the port and
  // bitmask for the digital pin connected to the DHT.  Other platforms will use
  // digitalRead.
  uint8_t _bit, _port;
#endif
  uint32_t _lastreadtime, _maxcycles;
  bool _lastresult;
  uint8_t pullTime; // Time (in usec) to pull up data line before reading

  uint32_t expectPulse(bool level);
};

/*!
 *  @brief  Class that defines Interrupt Lock Avaiability
 */
//Disabling interrupts is not required
 #if 0
class InterruptLock_2wire {
public:
  InterruptLock_2wire() {
#if !defined(ARDUINO_ARCH_NRF52)
    noInterrupts();
#endif
  }
  ~InterruptLock_2wire() {
#if !defined(ARDUINO_ARCH_NRF52)
    interrupts();
#endif
  }
};
#endif
#endif
