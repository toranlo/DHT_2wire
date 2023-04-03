// Example testing sketch for using various humidity/temperature sensors DHTxx
// connected to the Arduino with only 2 wires instead of 3 (sensor self-power).
// This allows placing the sensor in a remote location separated from 
// the Arduino, and connecting it using a two wire connection.
//
// This is the example DHT_tester.ino from ladyada included in the Adafruit DHT 
// sensor library (https://github.com/adafruit/DHT-sensor-library), modified 
// with some new comments, changing the default sensor type to DHT11, using the
// DHT_2wire library instead of Adafruit DHT library and printing the total
// number of errors.
//
// REQUIRES the DHT_2wire library: https://github.com/toranlo/DHT_2wire
// DOES NOT REQUIRE any additional libraries.
//
// Written by toranlo, public domain.
//
// Sensor is powered from the data line using the following circuit:
//
//    ARDUINO                                                   DHT SENSOR
// PIN --------------------------/~/-----------------------------<- DATA (pin 2)
//                                             |              |
//                                             |              #
//                                             |              # R2
//                                             |              #
//                                             |  R1    +D1-  |
//             TWO WIRE CABLE                  ---###---###---|-->- POWER (pin 1, left)
//                                                            |
//                                                            # +
//                                                            # C1
//                                                            # -
//                                                            |
// GND --------------------------/~/------------------------------- GND (pin4, right)
//
// R1=100 Ohm 5% 1/4 W.
// R2=pull up resistor, 5.1K 5% 1/4W for DHT11 (1K for DHT21 & DHT22).
// D1: Small signal Schottky diode (e.g. BAT42), anode connected to R1.
// C1: 100uF/10V electrolytic or tantalum capacitor, positive connected to D1.
// PIN: ARDUINO Digital pin, HIGH level must be 5V.
//
// Cable length can be up to 20m (according to the DHT11 datasheet), altough can 
// be less depending on the cable capacitance.
// Tested in Arduino UNO R3 with DHT11 and a 10m cable (cable capacitance=500 pf).

//#include "DHT.h"
#include "DHT_2wire.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Initialize DHT sensor.
//DHT dht(DHTPIN, DHTTYPE);
DHT_2WIRE dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx 2 wire interface test!"));

  dht.begin();
}

void loop() {
  static int err=0;
  
  // Wait a few seconds between measurements.
  delay(2000);

  // Reading the temperature or humidity takes a time that is not fixed and 
  // depends on two factors:
  // - It takes about 25 ms for the DHT11 sensor or about 6 mS for the DHT21 
  //   and DHT22 ones, if the previous measurement was made 2 seconds or more
  //   before. The returned value corresponds to the actual measurement of the
  //   temperature and humidity of the sensor.
  // - It is near instantaneous if the previous measurement was made less than
  //   2 seconds before. In this case, the returned value is the latest 
  //   measurement of the sensor (that can be up to 2 seconds ‘old’).

  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    err++;
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.print(F("°F "));
  Serial.print(F("(Errors: "));
  Serial.print(err);
  Serial.println(F(")"));
  
}
