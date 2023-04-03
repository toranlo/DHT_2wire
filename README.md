
# DHT_2wire

This is an Arduino library for DHT series of low cost temperature/humidity sensors using two wire interface (sensor self-power) instead of the three wires required by the sensor.
This allows placing the sensor in a remote location separated from the Arduino, and connecting it using a two wire connection.

DOES NOT REQUIRE any additional libraries.

Sensor is powered from the data line using the following circuit:

       ARDUINO                                                   DHT SENSOR  
    PIN --------------------------/~/-----------------------------<- DATA (pin 2)  
                                                |              |  
                                                |              #  
                                                |              # R2  
                                                |              #  
                                                |  R1    +D1-  |  
                TWO WIRE CABLE                  ---###---###---|-->- POWER (pin 1, left)  
                                                               |  
                                                               # +  
                                                               # C1  
                                                               # -  
                                                               |  
    GND --------------------------/~/------------------------------- GND (pin4, right)  

R1: 100 Ohm 5% 1/4 W.  
R2: pullup resistor, 5.1K 5% 1/4W for DHT11 (1K for DHT21 & DHT22).  
D1: Small signal Schottky diode (e.g. BAT42), anode connected to R1.  
C1: 100uF/10V electrolytic or tantalum capacitor, positive connected to D1.  
PIN: ARDUINO Digital pin, HIGH level must be 5V.  

Cable length can be up to 20m (according to the DHT11 datasheet), although it can be less depending on the cable capacitance.

Tested in ARDUINO UNO R3 with DHT11 and a 10m cable (cable capacitance=500 pf).

Written by toranlo.

Based in the DHT sensor library from Adafruit Industries
  https://github.com/adafruit/DHT-sensor-library

Note:  
In the DHT_2wire library, interrupts are enabled during the readout of the sensor so the Arduino millis() and delay() functions are not affected.  
In the original Adafruit DHT library, interrupts are disabled during the readout of the 40 data bits from the sensor, which takes 3 to 5 ms according to the datasheet. In this period, the millis interrupt cannot enter, losing 2 to 4 interrupts (as the first one is stored in the interrupt flag) and the millis counter is not updated. This makes the millis() function to loose time on every sensor measurement.  
In Arduino Uno the millis interrupt is the timer 0 interrupt that enters every 1.024 ms. If a measurement is done once every 2 seconds, like in the example DHTtester.ino from Adafruit, we will be losing 2 to 4 ms every 2 sec, that is, the millis() function would lose 3.6 to 7.2 sec per hour.  
To solve this problem, the timer 0 interrupt duration has been measured and it takes approx. 6.1 us. This time is low enough, compared with the duration of the sensor pulses (this duration is at least 26 us according to the datasheet),  to not interfere with the measurements of the bits of the sensor. For this reason, interrupts can be enabled during the measurement of the sensor pulses and no time is lost by the millis() function.   
It has been tested for hours without any measurement errors.  


MIT license, check license.txt for more information  
All text above must be included in any redistribution
