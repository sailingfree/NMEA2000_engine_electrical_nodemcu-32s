/*
Copyright (c) 2022 Peter Martin www.naiadhome.com

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Adafruit_INA219.h>
#include <Wire.h>

#define R1 0.0025  // 75mV 30A shunt
#define SHUNT_R R1
#define SHUNT_MAX_V 0.1  // 100mV to be safe
#define BUS_MAX_V 16     // 12Volt system
#define MAX_CURRENT 32   // max expected current from house battery

enum BatteryInstance_t {
    BAT_HOUSE = 0,
    BAT_ENGINE = 1
};

//#define INA219_DEBUG

/*
 * Create a derived class so we can have our own calibration options
 * Also save a name and instance number against the sensor so we can have more than one
 */
class Naiad_INA219 : public Adafruit_INA219 {
   public:
    const char* sensor_name;
    BatteryInstance_t instance;
    Naiad_INA219(uint8_t addr = INA219_ADDRESS, const char* name = "", BatteryInstance_t i = BAT_HOUSE);
    void setCalibration_16V_30A();
    bool ispresent();

    private:
        bool present;
};

typedef struct batteryStat {
    BatteryInstance_t instance;
    String name;
    double voltage;      // mV
    double current;      // mA
    double temperature;  // K
} BatteryStat;

BatteryStat read_ina219(Naiad_INA219& ina219);