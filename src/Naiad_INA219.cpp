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
#include <Naiad_INA219.h>
#include <Wire.h>

Naiad_INA219::Naiad_INA219(uint8_t addr, const char* name, BatteryInstance_t i)
    : Adafruit_INA219(addr) {
    // Call the base class constructor with the address
    sensor_name = name;
    instance = i;
}

/*
 * Calculate the calibration value using the algorithm in the data sheet.
 * Borrowed from here https://github.com/pcbreflux/espressif/blob/master/esp32/app/ESP32_ble_i2c_ina219/main/INA219.cpp
 */
uint16_t calibrate(float shunt_val, float v_shunt_max, float v_bus_max, float i_max_expected) {
    float r_shunt;
    float current_lsb;
    float power_lsb;
    uint16_t cal;
    uint16_t digits;
    float min_lsb, swap;
#ifdef INA219_DEBUG
    float max_current, max_before_overflow, max_shunt_v, max_shunt_v_before_overflow, max_power, i_max_possible, max_lsb;
#endif

    r_shunt = shunt_val;

    min_lsb = i_max_expected / 32767;

    current_lsb = min_lsb;
    digits = 0;

    /* From datasheet: This value was selected to be a round number near the Minimum_LSB.
     * This selection allows for good resolution with a rounded LSB.
     * eg. 0.000610 -> 0.000700
     */
    while (current_lsb > 0.0) {  // If zero there is something weird...
        if ((uint16_t)current_lsb / 1) {
            current_lsb = (uint16_t)current_lsb + 1;
            current_lsb /= pow(10, digits);
            break;
        } else {
            digits++;
            current_lsb *= 10.0;
        }
    };

    swap = (0.04096) / (current_lsb * r_shunt);
    cal = (uint16_t)swap;
    power_lsb = current_lsb * 20;

#ifdef INA219_DEBUG
    i_max_possible = v_shunt_max / r_shunt;
    max_lsb = i_max_expected / 4096;
    max_current = current_lsb * 32767;
    max_before_overflow = max_current > i_max_possible ? i_max_possible : max_current;

    max_shunt_v = max_before_overflow * r_shunt;
    max_shunt_v_before_overflow = max_shunt_v > v_shunt_max ? v_shunt_max : max_shunt_v;

    max_power = v_bus_max * max_before_overflow;
    Serial.print("v_bus_max:     ");
    Serial.println(v_bus_max, 8);
    Serial.print("v_shunt_max:   ");
    Serial.println(v_shunt_max, 8);
    Serial.print("i_max_possible:        ");
    Serial.println(i_max_possible, 8);
    Serial.print("i_max_expected: ");
    Serial.println(i_max_expected, 8);
    Serial.print("min_lsb:       ");
    Serial.println(min_lsb, 12);
    Serial.print("max_lsb:       ");
    Serial.println(max_lsb, 12);
    Serial.print("current_lsb:   ");
    Serial.println(current_lsb, 12);
    Serial.print("power_lsb:     ");
    Serial.println(power_lsb, 8);
    Serial.println("  ");
    Serial.print("cal:           ");
    Serial.println(cal);
    Serial.print("r_shunt:       ");
    Serial.println(r_shunt, 6);
    Serial.print("max_before_overflow:       ");
    Serial.println(max_before_overflow, 8);
    Serial.print("max_shunt_v_before_overflow:       ");
    Serial.println(max_shunt_v_before_overflow, 8);
    Serial.print("max_power:       ");
    Serial.println(max_power, 8);
    Serial.println("  ");
#endif
    return cal;
}

//**************************************************************************/
// Configures to INA219 to be able to measure up to 16V and 30A
// using the built in 0.1ohm shunt and an external 30A shunt that drops 75mV at 30A
// shunt current 30A
// shunt voltage 75mv
// R = V / I
// = 0.075 / 30
// Shunt resistance = 0.0025 ohms
//**************************************************************************/
/*!
 *  @brief  Configures to INA219 to be able to measure up to 16V at 30A
 */
void Naiad_INA219::setCalibration_16V_30A() {
    ina219_calValue = calibrate(SHUNT_R, SHUNT_MAX_V, BUS_MAX_V, MAX_CURRENT);

    // Set multipliers to convert raw current/power values
    ina219_currentDivider_mA = 1;    // Current LSB = 1000uA per bit (1000/1000 = 1)
    ina219_powerMultiplier_mW = 20;  // Power LSB = 1mW per bit (1/1)

    // Set Calibration register to 'Cal' calculated above
    Adafruit_BusIO_Register calibration_reg =
        Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
    calibration_reg.write(ina219_calValue, 2);

    // Set Config register to take into account the settings above
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                      INA219_CONFIG_GAIN_2_80MV |
                      INA219_CONFIG_BADCRES_12BIT_64S_34MS |
                      INA219_CONFIG_SADCRES_12BIT_64S_34MS |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    Adafruit_BusIO_Register config_reg =
        Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
    _success = config_reg.write(config, 2);
}

BatteryStat read_ina219(Naiad_INA219& ina219) {
    BatteryStat result;

    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;

    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    const char* sensor = ina219.sensor_name;

#ifdef INA219_DEBUG
    Serial.printf("Battery %s\n", sensor);
    Serial.print("Bus Voltage:   ");
    Serial.print(busvoltage);
    Serial.println(" V");
    Serial.print("Shunt Voltage: ");
    Serial.print(shuntvoltage);
    Serial.println(" mV");
    Serial.print("Load Voltage:  ");
    Serial.print(loadvoltage);
    Serial.println(" V");
    Serial.print("Current:       ");
    Serial.print(current_mA);
    Serial.println(" mA");
    Serial.print("Power:         ");
    Serial.print(power_mW);
    Serial.println(" mW");
    Serial.println("");
#endif
    result.voltage = busvoltage;           // in Nmv
    result.current = current_mA / 1000.0;  // In Amps
    result.temperature = 273 + 15;         // Just a guess in K
    result.instance = ina219.instance;

    return result;
}
