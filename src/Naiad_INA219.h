#include <Adafruit_INA219.h>
#include <Wire.h>

#define R1          0.0025      // 75mV 30A shunt
#define SHUNT_R     R1
#define SHUNT_MAX_V 0.1         // 100mV to be safe
#define BUS_MAX_V   16          // 12Volt system
#define MAX_CURRENT 32          // max expected current from house battery

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
   const char * sensor_name;
   BatteryInstance_t instance;
    Naiad_INA219(uint8_t addr = INA219_ADDRESS, const char * name = "", BatteryInstance_t i = BAT_HOUSE);
    void setCalibration_16V_30A();
};

typedef struct batteryStat {
    BatteryInstance_t   instance;
    String              name;
    double              voltage;    // mV
    double              current;    // mA
    double              temperature; // K
} BatteryStat;

BatteryStat read_ina219(Naiad_INA219 & ina219);