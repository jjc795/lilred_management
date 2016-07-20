/* Library for the INA226 Current/Power Monitor */

#include "Arduino.h"

/* Calibration Constants */

#define CALIBRATION_5V_5A                       (0x6400)
#define CALIBRATION_12V_5A                      (0x6400)
#define CALIBRATION_24V_60A                     (0x0A00)

/* Register Addresses */

#define CONFIG_REG                              (0x00)
#define SHUNT_VOLTAGE_REG                       (0x01)
#define BUS_VOLTAGE_REG                         (0x02)
#define POWER_REG                               (0x03)
#define CURRENT_REG                             (0x04)
#define CALIBRATION_REG                         (0x05)
#define MASK_ENABLE_REG                         (0x06)   
#define ALERT_LIMIT_REG                         (0x07)

/* Config Register Options */

#define CONFIG_RESET                            (0x8000)

#define CONFIG_AVG_MASK                         (0x0E00)  // Averaging mode mask
#define CONFIG_AVG_1                            (0x0000)  // 1 sample
#define CONFIG_AVG_4                            (0x0200)  // 4 samples
#define CONFIG_AVG_16                           (0x0400)  // 16 samples
#define CONFIG_AVG_64                           (0x0600)  // 64 samples
#define CONFIG_AVG_128                          (0x0800)  // 128 samples
#define CONFIG_AVG_256                          (0x0A00)  // 256 samples
#define CONFIG_AVG_512                          (0x0C00)  // 512 samples
#define CONFIG_AVG_1024                         (0x0E00)  // 1024 samples

#define CONFIG_VBUSCT_MASK                      (0x01C0)  // Bus voltage conversion time mask
#define CONFIG_VBUSCT_140US                     (0x0000)  // 140us
#define CONFIG_VBUSCT_204US                     (0x0040)  // 204us
#define CONFIG_VBUSCT_332US                     (0x0080)  // 332us
#define CONFIG_VBUSCT_588US                     (0x00C0)  // 588us
#define CONFIG_VBUSCT_1100US                    (0x0100)  // 1.1ms
#define CONFIG_VBUSCT_2116US                    (0x0140)  // 2.116ms
#define CONFIG_VBUSCT_4156US                    (0x0180)  // 4.156ms
#define CONFIG_VBUSCT_8244US                    (0x01C0)  // 8.244ms

#define CONFIG_VSHCT_MASK                       (0x0038)  // Shunt voltage conversion time mask
#define CONFIG_VSHCT_140US                      (0x0000)  // 140us
#define CONFIG_VSHCT_204US                      (0x0008)  // 204us
#define CONFIG_VSHCT_332US                      (0x0010)  // 332us
#define CONFIG_VSHCT_588US                      (0x0018)  // 588us
#define CONFIG_VSHCT_1100US                     (0x0020)  // 1.1ms
#define CONFIG_VSHCT_2116US                     (0x0028)  // 2.116ms
#define CONFIG_VSHCT_4156US                     (0x0030)  // 4.156ms
#define CONFIG_VSHCT_8244US                     (0x0038)  // 8.244ms

#define CONFIG_MODE_MASK                        (0x0007)  // Operation mode mask
#define CONFIG_MODE_POWERDOWN                   (0x0000)
#define CONFIG_MODE_VSH_TRIGGERED               (0x0001)
#define CONFIG_MODE_VBUS_TRIGEERED              (0x0002)
#define CONFIG_MODE_VSH_VBUS_TRIGGERED          (0x0003)
#define CONFIG_MODE_VSH_CONTINUOUS              (0X0005)
#define CONFIG_MODE_VBUS_CONTINOUS              (0x0006)
#define CONFIG_MODE_VSH_VBUS_CONTINUOUS         (0x0007)

/* Mask/Enable Register Options */

#define MASK_ENABLE_MASK                        (0xFC1F)
#define MASK_ENABLE_SOL                         (0x8000)  // Shunt voltage over-voltage
#define MASK_ENABLE_SUL                         (0x4000)  // Shunt voltage under-voltage
#define MASK_ENABLE_BOL                         (0x2000)  // Bus voltage over-voltage
#define MASK_ENABLE_BUL                         (0x1000)  // Bus voltage under-voltage
#define MASK_ENABLE_POL                         (0x0800)  // Power over-limit
#define MASK_ENABLE_CNVR                        (0x0400)  // Conversion ready
#define MASK_ENABLE_AFF                         (0x0010)  // Alert function flag
#define MASK_ENABLE_CVRF                        (0x0008)  // Conversion ready flag
#define MASK_ENABLE_OVF                         (0x0004)  // Math overflow flag
#define MASK_ENABLE_APOL                        (0x0002)  // Alert polarity bit; 1=inverted, 0=normal
#define MASK_ENABLE_LEN                         (0x0001)  // Alert latch enable; 1=latch enabled, 0=transparent

class ina226 {
  public:
    ina226(uint8_t addr);
    void begin(uint16_t cal, uint16_t avg, uint16_t vbusct, uint16_t vshct, uint16_t mode);
    void alertConfig(uint16_t func, uint16_t limit, uint16_t polarity, uint16_t len);
    float getBusVoltage(void);
    float getShuntVoltage(void);
    float getCurrent(void);
    float getPower(void);

  private:
    uint8_t slave_addr;
    uint16_t calValue;
    float currentLSB;
    float powerLSB;
    float shuntVoltageLSB;
    float busVoltageLSB;

    int16_t getBusVoltageRaw(void);
    int16_t getShuntVoltageRaw(void);
    int16_t getCurrentRaw(void);
    int16_t getPowerRaw(void);
};

