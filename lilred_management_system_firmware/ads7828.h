/* Library for the ADS7828 A/D converter */

#include "Arduino.h"

/** Command Byte Options **/

#define CHANNEL_SEL_MASK              (0xF0)

/* Channel selection with differential inputs -- IN+ followed by IN- */

#define CHANNEL_SEL_DIFF_0_1          (0x00) // IN+ is 0, IN- is 1
#define CHANNEL_SEL_DIFF_2_3          (0x10)
#define CHANNEL_SEL_DIFF_4_5          (0x20)
#define CHANNEL_SEL_DIFF_6_7          (0x30)
#define CHANNEL_SEL_DIFF_1_0          (0x40)
#define CHANNEL_SEL_DIFF_3_2          (0x50)
#define CHANNEL_SEL_DIFF_5_4          (0x60)
#define CHANNEL_SEL_DIFF_7_6          (0x70)

/* Channel selection with single ended inputs */

#define CHANNEL_SEL_SINGLE_0          (0x80) // Channel 0 is input
#define CHANNEL_SEL_SINGLE_1          (0xC0)
#define CHANNEL_SEL_SINGLE_2          (0x90)
#define CHANNEL_SEL_SINGLE_3          (0xD0)
#define CHANNEL_SEL_SINGLE_4          (0xA0)
#define CHANNEL_SEL_SINGLE_5          (0xE0)
#define CHANNEL_SEL_SINGLE_6          (0xB0)
#define CHANNEL_SEL_SINGLE_7          (0xF0)

/* Power down mode selection */

#define PWRDWN_SEL_MASK               (0x0C)
#define PWRDWN_SEL_BT_CNVR            (0x00) // Power down between conversions
#define PWRDWN_SEL_REF_OFF_CNVR_ON    (0x04) // Internal reference off, A/D converter on
#define PWRDWN_SEL_REF_ON_CNVR_OFF    (0x08) // Internal reference on, A/D converter off
#define PWRDWN_SEL_REF_ON_CNVR_ON     (0x0C) // Internal reference on, A/D converter on

class ads7828 {
  public:
    ads7828(uint8_t addr, float low, float high);
    void config(uint8_t channel_sel, uint8_t powerdown = PWRDWN_SEL_REF_OFF_CNVR_ON);
    uint16_t getDataRaw(void);
    float getData(void);

  private:
    uint8_t slave_addr;
    uint8_t command;
    float valueLSB;
};

