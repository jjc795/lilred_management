/* Utility functions */

uint16_t readRegister (uint8_t slave_addr, uint8_t reg_addr);
void writeRegister (uint8_t slave_addr, uint8_t reg_addr, uint16_t val);
uint16_t readData (uint8_t slave_addr);
void writeByte (uint8_t slave_addr, uint8_t val);
