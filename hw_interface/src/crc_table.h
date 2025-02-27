#ifndef CRC_TABLE_H
#define CRC_TABLE_H

#include <cstdint>

uint8_t CRC8_Table(const uint8_t* p, uint8_t counter);
uint16_t CRC16_Table(const uint8_t *p, uint8_t counter);
uint32_t CRC32_Table(const uint8_t *p, uint8_t counter);

#endif // CRC_TABLE_H
