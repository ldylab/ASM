#ifndef __CRC_H
#define __CRC_H
#include "stm32f10x.h"
#include <string.h>
#include <stdio.h>

uint32_t reflect(uint32_t crc, int bitnum);
void generate_crc_table();
uint32_t crctablefast(uint8_t* p, uint32_t len);
uint32_t crctable(uint8_t* p, uint32_t len);
uint32_t crcbitbybit(uint8_t* p, uint32_t len);
uint32_t crcbitbybitfast(uint8_t* p, uint32_t len);
void buffer_longtochar(const uint32_t *src, uint8_t *dest, uint32_t len);
uint32_t stm32crc(uint32_t *ptr, int len);
uint32_t stm32_crcbitbybitfast(uint8_t* p, uint32_t len);

#endif
