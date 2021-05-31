#ifndef FLASH_EX_H
#define FLASH_EX_H
#include "datatypes.h"

void hal_flash_write(uint32_t addr, void *conf, uint32_t size);
void hal_flash_read(uint32_t addr, void *conf, uint32_t size);

#endif
