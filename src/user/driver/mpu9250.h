#pragma once

#include "datatypes.h"
#include "platform.h"
#include "imu.h"

class MPU9250
{
private:
	GPIO_TypeDef *port;
	uint16_t pin;

	void cs(bool en);
	uint8_t spi_read_write_byte(uint8_t data);
	void spi_read(uint8_t addr, uint8_t *buffer, uint32_t size);
	void spi_write(uint8_t addr, uint8_t *buffer, uint32_t size);
	void reg_read(uint8_t addr, uint8_t *buffer, uint32_t size);
	void reg_write(uint8_t addr, uint8_t *buffer, uint32_t size);

	bool ready;
public:
	MPU9250();
	void init(GPIO_TypeDef *po, uint16_t pi);
	void read(mpu_t *sensor);
};
