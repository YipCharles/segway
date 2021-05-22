#pragma once

#include "datatypes.h"
#include "imu.h"
#include "platform.h"

class MPU9250
{
private:
	SPI_HandleTypeDef *spi;
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

	bool init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *hport, uint16_t hpin);
	bool read(mpu_t *sensor);
	// void read_request(mpu_t *sensor);

	bool isready(void);
};
