#include "mpu9250.h"

#define SELF_TEST_X_GYRO 0x0
#define SELF_TEST_Y_GYRO 0x1
#define SELF_TEST_Z_GYRO 0x2
#define MAG_HXL 0x03
#define MAG_HXH 0x04
#define MAG_HYL 0x05
#define MAG_HYH 0x06
#define MAG_HZL 0x07
#define MAG_HZH 0x08
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F
#define MAG_ASAX 0x10
#define MAG_ASAY 0x11
#define MAG_ASAZ 0x12
#define XG_OFFS_USRH 0x13
#define XG_OFFS_USRL 0x14
#define YG_OFFS_USRH 0x15
#define YG_OFFS_USRL 0x16
#define ZG_OFFS_USRH 0x17
#define ZG_OFFS_USRL 0x18
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG_2 0x1D
#define LP_MODE_CFG 0x1E
#define ACCEL_WOM_X_THR 0x20
#define ACCEL_WOM_Y_THR 0x21
#define ACCEL_WOM_Z_THR 0x22
#define FIFO_EN 0x23
#define FSYNC_INT 0x36
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define DMP_INT_STATUS 0x39
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define SIGNAL_PATH_RESET 0x68
#define ACCEL_INTEL_CTRL 0x69
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

#define CLEAR_BUFFER(__B) memset(__B, 0, sizeof(__B))
#define DELAY HAL_Delay


void MPU9250::cs(bool en)
{
	if (en)
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

inline uint8_t MPU9250::spi_read_write_byte(uint8_t data)
{
	uint8_t tx = data, rx;

	HAL_SPI_TransmitReceive(spi, &tx, &rx, 1, 1);
	return rx;
}

void MPU9250::spi_read(uint8_t addr, uint8_t *buffer, uint32_t size)
{
	spi_read_write_byte(0x80 | addr);
	for (uint32_t i = 0; i < size; i++)
	{
		buffer[i] = spi_read_write_byte(NULL);
	}
}

void MPU9250::spi_write(uint8_t addr, uint8_t *buffer, uint32_t size)
{
	spi_read_write_byte(addr);
	for (uint32_t i = 0; i < size; i++)
	{
		spi_read_write_byte(buffer[i]);
	}
}

void MPU9250::reg_read(uint8_t addr, uint8_t *buffer, uint32_t size)
{
	cs(true);
	spi_read(addr, buffer, size);
	cs(false);
}

void MPU9250::reg_write(uint8_t addr, uint8_t *buffer, uint32_t size)
{
	cs(true);
	spi_write(addr, buffer, size);
	cs(false);
}

MPU9250::MPU9250(SPI_HandleTypeDef *hspi, GPIO_TypeDef *hport, uint16_t hpin)
{
	spi = hspi;
	port = hport;
	pin = hpin;

	ready = false;
}

bool MPU9250::init()
{
	uint8_t buffer[10];

	cs(false);
	CLEAR_BUFFER(buffer);

	while (1)
	{
		reg_read(WHO_AM_I, buffer, 1);
		if (buffer[0] == 0x71)
			break;
		DELAY(100);
	}

	CLEAR_BUFFER(buffer);
	buffer[0] = 0x80;
	reg_write(PWR_MGMT_1, buffer, 1);
	DELAY(50);

	// gyro z clock as main clock
	CLEAR_BUFFER(buffer);
	buffer[0] = 0x03;
	reg_write(PWR_MGMT_1, buffer, 1);
	DELAY(1);

	// SAMPLE_RATE = 1Khz / (1 + SMPLRT_DIV)
	CLEAR_BUFFER(buffer);
	buffer[0] = 0x00;
	reg_write(SMPLRT_DIV, buffer, 1);
	DELAY(1);

	// TODO
	CLEAR_BUFFER(buffer);
	buffer[0] = 0x00;
	reg_write(CONFIG, buffer, 1);
	DELAY(1);

	// acc scale: 8G
	CLEAR_BUFFER(buffer);
	buffer[0] = 0x10;
	reg_write(ACCEL_CONFIG, buffer, 1);
	DELAY(1);

	// gyro scale: 500dps
	// DLPF: 0 (gyro sample rate: 8Khz)
	CLEAR_BUFFER(buffer);
	buffer[0] = 0x08;
	reg_write(GYRO_CONFIG, buffer, 1);
	DELAY(1);

	ready = true;
	
	return true;
}

void MPU9250::read(mpu_t *sensor)
{
	if (!ready)
		return;

	uint8_t buffer[14];
	
	reg_read(ACCEL_XOUT_H, buffer, 14);

	sensor->gyro[0] = (int16_t)buffer[8] << 8 | buffer[9];
	sensor->gyro[1] = (int16_t)buffer[10] << 8 | buffer[11];
	sensor->gyro[2] = (int16_t)buffer[12] << 8 | buffer[13];
	sensor->accel[0] = (int16_t)buffer[0] << 8 | buffer[1];
	sensor->accel[1] = (int16_t)buffer[2] << 8 | buffer[3];
	sensor->accel[2] = (int16_t)buffer[4] << 8 | buffer[5];
	sensor->temp = (int16_t)buffer[6] << 8 | buffer[7];

//	reg_read(MAG_HXL, buffer, 6);
//	sensor->magnet[0] = (int16_t)buffer[0] << 8 | buffer[1];
//	sensor->magnet[1] = (int16_t)buffer[2] << 8 | buffer[3];
//	sensor->magnet[2] = (int16_t)buffer[4] << 8 | buffer[5];
}

//void MPU9250::read_request(mpu_t *sensor)
//{

//}