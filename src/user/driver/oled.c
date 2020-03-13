#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"
#include "oled.h"
#include "oledfont.h"

#define I2C_DEV_ADDR 0x78
#define OLED_CMD 0 
#define OLED_DATA 1
#define OLED_MODE 0

static uint8_t GraphicBuffer[128 * 32 / 8];

void oled_reg_read(uint8_t reg, uint8_t *buffer, uint8_t size)
{
	HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, &reg, 1, 1);
	HAL_I2C_Master_Receive(&hi2c1, I2C_DEV_ADDR | 0x01, buffer, size, 2);
}

void oled_reg_write(uint8_t reg, uint8_t *buffer, uint8_t size)
{
	static uint8_t buff[20];

	buff[0] = reg;
	if(size>20-1)
		size=20-1;
	memcpy(buff + 1, buffer, size);
	HAL_I2C_Master_Transmit(&hi2c1, I2C_DEV_ADDR, buff, size + 1, 1);
}

void oled_cmd(uint8_t *buffer, uint8_t size)
{
	oled_reg_write(0x00, buffer, size);
}

void oled_data(uint8_t *buffer, uint8_t size)
{
	oled_reg_write(0x40, buffer, size);
}

void OLED_WR_Byte(uint8_t c, uint8_t is_data) 
{
	if (is_data)
		oled_data(&c, 1);
	else
		oled_cmd(&c, 1);
}





//坐标设置
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	OLED_WR_Byte(0xb0 + y, OLED_CMD);
	OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
	OLED_WR_Byte((x & 0x0f), OLED_CMD);
}
//开启OLED显示
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D, OLED_CMD); //SET DCDC命令
	OLED_WR_Byte(0X14, OLED_CMD); //DCDC ON
	OLED_WR_Byte(0XAF, OLED_CMD); //DISPLAY ON
}
//关闭OLED显示
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D, OLED_CMD); //SET DCDC命令
	OLED_WR_Byte(0X10, OLED_CMD); //DCDC OFF
	OLED_WR_Byte(0XAE, OLED_CMD); //DISPLAY OFF
}
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		OLED_WR_Byte(0xb0 + i, OLED_CMD); //设置页地址（0~7）
		OLED_WR_Byte(0x00, OLED_CMD);	 //设置显示位置—列低地址
		OLED_WR_Byte(0x10, OLED_CMD);	 //设置显示位置—列高地址
		for (n = 0; n < 128; n++)
			OLED_WR_Byte(0, OLED_DATA);
	} //更新显示
}
void OLED_On(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		OLED_WR_Byte(0xb0 + i, OLED_CMD); //设置页地址（0~7）
		OLED_WR_Byte(0x00, OLED_CMD);	 //设置显示位置—列低地址
		OLED_WR_Byte(0x10, OLED_CMD);	 //设置显示位置—列高地址
		for (n = 0; n < 128; n++)
			OLED_WR_Byte(1, OLED_DATA);
	} //更新显示
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size)
{
	unsigned char c = 0, i = 0;
	c = chr - ' '; //得到偏移后的值
	if (x > 128 - 1)
	{
		x = 0;
		y = y + 2;
	}
	if (Char_Size == 16)
	{
		OLED_Set_Pos(x, y);
		for (i = 0; i < 8; i++)
			OLED_WR_Byte(F8X16[c * 16 + i], OLED_DATA);
		OLED_Set_Pos(x, y + 1);
		for (i = 0; i < 8; i++)
			OLED_WR_Byte(F8X16[c * 16 + i + 8], OLED_DATA);
	}
	else
	{
		OLED_Set_Pos(x, y);
		for (i = 0; i < 6; i++)
			OLED_WR_Byte(F6x8[c][i], OLED_DATA);
	}
}
//m^n函数
uint32_t oled_pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while (n--)
		result *= m;
	return result;
}
//显示2个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2)
{
	uint8_t t, temp;
	uint8_t enshow = 0;
	for (t = 0; t < len; t++)
	{
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2);
				continue;
			}
			else
				enshow = 1;
		}
		OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2);
	}
}
//显示一个字符号串
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t Char_Size)
{
	unsigned char j = 0;
	while (chr[j] != '\0')
	{
		OLED_ShowChar(x, y, chr[j], Char_Size);
		x += 8;
		if (x > 120)
		{
			x = 0;
			y += 2;
		}
		j++;
	}
}
//显示汉字
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
	uint8_t t, adder = 0;
	OLED_Set_Pos(x, y);
	for (t = 0; t < 16; t++)
	{
		OLED_WR_Byte(Hzk[2 * no][t], OLED_DATA);
		adder += 1;
	}
	OLED_Set_Pos(x, y + 1);
	for (t = 0; t < 16; t++)
	{
		OLED_WR_Byte(Hzk[2 * no + 1][t], OLED_DATA);
		adder += 1;
	}
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
	unsigned int j = 0;
	unsigned char x, y;

	if (y1 % 8 == 0)
		y = y1 / 8;
	else
		y = y1 / 8 + 1;
	for (y = y0; y < y1; y++)
	{
		OLED_Set_Pos(x0, y);
		for (x = x0; x < x1; x++)
		{
			OLED_WR_Byte(BMP[j++], OLED_DATA);
		}
	}
}

void oled_init(void)
{
	vTaskDelay(100);

	OLED_WR_Byte(0xAE, OLED_CMD); //关闭显示

	OLED_WR_Byte(0x40, OLED_CMD); //---set low column address
	OLED_WR_Byte(0xB0, OLED_CMD); //---set high column address

	OLED_WR_Byte(0xC8, OLED_CMD); //-not offset

	OLED_WR_Byte(0x81, OLED_CMD); //设置对比度
	OLED_WR_Byte(0xff, OLED_CMD);

	OLED_WR_Byte(0xa1, OLED_CMD); //段重定向设置

	OLED_WR_Byte(0xa6, OLED_CMD); //

	OLED_WR_Byte(0xa8, OLED_CMD); //设置驱动路数
	OLED_WR_Byte(0x1f, OLED_CMD);

	OLED_WR_Byte(0xd3, OLED_CMD);
	OLED_WR_Byte(0x00, OLED_CMD);

	OLED_WR_Byte(0xd5, OLED_CMD);
	OLED_WR_Byte(0xf0, OLED_CMD);

	OLED_WR_Byte(0xd9, OLED_CMD);
	OLED_WR_Byte(0x22, OLED_CMD);

	OLED_WR_Byte(0xda, OLED_CMD);
	OLED_WR_Byte(0x02, OLED_CMD);

	OLED_WR_Byte(0xdb, OLED_CMD);
	OLED_WR_Byte(0x49, OLED_CMD);

	OLED_WR_Byte(0x8d, OLED_CMD);
	OLED_WR_Byte(0x14, OLED_CMD);

	OLED_WR_Byte(0xaf, OLED_CMD);
	OLED_Clear();

}

// x0: 0~127
// y0: 0~31
void oled_point_write(uint32_t x0, uint32_t y0, bool ink)
{
//	uint32_t y;

//	x=
//	y = y0 / 8;

//	if(ink)
//		GraphicBuffer[x0 + y];

//	uint32_t j = 0;
//	uint32_t x, y;

//	if (y1 % 8 == 0)
//		y = y1 / 8;
//	else
//		y = y1 / 8 + 1;
//	for (y = y0; y < y1; y++)
//	{
//		OLED_Set_Pos(x0, y);
//		for (x = x0; x < x1; x++)
//		{
//			OLED_WR_Byte(BMP[j++], OLED_DATA);
//		}
//	}
}

void oled_update(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, uint8_t *buffer)
{


}


void oled_handle(void)
{
	OLED_DrawBMP(0, 0, 127, 7, GraphicBuffer);
}


