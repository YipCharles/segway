#ifndef CONFIGURE_H
#define CONFIGURE_H

/* change it if needed */
#define FIRMWARE_VERSION				0x0002
#define HARDWARE_VERSION				0xE201		// still deferent from hw group
#define VECT_TAB_OFFSET					0x0000

/* fixed */
// version
#define GET_PRJ_VERSION()				((HARDWARE_VERSION & 0xFFFF) >> 8)
#define GET_HW_VERSION()				((HARDWARE_VERSION & 0xFF))

// flash
#define FLASH_CONF_ADDR					0x080E0000
#define FLASH_APP_START_ADDR			(0x08000000 + VECT_TAB_OFFSET)
#define FLASH_MAGIC_ADDR_OFFSET			0x800

// enc
#define ENC_SAMPLE_RATE					10000
#define ENC_ANGLE_RESOLUTION			(16384)
#define LPF_ENC_RAW_SAMPLE_RATE			ENC_SAMPLE_RATE
#define LPF_ENC_RAW_CUTOFF_FREQ			800
#define LPF_ENC_SPD_SAMPLE_RATE			ENC_SAMPLE_RATE
#define LPF_ENC_SPD_CUTOFF_FREQ			10

#define CTRL_FREQENCY					1000
#endif
