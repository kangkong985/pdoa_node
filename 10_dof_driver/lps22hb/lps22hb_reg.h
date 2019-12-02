// Inspired by Kris Winer (https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB)
// Adapted for NRF52 by Niklas Casaril <niklas@loligoelectronics.com>

#define LPS22HB_INTERRUPT_CFG 	0x0b
#define LPS22HB_THIS_P_L 	0x0c
#define LPS22HB_THIS_P_H 	0x0d
#define LPS22HB_WHO_AM_I  	0x0f	
#define LPS22HB_WHO_AM_I_RES 0b10110001

#define LPS22HB_CTRL_REG1	0x10
#define LPS22HB_CTRL_REG2	0x11
#define LPS22HB_CTRL_REG3	0x12
#define LPS22HB_FIFO_CTRL 	0x14
#define LPS22HB_REF_P_XL 	0x15
#define LPS22HB_REF_P_L		0x16
#define LPS22HB_REF_P_H		0x17
#define LPS22HB_RPDS_L		0x18
#define LPS22HB_RPDS_H		0x19
#define LPS22HB_RES_CONF	0x1A
#define LPS22HB_INT_SOURCE	0x25
#define LPS22HB_FIFO_STATUS	0x26
#define LPS22HB_STATUS		0x27
#define LPS22HB_PRESS_OUT_XL	0x28
#define LPS22HB_PRESS_OUT_L	0x29
#define LPS22HB_PRESS_OUT_H	0x2A
#define LPS22HB_TEMP_OUT_L	0x2B
#define LPS22HB_TEMP_OUT_H	0x2C
#define LPS22HB_LPFP_RES	0x33

#define LPS22HB_REG2_ONESHOT (1<<0)
#define LPS22HB_REG2_RESET (1<<2)
#define LPS22HB_REG3_DRDY (1<<2)

