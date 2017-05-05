

// Modified for personal project by Keelin Wheeler in March 2017:
//  This code is converted to C from an arduino library at https://github.com/kriswiner/MPU-9250
//  The details of this code need not be discussed or commented in much detail,
//    what the code does is get readings from the IMU and converts to yaw, pitch, roll format


/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND

 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */

#ifndef MPU9250_H
#define MPU9250_H

#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "twi_lib.h"
#include "BT.h"
#include "timer.h"

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define MPU9250_SELF_TEST_X_GYRO 0x00
#define MPU9250_SELF_TEST_Y_GYRO 0x01
#define MPU9250_SELF_TEST_Z_GYRO 0x02

/*#define MPU9250_X_FINE_GAIN      0x03 // [7:0] fine gain
#define MPU9250_Y_FINE_GAIN      0x04
#define MPU9250_Z_FINE_GAIN      0x05
#define MPU9250_XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define MPU9250_XA_OFFSET_L_TC   0x07
#define MPU9250_YA_OFFSET_H      0x08
#define MPU9250_YA_OFFSET_L_TC   0x09
#define MPU9250_ZA_OFFSET_H      0x0A
#define MPU9250_ZA_OFFSET_L_TC   0x0B */

#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F

#define MPU9250_SELF_TEST_A      0x10

#define MPU9250_XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define MPU9250_XG_OFFSET_L      0x14
#define MPU9250_YG_OFFSET_H      0x15
#define MPU9250_YG_OFFSET_L      0x16
#define MPU9250_ZG_OFFSET_H      0x17
#define MPU9250_ZG_OFFSET_L      0x18
#define MPU9250_SMPLRT_DIV       0x19
#define MPU9250_CONFIG           0x1A
#define MPU9250_GYRO_CONFIG      0x1B
#define MPU9250_ACCEL_CONFIG     0x1C
#define MPU9250_ACCEL_CONFIG2    0x1D
#define MPU9250_LP_ACCEL_ODR     0x1E
#define MPU9250_WOM_THR          0x1F

#define MPU9250_MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU9250_ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define MPU9250_ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define MPU9250_FIFO_EN          0x23
#define MPU9250_I2C_MST_CTRL     0x24
#define MPU9250_I2C_SLV0_ADDR    0x25
#define MPU9250_I2C_SLV0_REG     0x26
#define MPU9250_I2C_SLV0_CTRL    0x27
#define MPU9250_I2C_SLV1_ADDR    0x28
#define MPU9250_I2C_SLV1_REG     0x29
#define MPU9250_I2C_SLV1_CTRL    0x2A
#define MPU9250_I2C_SLV2_ADDR    0x2B
#define MPU9250_I2C_SLV2_REG     0x2C
#define MPU9250_I2C_SLV2_CTRL    0x2D
#define MPU9250_I2C_SLV3_ADDR    0x2E
#define MPU9250_I2C_SLV3_REG     0x2F
#define MPU9250_I2C_SLV3_CTRL    0x30
#define MPU9250_I2C_SLV4_ADDR    0x31
#define MPU9250_I2C_SLV4_REG     0x32
#define MPU9250_I2C_SLV4_DO      0x33
#define MPU9250_I2C_SLV4_CTRL    0x34
#define MPU9250_I2C_SLV4_DI      0x35
#define MPU9250_I2C_MST_STATUS   0x36
#define MPU9250_INT_PIN_CFG      0x37
#define MPU9250_INT_ENABLE       0x38
#define MPU9250_DMP_INT_STATUS   0x39  // Check DMP interrupt
#define MPU9250_INT_STATUS       0x3A
#define MPU9250_ACCEL_XOUT_H     0x3B
#define MPU9250_ACCEL_XOUT_L     0x3C
#define MPU9250_ACCEL_YOUT_H     0x3D
#define MPU9250_ACCEL_YOUT_L     0x3E
#define MPU9250_ACCEL_ZOUT_H     0x3F
#define MPU9250_ACCEL_ZOUT_L     0x40
#define MPU9250_TEMP_OUT_H       0x41
#define MPU9250_TEMP_OUT_L       0x42
#define MPU9250_GYRO_XOUT_H      0x43
#define MPU9250_GYRO_XOUT_L      0x44
#define MPU9250_GYRO_YOUT_H      0x45
#define MPU9250_GYRO_YOUT_L      0x46
#define MPU9250_GYRO_ZOUT_H      0x47
#define MPU9250_GYRO_ZOUT_L      0x48
#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_EXT_SENS_DATA_01 0x4A
#define MPU9250_EXT_SENS_DATA_02 0x4B
#define MPU9250_EXT_SENS_DATA_03 0x4C
#define MPU9250_EXT_SENS_DATA_04 0x4D
#define MPU9250_EXT_SENS_DATA_05 0x4E
#define MPU9250_EXT_SENS_DATA_06 0x4F
#define MPU9250_EXT_SENS_DATA_07 0x50
#define MPU9250_EXT_SENS_DATA_08 0x51
#define MPU9250_EXT_SENS_DATA_09 0x52
#define MPU9250_EXT_SENS_DATA_10 0x53
#define MPU9250_EXT_SENS_DATA_11 0x54
#define MPU9250_EXT_SENS_DATA_12 0x55
#define MPU9250_EXT_SENS_DATA_13 0x56
#define MPU9250_EXT_SENS_DATA_14 0x57
#define MPU9250_EXT_SENS_DATA_15 0x58
#define MPU9250_EXT_SENS_DATA_16 0x59
#define MPU9250_EXT_SENS_DATA_17 0x5A
#define MPU9250_EXT_SENS_DATA_18 0x5B
#define MPU9250_EXT_SENS_DATA_19 0x5C
#define MPU9250_EXT_SENS_DATA_20 0x5D
#define MPU9250_EXT_SENS_DATA_21 0x5E
#define MPU9250_EXT_SENS_DATA_22 0x5F
#define MPU9250_EXT_SENS_DATA_23 0x60
#define MPU9250_MOT_DETECT_STATUS 0x61
#define MPU9250_I2C_SLV0_DO      0x63
#define MPU9250_I2C_SLV1_DO      0x64
#define MPU9250_I2C_SLV2_DO      0x65
#define MPU9250_I2C_SLV3_DO      0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET  0x68
#define MPU9250_MOT_DETECT_CTRL  0x69
#define MPU9250_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define MPU9250_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define MPU9250_PWR_MGMT_2       0x6C
#define MPU9250_DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define MPU9250_DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define MPU9250_DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define MPU9250_DMP_REG_1        0x70
#define MPU9250_DMP_REG_2        0x71
#define MPU9250_FIFO_COUNTH      0x72
#define MPU9250_FIFO_COUNTL      0x73
#define MPU9250_FIFO_R_W         0x74
#define WHO_AM_I_MPU9250          0x75 // Should return 0x71
#define MPU9250_XA_OFFSET_H      0x77
#define MPU9250_XA_OFFSET_L      0x78
#define MPU9250_YA_OFFSET_H      0x7A
#define MPU9250_YA_OFFSET_L      0x7B
#define MPU9250_ZA_OFFSET_H      0x7D
#define MPU9250_ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0

// Set initial input parameters
#define MPU9250_AFS_2G 0
#define MPU9250_AFS_4G 1
#define MPU9250_AFS_8G 2
#define MPU9250_AFS_16G 3

#define MPU9250_GFS_250DPS 0
#define MPU9250_GFS_500DPS 1
#define MPU9250_GFS_1000DPS 2
#define MPU9250_GFS_2000DPS 3

#define MPU9250_MFS_14BITS 0 // 0.6 mG per LSB
#define MPU9250_MFS_16BITS 1 // 0.15 mG per LSB

#define PI 3.14159265358979323846f
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

void mpu9250_init(uint8_t);
uint8_t mpu9250_dataReady(void);
void mpu9250_readData(float* Yaw, float* Pitch, float* Roll);

#endif
