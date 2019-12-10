/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file InvenSense_ICM20602_registers.hpp
 *
 * Invensense ICM-20602 registers.
 *
 */

#pragma once

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace InvenSense_ICM20948
{
static constexpr uint32_t SPI_SPEED = 7 * 1000 * 1000; // 7 MHz SPI
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHOAMI = 0xEA;

namespace Register
{

enum class BANK_0 : uint8_t {
	WHO_AM_I             = 0x00,

	USER_CTRL            = 0x03,

	PWR_MGMT_1           = 0x06,

	I2C_MST_STATUS       = 0x17,

	EXT_SLV_SENS_DATA_00 = 0x3B,
	EXT_SLV_SENS_DATA_01 = 0x3C,
	EXT_SLV_SENS_DATA_02 = 0x3D,
	EXT_SLV_SENS_DATA_03 = 0x3E,
	EXT_SLV_SENS_DATA_04 = 0x3F,
	EXT_SLV_SENS_DATA_05 = 0x40,
	EXT_SLV_SENS_DATA_06 = 0x41,
	EXT_SLV_SENS_DATA_07 = 0x42,
	EXT_SLV_SENS_DATA_08 = 0x43,


	FIFO_EN_1            = 0x66,
	FIFO_EN_2            = 0x67,
	FIFO_RST             = 0x68,

	FIFO_COUNTH          = 0x70,
	FIFO_COUNTL          = 0x71,
	FIFO_R_W             = 0x72,

	REG_BANK_SEL         = 0x7F,
};

enum class BANK_1 : uint8_t {
	// Self test registers
	REG_BANK_SEL         = 0x7F,
};

enum class BANK_2 : uint8_t {
	GYRO_SMPLRT_DIV      = 0x00,
	GYRO_CONFIG_1        = 0x01,

	ACCEL_CONFIG         = 0x14,

	REG_BANK_SEL         = 0x7F,
};

enum class BANK_3 : uint8_t {
	I2C_MST_ODR_CONFIG   = 0x00,
	I2C_MST_CTRL         = 0x01,

	I2C_SLV0_ADDR        = 0x03,
        I2C_SLV0_REG         = 0x04,
	I2C_SLV0_CTRL        = 0x05,
	I2C_SLV0_DO          = 0x06,
	I2C_SLV1_ADDR        = 0x07,

	REG_BANK_SEL         = 0x7F,
};


};



// CONFIG
enum CONFIG_BIT : uint8_t {
	FIFO_WM   = Bit7,
	FIFO_MODE = Bit6, // when the FIFO is full, additional writes will not be written to FIFO

	DLPF_CFG_BYPASS_DLPF_8KHZ = 7, // Rate 8 kHz [2:0]
};

// GYRO_CONFIG
enum GYRO_CONFIG_BIT : uint8_t {
	// FS_SEL [4:3]
	FS_SEL_250_DPS	= 0,           // 0b00000
	FS_SEL_500_DPS	= Bit3,        // 0b01000
	FS_SEL_1000_DPS	= Bit4,        // 0b10000
	FS_SEL_2000_DPS	= Bit4 | Bit3, // 0b11000

	// FCHOICE_B [1:0]
	FCHOICE_B_8KHZ_BYPASS_DLPF = Bit1 | Bit0, // 0b10 - 3-dB BW: 3281 Noise BW (Hz): 3451.0   8 kHz
};

// ACCEL_CONFIG
enum ACCEL_CONFIG_BIT : uint8_t {
	// ACCEL_FS_SEL [4:3]
	ACCEL_FS_SEL_2G  = 0,           // 0b00000
	ACCEL_FS_SEL_4G  = Bit3,        // 0b01000
	ACCEL_FS_SEL_8G  = Bit4,        // 0b10000
	ACCEL_FS_SEL_16G = Bit4 | Bit3, // 0b11000
};

// ACCEL_CONFIG2
enum ACCEL_CONFIG2_BIT : uint8_t {
	ACCEL_FCHOICE_B_BYPASS_DLPF = Bit3,
};

// FIFO_EN
enum FIFO_EN_BIT : uint8_t {
	GYRO_FIFO_EN  = Bit4,
	ACCEL_FIFO_EN = Bit3,
};

// INT_ENABLE
enum INT_ENABLE_BIT : uint8_t {
	FIFO_OFLOW_EN   = Bit4,
	DATA_RDY_INT_EN = Bit0
};

// INT_STATUS
enum INT_STATUS_BIT : uint8_t {
	FIFO_OFLOW_INT = Bit4,
	DATA_RDY_INT   = Bit0,
};

// USER_CTRL
enum USER_CTRL_BIT : uint8_t {
	FIFO_EN  = Bit6,
	FIFO_RST = Bit2,
};

// PWR_MGMT_1
enum PWR_MGMT_1_BIT : uint8_t {
	DEVICE_RESET = Bit7,
	CLKSEL_2     = Bit2,
	CLKSEL_1     = Bit1,
	CLKSEL_0     = Bit0,
};


namespace FIFO
{
static constexpr size_t SIZE = 1008;

// FIFO_DATA layout when FIFO_EN has both GYRO_FIFO_EN and ACCEL_FIFO_EN set
struct DATA {
	uint8_t ACCEL_XOUT_H;
	uint8_t ACCEL_XOUT_L;
	uint8_t ACCEL_YOUT_H;
	uint8_t ACCEL_YOUT_L;
	uint8_t ACCEL_ZOUT_H;
	uint8_t ACCEL_ZOUT_L;
	uint8_t TEMP_OUT_H;
	uint8_t TEMP_OUT_L;
	uint8_t GYRO_XOUT_H;
	uint8_t GYRO_XOUT_L;
	uint8_t GYRO_YOUT_H;
	uint8_t GYRO_YOUT_L;
	uint8_t GYRO_ZOUT_H;
	uint8_t GYRO_ZOUT_L;
};
static_assert(sizeof(DATA) == 14);
}

} // namespace InvenSense_ICM20602
