/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ZEPHYR_INCLUDE_ADNS9800_H_
#define ZEPHYR_INCLUDE_ADNS9800_H_

/**
 * @file adns9800.h
 *
 * @brief Header file for the adns9800 driver.
 */

#include <zephyr/drivers/sensor.h>
#include "avago.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Timings defined by spec (in us) */
#define T_NCS_SCLK 1                    /* 120 ns (rounded to 1us?)*/
#define T_SRX (20 - T_NCS_SCLK)         /* 20 us */
#define T_SCLK_NCS_WR (35 - T_NCS_SCLK) /* 35 us */
#define T_SWX (180 - T_SCLK_NCS_WR)     /* 180 us */
#define T_SRAD 100                      /* 100 us */
#define T_SRAD_MOTBR T_SRAD             /* 100 us */
#define T_BEXIT 1                       /* 1 us */

/* Timing defined on SROM download burst mode figure */
#define T_BRSEP 15 /* 15 us */

/* Sensor registers */
#define ADNS9800_REG_PRODUCT_ID 0x00
#define ADNS9800_REG_REVISION_ID 0x01
#define ADNS9800_REG_MOTION 0x02
#define ADNS9800_REG_DELTA_X_L 0x03
#define ADNS9800_REG_DELTA_X_H 0x04
#define ADNS9800_REG_DELTA_Y_L 0x05
#define ADNS9800_REG_DELTA_Y_H 0x06
#define ADNS9800_REG_SQUAL 0x07
#define ADNS9800_REG_RAW_DATA_SUM 0x08
#define ADNS9800_REG_MAXIMUM_RAW_DATA 0x09
#define ADNS9800_REG_MINIMUM_RAW_DATA 0x0A
#define ADNS9800_REG_SHUTTER_LOWER 0x0B
#define ADNS9800_REG_SHUTTER_UPPER 0x0C
#define ADNS9800_REG_CONTROL 0x0D
#define ADNS9800_REG_CONFIG1 0x0F
#define ADNS9800_REG_CONFIG2 0x10
#define ADNS9800_REG_ANGLE_TUNE 0x11
#define ADNS9800_REG_FRAME_CAPTURE 0x12
#define ADNS9800_REG_SROM_ENABLE 0x13
#define ADNS9800_REG_RUN_DOWNSHIFT 0x14
#define ADNS9800_REG_REST1_RATE 0x15
#define ADNS9800_REG_REST1_DOWNSHIFT 0x16
#define ADNS9800_REG_REST2_RATE 0x17
#define ADNS9800_REG_REST2_DOWNSHIFT 0x18
#define ADNS9800_REG_REST3_RATE 0x19
#define ADNS9800_REG_OBSERVATION 0x24
#define ADNS9800_REG_DATA_OUT_LOWER 0x25
#define ADNS9800_REG_DATA_OUT_UPPER 0x26
#define ADNS9800_REG_RAW_DATA_DUMP 0x29
#define ADNS9800_REG_SROM_ID 0x2A
#define ADNS9800_REG_MIN_SQ_RUN 0x2B
#define ADNS9800_REG_RAW_DATA_THRESHOLD 0x2C
#define ADNS9800_REG_CONFIG5 0x2F
#define ADNS9800_REG_CONFIG4 0x39
#define ADNS9800_REG_POWER_UP_RESET 0x3A
#define ADNS9800_REG_SHUTDOWN 0x3B
#define ADNS9800_REG_INVERSE_PRODUCT_ID 0x3F
#define ADNS9800_REG_LIFTCUTOFF_TUNE3 0x41
#define ADNS9800_REG_ANGLE_SNAP 0x42
#define ADNS9800_REG_LIFTCUTOFF_TUNE1 0x4A
#define ADNS9800_REG_MOTION_BURST 0x50
#define ADNS9800_REG_LIFTCUTOFF_TUNE_TIMEOUT 0x58
#define ADNS9800_REG_LIFTCUTOFF_TUNE_MIN_LENGTH 0x5A
#define ADNS9800_REG_SROM_LOAD_BURST 0x62
#define ADNS9800_REG_LIFT_CONFIG 0x63
#define ADNS9800_REG_RAW_DATA_BURST 0x64
#define ADNS9800_REG_LIFTCUTOFF_TUNE2 0x65
#define ADNS9800_REG_LASER_CTRL0 0x20 // ADNS-9800 only

/* Sensor identification values */
#define ADNS9800_PRODUCT_ID 0x33
#define ADNS9800_FIRMWARE_ID 0xA6

/* Power-up register commands */
#define ADNS9800_POWERUP_CMD_RESET 0x5A

/* Max register count readable in a single motion burst */
#define ADNS9800_MAX_BURST_SIZE 14

/* Register count used for reading a single motion burst */
#define ADNS9800_BURST_SIZE 6

/* Position of X in motion burst data */
#define ADNS9800_DX_POS 2
#define ADNS9800_DY_POS 4

/* Position of SQUAL in motion burst data */
// #define ADNS9800_SQUAL_POS 6 // uncomment to log surface quality
#ifdef ADNS9800_SQUAL_POS
#undef ADNS9800_BURST_SIZE
#define ADNS9800_BURST_SIZE 7
#endif

/* Rest_En position in Config2 register. */
#define ADNS9800_REST_EN_POS 5

#define ADNS9800_MAX_CPI 8200
#define ADNS9800_MIN_CPI 50
#define ADNS9800_STP_CPI 50

#define SPI_WRITE_BIT BIT(7)

/* Helper macros used to convert sensor values. */
#define ADNS9800_SVALUE_TO_CPI(svalue) ((uint32_t)(svalue).val1)
#define ADNS9800_SVALUE_TO_TIME(svalue) ((uint32_t)(svalue).val1)
#define ADNS9800_SVALUE_TO_BOOL(svalue) ((svalue).val1 != 0)

#ifdef CONFIG_ADNS9800_INVERT_SCROLL_X
#define ADNS9800_SCROLL_X_NEG 1
#define ADNS9800_SCROLL_X_POS -1
#else
#define ADNS9800_SCROLL_X_NEG -1
#define ADNS9800_SCROLL_X_POS 1
#endif

#ifdef CONFIG_ADNS9800_INVERT_SCROLL_Y
#define ADNS9800_SCROLL_Y_NEG 1
#define ADNS9800_SCROLL_Y_POS -1
#else
#define ADNS9800_SCROLL_Y_NEG -1
#define ADNS9800_SCROLL_Y_POS 1
#endif

/** @brief Sensor specific attributes of ADNS9800. */
enum adns9800_attribute {

	/** Sensor CPI for both X and Y axes. */
	ADNS9800_ATTR_CPI,

	/** Enable or disable sleep modes. */
	ADNS9800_ATTR_REST_ENABLE,

	/** Entering time from Run mode to REST1 mode [ms]. */
	ADNS9800_ATTR_RUN_DOWNSHIFT_TIME,

	/** Entering time from REST1 mode to REST2 mode [ms]. */
	ADNS9800_ATTR_REST1_DOWNSHIFT_TIME,

	/** Entering time from REST2 mode to REST3 mode [ms]. */
	ADNS9800_ATTR_REST2_DOWNSHIFT_TIME,

	/** Sampling frequency time during REST1 mode [ms]. */
	ADNS9800_ATTR_REST1_SAMPLE_TIME,

	/** Sampling frequency time during REST2 mode [ms]. */
	ADNS9800_ATTR_REST2_SAMPLE_TIME,

	/** Sampling frequency time during REST3 mode [ms]. */
	ADNS9800_ATTR_REST3_SAMPLE_TIME,

};


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_ADNS9800_H_ */
