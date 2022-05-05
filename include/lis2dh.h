/**
  ******************************************************************************
  * @file           : lis2dh.h
  * @author         : Mauricio Barroso Benavides
  * @date           : May 1, 2022
  * @brief          : todo: write brief 
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2022 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LIS2DH_H_
#define LIS2DH_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "esp_err.h"
#include "hal/i2c_types.h"

/* Exported define -----------------------------------------------------------*/
/**
* @brief LIS2DH I2C address
*/
#define LIS2DH_I2C_ADDRESS	0x19	/*!< I2C device address */

/**
* @brief LIS2DH registers
*/
#define LIS2DH_STATUS_REG_AUX_REG	0x07	/*!< Temperature data status register */
#define LIS2DH_OUT_TEMP_L_REG		0x0C	/*!< Temperature data (LSB) */
#define LIS2DH_OUT_TEMP_H_REG		0x0D	/*!< Temperature data (HSB) */
#define LIS2DH_WHO_AM_I_REG			0x0F	/*!< Device Identification register */
#define LIS2DH_CTRL_REG0			0x1E	/*!< Control register 0 */
#define LIS2DH_TEMP_CFG_REG			0x1F	/*!< Temperature config register */
#define LIS2DH_CTRL_REG1			0x20	/*!< Control register 1 */
#define LIS2DH_CTRL_REG2			0x21	/*!< Control register 2 */
#define LIS2DH_CTRL_REG3			0x22	/*!< Control register 3 */
#define LIS2DH_CTRL_REG4			0x23	/*!< Control register 4 */
#define LIS2DH_CTRL_REG5			0x24	/*!< Control register 5 */
#define LIS2DH_CTRL_REG6			0x25	/*!< Control register 6 */
#define LIS2DH_REFERENCE_REG		0x26	/*!< Reference value register */
#define LIS2DH_STATUS_REG			0x27	/*!< Status register */
#define LIS2DH_OUT_X_L_REG			0x28	/*!< X-axis acceleration data (LSB) */
#define LIS2DH_OUT_X_H_REG			0x29	/*!< X-axis acceleration data (MSB) */
#define LIS2DH_OUT_Y_L_REG			0x2A	/*!< Y-axis acceleration data (LSB) */
#define LIS2DH_OUT_Y_H_REG			0x2B	/*!< Y-axis acceleration data (MSB) */
#define LIS2DH_OUT_Z_L_REG			0x2C	/*!< Z-axis acceleration data (LSB) */
#define LIS2DH_OUT_Z_H_REG			0x2D	/*!< Z-axis acceleration data (MSB) */
#define LIS2DH_FIFO_CTRL_REG		0x2E	/*!< FIFO control register */
#define LIS2DH_FIFO_SRC_CTRL_REG	0x2F	/*!< FIFO source register */
#define LIS2DH_INT1_CFG_REG			0x30	/*!< Interrupt 1 config register */
#define LIS2DH_INT1_SRC_REG			0x31	/*!< Interrupt 1 source register */
#define LIS2DH_INT1_THS_REG			0x32	/*!< Interrupt 1 threshold register */
#define LIS2DH_INT1_DURATION_REG	0x33	/*!< Interrupt 1 duration register */
#define LIS2DH_INT2_CFG_REG			0x34	/*!< Interrupt 2 config register */
#define LIS2DH_INT2_SRC_REG			0x35	/*!< Interrupt 2 source register */
#define LIS2DH_INT2_THS_REG			0x36	/*!< Interrupt 2 threshold register */
#define LIS2DH_INT2_DURATION_REG	0x37	/*!< Interrupt 2 duration register */
#define LIS2DH_CLICK_CFG_REG		0x38	/*!< Click config register */
#define LIS2DH_CLICK_SRC_REG		0x39	/*!< Click source register */
#define LIS2DH_CLICK_THS_REG		0x3A	/*!< Click threshold register */
#define LIS2DH_TIME_LIMIT_REG		0x3B	/*!< Click time limit register */
#define LIS2DH_TIME_LATENCY_REG		0x3C	/*!< Click time latency register */
#define LIS2DH_TIME_WINDOW_REG		0x3D	/*!< Click time window register */
#define LIS2DH_ACT_THS_REG			0x3E	/*!< ACT threshold register */
#define LIS2DH_ACT_DUR_REG			0x3F	/*!< ACT duration register */

/* Exported macro ------------------------------------------------------------*/

/* Exported typedef ----------------------------------------------------------*/
/**
* @brief  State enable
*/
typedef enum {
    LIS2DH_DISABLE = 0,
	LIS2DH_ENABLE,
} lis2dh_state_t;

/**
* @brief  Bit status
*/
typedef enum {
	LIS2DH_RESET = 0,
	LIS2DH_SET,
} lis2dh_bit_status_t;

/**
* @brief Temperature sensor enable
*/
typedef enum {
	LIS2DH_TEMP_DISABLE = 0x00,	/*!< Temperature sensor disable*/
	LIS2DH_TEMP_ENABLE = 0x03		/*!< Temperature sensor enable */
} lis2dh_temp_en_t;

/**
* @brief  Output data rate configuration
*/
typedef enum {
	LIS2DH_OPT_NORMAL = 0x00,		/*!< Normal mode */
	LIS2DH_OPT_HIGH_RES = 0x01,		/*!< High resolution */
	LIS2DH_OPT_LOW_POWER = 0x02,	/*!< low power mode */
} lis2dh_opt_mode_t;

/**
* @brief  Output data rate configuration
*/
typedef enum {
	LIS2DH_ODR_LOW_POWER = 0x00,	/*!< Power-down mode */
	LIS2DH_ODR_1HZ = 0x01,			/*!< HR / Normal / Low-power mode (1 Hz) */
	LIS2DH_ODR_10HZ = 0x02,			/*!< HR / Normal / Low-power mode (10 Hz) */
	LIS2DH_ODR_25HZ = 0x03,			/*!< HR / Normal / Low-power mode (25 Hz) */
	LIS2DH_ODR_50HZ = 0x04,			/*!< HR / Normal / Low-power mode (50 Hz) */
	LIS2DH_ODR_100HZ = 0x05,		/*!< HR / Normal / Low-power mode (100 Hz) */
	LIS2DH_ODR_200HZ = 0x06,		/*!< HR / Normal / Low-power mode (200 Hz) */
	LIS2DH_ODR_400HZ = 0x07,		/*!< HR/ Normal / Low-power mode (400 Hz) */
	LIS2DH_ODR_1620HZ = 0x08,		/*!< Low-power mode (1.620 kHz) */
	LIS2DH_ODR_1344HZ = 0x09,		/*!< HR/ Normal (1.344 kHz); Low-power mode (5.376 kHz) */
} lis2dh_odr_t;

/**
* @brief  High-pass filter mode selection
*/
typedef enum {
	LIS2DH_HPM_NORMAL_RESET    = 0x00,	/*!< Normal mode (reset by reading REFERENCE (26h) register) */
	LIS2DH_HPM_REF_SIG_FILTER  = 0x01,	/*!< Reference signal for filtering */
	LIS2DH_HPM_NORMAL_MODE     = 0x02,	/*!< Normal mode */
	LIS2DH_HPM_AUTO_IA         = 0x03,	/*!< Autoreset on interrupt event */
} lis2dh_hpm_mode_t;

/**
* @brief  Full-scale selection
*/
typedef enum {
	LIS2DH_FS_2G = 0x00,	/*!< Full scale: +/-2g */
	LIS2DH_FS_4G = 0x01,	/*!< Full scale: +/-4g  */
	LIS2DH_FS_8G = 0x02,	/*!< Full scale: +/-8g  */
	LIS2DH_FS_16G = 0x03,	/*!< Full scale: +/-16g  */
} lis2dh_fs_t;

/**
* @brief  Self-test mode selection
*/
typedef enum {
	LIS2DH_ST_DISABLE = 0x00,	/*!< Normal mode */
	LIS2DH_ST_MODE0 = 0x01,	/*!< Self test 0  */
	LIS2DH_ST_MODE1 = 0x02,	/*!< Self test 1  */
} lis2dh_self_test_t;


/**
* @brief  LIS2DH Init structure definition.
*/
typedef struct {
    lis2dh_state_t sdo_pu_disc;		/*!< Disconnect SDO/SA0 pull-up  */
    lis2dh_temp_en_t temp_enable;	/*!< Temperature sensor enable  */
    lis2dh_odr_t odr;				/*!< Data rate selection  */
    lis2dh_opt_mode_t opt_mode;		/*!< Operating mode selection  */
    lis2dh_state_t z_enable;		/*!< Z-axis enable  */
    lis2dh_state_t y_enable;		/*!< Y-axis enable  */
    lis2dh_state_t x_enable;		/*!< X-axis enable  */
    lis2dh_hpm_mode_t hmp_mode;		/*!< High-pass filter mode selection  */
    lis2dh_state_t fds;				/*!< Filtered data selection  */
    lis2dh_state_t bdu_status;		/*!< Block data update  */
    lis2dh_state_t ble_status;		/*!< Big/Little Endian data selection, can be activated only in high-resolution mode  */
    lis2dh_fs_t fs;					/*!< Full-scale selection  */
    lis2dh_state_t fifo_enable;		/*!< FIFO enable  */
} lis2dh_config_t;

typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} lis2dh_raw_acce_value_t;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} lis2dh_acce_value_t;

typedef struct {
	uint8_t dev_addr;
	i2c_port_t i2c_num;
	lis2dh_config_t config;
} lis2dh_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Init sensor object
 *
 * @param me Pointer to sensor object
 * @param dev_addr I2C device address
 * @param i2c_num I2C port number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_init(lis2dh_t * const me, uint8_t dev_addr, i2c_port_t i2c_num);

/**
 * @brief Get device identification of LIS2DH
 *
 * @param me Pointer to sensor object
 * @param device_id a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_get_device_id(lis2dh_t * const me, uint8_t * device_id);

/**
 * @brief Set configration of LIS2DH
 *
 * @param me Pointer to sensor object
 * @param config A structure pointer of configration
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_set_config(lis2dh_t * const me, lis2dh_config_t * config);

/**
 * @brief Get configration of LIS2DH
 *
 * @param me Pointer to sensor object
 * @param config a structure pointer of configration
 *
 * @return
 *     - ESP_OK Success
 *     - others Fail
 */
esp_err_t lis2dh_get_config(lis2dh_t * const me, lis2dh_config_t * config);

/**
 * @brief Enable the temperature sensor of LIS2DH
 *
 * @param me Pointer to sensor object
 * @param temp_en temperature enable status
 *
 * @return
 *     - ESP_OK Success
 *     - others Fail
 */
esp_err_t lis2dh_set_temp_enable(lis2dh_t * const me, lis2dh_temp_en_t temp_en);

/**
 * @brief Set output data rate
 *
 * @param me Pointer to sensor object
 * @param odr output data rate value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_set_odr(lis2dh_t * const me, lis2dh_odr_t odr);

/**
 * @brief Enable z axis
 *
 * @param me Pointer to sensor object
 * @param status enable status
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_set_z_enable(lis2dh_t * const me, lis2dh_state_t status);

/**
 * @brief Enable y axis
 *
 * @param me Pointer to sensor object
 * @param status enable status
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_set_y_enable(lis2dh_t * const me, lis2dh_state_t status);

/**
 * @brief Enable x axis
 *
 * @param me Pointer to sensor object
 * @param status enable status
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_set_x_enable(lis2dh_t * const me, lis2dh_state_t status);

/**
 * @brief Enable block data update
 *
 * @param me Pointer to sensor object
 * @param status enable status
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_set_bdumode(lis2dh_t * const me, lis2dh_state_t status);

/**
 * @brief Set full scale
 *
 * @param me Pointer to sensor object
 * @param fs full scale value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_set_fs(lis2dh_t * const me, lis2dh_fs_t fs);

/**
 * @brief Get full scale
 *
 * @param me Pointer to sensor object
 * @param fs full scale value
 * @return esp_err_t
 *  -  - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_get_fs(lis2dh_t * const me, lis2dh_fs_t * fs);
/**
 * @brief Set operation mode
 *
 * @param me Pointer to sensor object
 * @param opt_mode operation mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_set_opt_mode(lis2dh_t * const me, lis2dh_opt_mode_t opt_mode);

/**
 * @brief Get x axis acceleration
 *
 * @param me Pointer to sensor object
 * @param x_acc a poionter of x axis acceleration
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_get_x_acc(lis2dh_t * const me, uint16_t * x_acc);

/**
 * @brief Get y axis acceleration
 *
 * @param me Pointer to sensor object
 * @param y_acc a poionter of y axis acceleration
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_get_y_acc(lis2dh_t * const me, uint16_t * y_acc);
/**
 * @brief Get z axis acceleration
 *
 * @param me Pointer to sensor object
 * @param z_acc a poionter of z axis acceleration
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t lis2dh_get_z_acc(lis2dh_t * const me, uint16_t * z_acc);

/**
 * @brief Read raw accelerometer measurements
 *
 * @param me Pointer to sensor object
 * @param raw_acce_value raw accelerometer measurements value
 * @return esp_err_t
 */
esp_err_t lis2dh_get_raw_acce(lis2dh_t * const me, lis2dh_raw_acce_value_t * raw_acce_value);

/**
 * @brief Read accelerometer measurements
 *
 * @param me Pointer to sensor object
 * @param acce_value accelerometer measurements， g
 * @return esp_err_t
 */
esp_err_t lis2dh_get_acce(lis2dh_t * const me, lis2dh_acce_value_t * acce_value);

#ifdef __cplusplus
}
#endif

#endif /* LIS2DH_H_ */

/***************************** END OF FILE ************************************/
