/**
  ******************************************************************************
  * @file           : lis2dh.c
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

/* Includes ------------------------------------------------------------------*/
#include "lis2dh.h"
#include "esp_log.h"
#include "driver/i2c.h"

/* Private define ------------------------------------------------------------*/
/**
* @brief I2C protocol bit values
*/
#define WRITE_BIT		I2C_MASTER_WRITE	/*!< I2C master write */
#define READ_BIT		I2C_MASTER_READ		/*!< I2C master read */
#define ACK_CHECK_EN	(0x1)					/*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS	(0x0)					/*!< I2C master will not check ack from slave */
#define ACK_VAL			(0x0)					/*!< I2C ack value */
#define NACK_VAL		(0x1)					/*!< I2C nack value */

/**
* @brief
*/
#define I2C_TIMEOUT_MS	(1000)

/* Private macro -------------------------------------------------------------*/
/**
* @brief  Bitfield positioning.
*/
#define LIS2DH_BIT(x)	(x)

/**
* @brief  I2C address.
*/
#define LIS2DH_I2C_MULTI_REG_ONCE	0x80 /*If read/write multipul register once*/

/**
* @brief  Temperature data status register
*/
#define LIS2DH_TOR_BIT	LIS2DH_BIT(6)
#define LIS2DH_TDA_BIT	LIS2DH_BIT(2)

#define LIS2DH_TOR_MASK	0x40
#define LIS2DH_TDA_MASK	0x04

/**
* @brief Device Identification value.
*/
#define LIS2DH_WHO_AM_I_VAL	0x33

/**
* @brief  Control register 0
*/
#define LIS2DH_SDO_PDU_DISC_BIT		LIS2DH_BIT(7)
#define LIS2DH_SDO_PDU_DISC_MASK	0x80

/**
* @brief  Temperature config register
*/
#define LIS2DH_TEMP_EN_BIT	LIS2DH_BIT(6)
#define LIS2DH_TEMP_EN_MASK	0xC0

/**
* @brief  Control register 1
*/
#define LIS2DH_ODR_BIT		LIS2DH_BIT(4)
#define LIS2DH_LP_EN_BIT	LIS2DH_BIT(3)
#define LIS2DH_Z_EN_BIT		LIS2DH_BIT(2)
#define LIS2DH_Y_EN_BIT		LIS2DH_BIT(1)
#define LIS2DH_X_EN_BIT		LIS2DH_BIT(0)

#define LIS2DH_ODR_MASK		0xF0
#define LIS2DH_LP_EN_MASK	0x08
#define LIS2DH_Z_EN_MASK	0x04
#define LIS2DH_Y_EN_MASK	0x02
#define LIS2DH_X_EN_MASK	0x01

/**
* @brief  Control register 2
*/
#define LIS2DH_HMP_BIT		LIS2DH_BIT(6)
#define LIS2DH_HPCF_BIT		LIS2DH_BIT(4)
#define LIS2DH_FDS_BIT		LIS2DH_BIT(3)
#define LIS2DH_HPCLICK_BIT	LIS2DH_BIT(2)
#define LIS2DH_HP_IA2_BIT	LIS2DH_BIT(1)
#define LIS2DH_HP_IA1_BIT	LIS2DH_BIT(0)

#define LIS2DH_HMP_MASK		0xC0
#define LIS2DH_HPCF_MASK	0x30
#define LIS2DH_FDS_MASK		0x08
#define LIS2DH_HPCLICK_MASK	0x04
#define LIS2DH_HP_IA2_MASK	0x02
#define LIS2DH_HP_IA1_MASK	0x01

/**
* @brief  Control register 3
*/
#define LIS2DH_I1_CLICK_BIT		LIS2DH_BIT(7)
#define LIS2DH_I1_IA1_BIT		LIS2DH_BIT(6)
#define LIS2DH_I1_IA2_BIT		LIS2DH_BIT(5)
#define LIS2DH_I1_ZYXDA_BIT		LIS2DH_BIT(4)
#define LIS2DH_I1_WTM_BIT		LIS2DH_BIT(2)
#define LIS2DH_I1_OVERRUAN_BIT	LIS2DH_BIT(1)

#define LIS2DH_I1_CLICK_MASK	0x80
#define LIS2DH_I1_IA1_MASK		0x40
#define LIS2DH_I1_IA2_MASK		0x20
#define LIS2DH_I1_ZYXDA_MASK	0x10
#define LIS2DH_I1_WTM_MASK		0x04
#define LIS2DH_I1_OVERRUAN_MASK	0x02

/**
* @brief  Control register 4
*/
#define LIS2DH_BDU_BIT	LIS2DH_BIT(7)
#define LIS2DH_BLE_BIT	LIS2DH_BIT(6)
#define LIS2DH_FS_BIT	LIS2DH_BIT(4)
#define LIS2DH_HR_BIT	LIS2DH_BIT(3)
#define LIS2DH_ST_BIT	LIS2DH_BIT(1)
#define LIS2DH_SIM_BIT	LIS2DH_BIT(0)

#define LIS2DH_BDU_MASK	0x80
#define LIS2DH_BLE_MASK	0x40
#define LIS2DH_FS_MASK	0x30
#define LIS2DH_HR_MASK	0x08
#define LIS2DH_ST_MASK	0x06
#define LIS2DH_SIM_MASK	0x01

/**
* @brief  Control register 5
*/
#define LIS2DH_BOOT_BIT		LIS2DH_BIT(7)
#define LIS2DH_FIFO_EN_BIT	LIS2DH_BIT(6)
#define LIS2DH_LIR_INT1_BIT	LIS2DH_BIT(3)
#define LIS2DH_D4D_INT1_BIT	LIS2DH_BIT(2)
#define LIS2DH_LIR_INT2_BIT	LIS2DH_BIT(1)
#define LIS2DH_D4D_INT2_BIT	LIS2DH_BIT(0)

#define LIS2DH_BOOT_MASK		0x80
#define LIS2DH_FIFO_EN_MASK		0x40
#define LIS2DH_LIR_INT1_MASK	0x08
#define LIS2DH_D4D_INT1_MASK	0x04
#define LIS2DH_LIR_INT2_MASK	0x02
#define LIS2DH_D4D_INT2_MASK	0x01

/**
* @brief  Control register 6
*/
#define LIS2DH_I2_CLICK_BIT		LIS2DH_BIT(7)
#define LIS2DH_I2_IA1_BIT		LIS2DH_BIT(6)
#define LIS2DH_I2_IA2_BIT		LIS2DH_BIT(5)
#define LIS2DH_I2_BOOT_BIT		LIS2DH_BIT(4)
#define LIS2DH_I2_ACT_BIT		LIS2DH_BIT(3)
#define LIS2DH_I2_POLARITY_BIT	LIS2DH_BIT(1)

#define LIS2DH_I2_CLICK_MASK	0x80
#define LIS2DH_I2_IA1_MASK		0x40
#define LIS2DH_I2_IA2_MASK		0x20
#define LIS2DH_I2_BOOT_MASK		0x10
#define LIS2DH_I2_ACT_MASK		0x08
#define LIS2DH_I2_POLARITY_MASK	0x02

/**
* @brief  Reference value register
*/
#define LIS2DH_REFERENCE_BIT	LIS2DH_BIT(0)

#define LIS2DH_REFERENCE_MASK	0xFF

/**
* @brief  FIFO control register
*/
#define LIS2DH_FM_BIT	LIS2DH_BIT(6)
#define LIS2DH_TR_BIT	LIS2DH_BIT(5)
#define LIS2DH_FTH_BIT	LIS2DH_BIT(0)
#define LIS2DH_FM_MASK	0xC0
#define LIS2DH_TR_MASK	0x20
#define LIS2DH_FTH_MASK	0x1F

/**
* @brief  FIFO source register
*/
#define LIS2DH_WTM_BIT			LIS2DH_BIT(7)
#define LIS2DH_OVRN_FIFO_BIT	LIS2DH_BIT(6)
#define LIS2DH_EMPTY_BIT		LIS2DH_BIT(5)
#define LIS2DH_FSS_BIT			LIS2DH_BIT(0)

#define LIS2DH_WTM_MASK			0x80
#define LIS2DH_OVRN_FIFO_MASK	0x40
#define LIS2DH_EMPTY_MASK		0x20
#define LIS2DH_FSS_MASK			0x1F

/**
* @brief  Interrupt 1 config register
*/
#define LIS2DH_INT1_AOI_BIT		LIS2DH_BIT(7)
#define LIS2DH_INT1_6D_BIT		LIS2DH_BIT(6)
#define LIS2DH_INT1_ZHIE_BIT	LIS2DH_BIT(5)
#define LIS2DH_INT1_ZLIE_BIT	LIS2DH_BIT(4)
#define LIS2DH_INT1_YHIE_BIT	LIS2DH_BIT(3)
#define LIS2DH_INT1_YLIE_BIT	LIS2DH_BIT(2)
#define LIS2DH_INT1_XHIE_BIT	LIS2DH_BIT(1)
#define LIS2DH_INT1_XLIE_BIT	LIS2DH_BIT(0)
#define LIS2DH_INT1_AOI_MASK	0x80
#define LIS2DH_INT1_6D_MASK		0x40
#define LIS2DH_INT1_ZHIE_MASK	0x20
#define LIS2DH_INT1_ZLIE_MASK	0x10
#define LIS2DH_INT1_YHIE_MASK	0x08
#define LIS2DH_INT1_YLIE_MASK	0x04
#define LIS2DH_INT1_XHIE_MASK	0x02
#define LIS2DH_INT1_XLIE_MASK	0x01

/**
* @brief  Interrupt 1 source register
*/
#define LIS2DH_INT1_IA_BIT	LIS2DH_BIT(6)
#define LIS2DH_INT1_ZH_BIT	LIS2DH_BIT(5)
#define LIS2DH_INT1_ZL_BIT	LIS2DH_BIT(4)
#define LIS2DH_INT1_YH_BIT	LIS2DH_BIT(3)
#define LIS2DH_INT1_YL_BIT	LIS2DH_BIT(2)
#define LIS2DH_INT1_XH_BIT	LIS2DH_BIT(1)
#define LIS2DH_INT1_XL_BIT	LIS2DH_BIT(0)
#define LIS2DH_INT1_IA_MASK	0x40
#define LIS2DH_INT1_ZH_MASK	0x20
#define LIS2DH_INT1_ZL_MASK	0x10
#define LIS2DH_INT1_YH_MASK	0x08
#define LIS2DH_INT1_YL_MASK	0x04
#define LIS2DH_INT1_XH_MASK	0x02
#define LIS2DH_INT1_XL_MASK	0x01

/**
* @brief  Interrupt 1 threshold register
*/
#define LIS2DH_INT1_THS_BIT	LIS2DH_BIT(0)
#define LIS2DH_INT1_THS_MASK	0x7F

/**
* @brief  Interrupt 1 duration register
*/
#define LIS2DH_INT1_DURATION_BIT	LIS2DH_BIT(0)
#define LIS2DH_INT1_DURATION_MASK	0x7F

/**
* @brief  Interrupt 2 config register
*/
#define LIS2DH_INT2_AOI_BIT	LIS2DH_BIT(7)
#define LIS2DH_INT2_6D_BIT	LIS2DH_BIT(6)
#define LIS2DH_INT2_ZHIE_BIT	LIS2DH_BIT(5)
#define LIS2DH_INT2_ZLIE_BIT	LIS2DH_BIT(4)
#define LIS2DH_INT2_YHIE_BIT	LIS2DH_BIT(3)
#define LIS2DH_INT2_YLIE_BIT	LIS2DH_BIT(2)
#define LIS2DH_INT2_XHIE_BIT	LIS2DH_BIT(1)
#define LIS2DH_INT2_XLIE_BIT	LIS2DH_BIT(0)

#define LIS2DH_INT2_AOI_MASK	0x80
#define LIS2DH_INT2_6D_MASK	0x40
#define LIS2DH_INT2_ZHIE_MASK	0x20
#define LIS2DH_INT2_ZLIE_MASK	0x10
#define LIS2DH_INT2_YHIE_MASK	0x08
#define LIS2DH_INT2_YLIE_MASK	0x04
#define LIS2DH_INT2_XHIE_MASK	0x02
#define LIS2DH_INT2_XLIE_MASK	0x01

/**
* @brief  Interrupt 2 source register
*/
#define LIS2DH_INT2_IA_BIT	LIS2DH_BIT(6)
#define LIS2DH_INT2_ZH_BIT	LIS2DH_BIT(5)
#define LIS2DH_INT2_ZL_BIT	LIS2DH_BIT(4)
#define LIS2DH_INT2_YH_BIT	LIS2DH_BIT(3)
#define LIS2DH_INT2_YL_BIT	LIS2DH_BIT(2)
#define LIS2DH_INT2_XH_BIT	LIS2DH_BIT(1)
#define LIS2DH_INT2_XL_BIT	LIS2DH_BIT(0)

#define LIS2DH_INT2_IA_MASK	0x40
#define LIS2DH_INT2_ZH_MASK	0x20
#define LIS2DH_INT2_ZL_MASK	0x10
#define LIS2DH_INT2_YH_MASK	0x08
#define LIS2DH_INT2_YL_MASK	0x04
#define LIS2DH_INT2_XH_MASK	0x02
#define LIS2DH_INT2_XL_MASK	0x01

/**
* @brief   Interrupt 2 threshold register
*/
#define LIS2DH_INT2_THS_BIT	LIS2DH_BIT(0)
#define LIS2DH_INT2_THS_MASK	0x7F

/**
* @brief  Interrupt 2 duration register
*/
#define LIS2DH_INT2_DURATION_BIT	LIS2DH_BIT(0)

#define LIS2DH_INT2_DURATION_MASK	0x7F

/**
* @brief  Click config register
*/
#define LIS2DH_ZD_BIT		LIS2DH_BIT(5)
#define LIS2DH_ZS_BIT		LIS2DH_BIT(4)
#define LIS2DH_YD_BIT		LIS2DH_BIT(3)
#define LIS2DH_YS_BIT		LIS2DH_BIT(2)
#define LIS2DH_XD_BIT		LIS2DH_BIT(1)
#define LIS2DH_XS_BIT		LIS2DH_BIT(0)
#define LIS2DH_ZD_MASK	0x20
#define LIS2DH_ZS_MASK	0x10
#define LIS2DH_YD_MASK	0x08
#define LIS2DH_YS_MASK	0x04
#define LIS2DH_XD_MASK	0x02
#define LIS2DH_XS_MASK	0x01

/**
* @brief  Click source register
*/
#define LIS2DH_CLICK_IA_BIT		LIS2DH_BIT(6)
#define LIS2DH_CLICK_DCLICK_BIT	LIS2DH_BIT(5)
#define LIS2DH_CLICK_SCLICK_BIT	LIS2DH_BIT(4)
#define LIS2DH_CLICK_SIGN_BIT		LIS2DH_BIT(3)
#define LIS2DH_CLICK_Z_BIT		LIS2DH_BIT(2)
#define LIS2DH_CLICK_Y_BIT		LIS2DH_BIT(1)
#define LIS2DH_CLICK_X_BIT		LIS2DH_BIT(0)

#define LIS2DH_CLICK_IA_MASK		0x40
#define LIS2DH_CLICK_DCLICK_MASK	0x20
#define LIS2DH_CLICK_SCLICK_MASK	0x10
#define LIS2DH_CLICK_SIGN_MASK	0x08
#define LIS2DH_CLICK_Z_MASK		0x04
#define LIS2DH_CLICK_Y_MASK		0x02
#define LIS2DH_CLICK_X_MASK		0x01

/**
* @brief  Click threshold register
*/
#define LIS2DH_CLICK_LIR_BIT	LIS2DH_BIT(7)
#define LIS2DH_CLICK_THS_BIT	LIS2DH_BIT(0)
#define LIS2DH_CLICK_LIR_MASK	0x80
#define LIS2DH_CLICK_THS_MASK	0x7F

/**
* @brief  Click time limit register
*/
#define LIS2DH_TLI_BIT	LIS2DH_BIT(0)
#define LIS2DH_TLI_MASK	0x7F

/**
* @brief  Click time latency register
*/
#define LIS2DH_TLA_BIT	LIS2DH_BIT(0)
#define LIS2DH_TLA_MASK	0xFF

/**
* @brief  Click time window register
*/
#define LIS2DH_TW_BIT	LIS2DH_BIT(0)
#define LIS2DH_TW_MASK	0xFF

/**
* @brief  ACT threshold register
*/
#define LIS2DH_ACTH_BIT		LIS2DH_BIT(0)
#define LIS2DH_ACTH_MASK	0x7F

/**
* @brief  ACT duration register
*/
#define LIS2DH_TW_BIT	LIS2DH_BIT(0)
#define LIS2DH_TW_MASK	0xFF

#define LIS2DH_FROM_FS_2g_HR_TO_mg(lsb)		((float)((int16_t)lsb>>4) * 1.0f)
#define LIS2DH_FROM_FS_4g_HR_TO_mg(lsb)		((float)((int16_t)lsb>>4) * 2.0f)
#define LIS2DH_FROM_FS_8g_HR_TO_mg(lsb)		((float)((int16_t)lsb>>4) * 4.0f)
#define LIS2DH_FROM_FS_16g_HR_TO_mg(lsb)	((float)((int16_t)lsb>>4) * 12.0f)
#define LIS2DH_FROM_LSB_TO_degC_HR(lsb)		((float)((int16_t)lsb>>6) / 4.0f+25.0f)

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char * TAG = "lis2dh";

/* Private function prototypes -----------------------------------------------*/
esp_err_t lis2dh_read_reg(lis2dh_t * const me, uint8_t reg_addr, uint8_t * data, size_t data_len);
esp_err_t lis2dh_write_reg(lis2dh_t * const me, uint8_t reg_addr, uint8_t * data, size_t data_len);

/* Exported functions --------------------------------------------------------*/
esp_err_t lis2dh_init(lis2dh_t * const me, uint8_t dev_addr, i2c_port_t i2c_num) {
	ESP_LOGI(TAG, "Initializing %s component...", TAG);

	esp_err_t ret;

	/* Copy configuration */
	if(dev_addr >= 0xFF) {
		ESP_LOGE(TAG, "Device address invalid");

		return ESP_FAIL;
	}

	me->dev_addr = dev_addr;

	if(i2c_num > I2C_NUM_MAX - 1) {
		ESP_LOGE(TAG, "I2C port number invalid");

		return ESP_FAIL;
	}

	me->i2c_num = i2c_num;

	/* Get and print default configuration */
	lis2dh_config_t default_config = {0};
	ret = lis2dh_get_config(me, &default_config);

	if(ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed getting device configuration");

		return ESP_FAIL;
	}

	/* Print default configuration */
//	ESP_LOGI(TAG, "temp_enable is: %02x", default_config.temp_enable);
//    ESP_LOGI(TAG, "odr is: %02x", default_config.odr);
//    ESP_LOGI(TAG, "option mode is: %02x", default_config.opt_mode);
//    ESP_LOGI(TAG, "z_enable status is: %02x", default_config.z_enable);
//    ESP_LOGI(TAG, "y_enable status is: %02x", default_config.y_enable);
//    ESP_LOGI(TAG, "x_enable status is: %02x", default_config.x_enable);
//    ESP_LOGI(TAG, "bdu_status status is: %02x", default_config.bdu_status);
//    ESP_LOGI(TAG, "full scale is: %02x", default_config.fs);

    /* Get and print device ID */
	uint8_t device_id;

	ret = lis2dh_get_device_id(me, &device_id);

	if(ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed getting device ID");

		return ESP_FAIL;
	}

    ESP_LOGI(TAG, "LIS2DH device id is: %02x\n", device_id);

	return ret;
}

esp_err_t lis2dh_get_device_id(lis2dh_t * const me, uint8_t * device_id) {
	esp_err_t ret;

	ret = lis2dh_read_reg(me, LIS2DH_WHO_AM_I_REG, device_id, sizeof(* device_id));

	return ret;
}

esp_err_t lis2dh_set_config(lis2dh_t * const me, lis2dh_config_t * lis2dh_config) {
	esp_err_t ret;

	uint8_t data_tmp [6];

//	ERR_ASSERT(TAG, i2c_bus_read_byte(sens->i2c_dev, LIS2DH_TEMP_CFG_REG, data_tmp));
	ret = lis2dh_read_reg(me, LIS2DH_TEMP_CFG_REG, &data_tmp[0], 1);
    data_tmp[0] &= ~LIS2DH_TEMP_EN_MASK;
    data_tmp[0] |= ((uint8_t)lis2dh_config->temp_enable) << LIS2DH_TEMP_EN_BIT;
//    ERR_ASSERT(TAG, i2c_bus_write_byte(sens->i2c_dev, LIS2DH_TEMP_CFG_REG, data_tmp[0]));
    ret = lis2dh_write_reg(me, LIS2DH_TEMP_CFG_REG, &data_tmp[0], 1);
//    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH_CTRL_REG1 | LIS2DH_I2C_MULTI_REG_ONCE, 6, data_tmp));
    lis2dh_read_reg(me, LIS2DH_CTRL_REG1 | LIS2DH_I2C_MULTI_REG_ONCE, data_tmp, 6);

    data_tmp[0] &= (uint8_t) ~(LIS2DH_ODR_MASK | LIS2DH_LP_EN_MASK | LIS2DH_Z_EN_MASK | LIS2DH_Y_EN_MASK | LIS2DH_X_EN_MASK);
    data_tmp[0] |= ((uint8_t)lis2dh_config->odr) << LIS2DH_ODR_BIT;
    data_tmp[0] |= ((uint8_t)((lis2dh_config->opt_mode >> 1) << LIS2DH_LP_EN_BIT)&LIS2DH_LP_EN_MASK);
    data_tmp[0] |= ((uint8_t)lis2dh_config->z_enable) << LIS2DH_Z_EN_BIT;
    data_tmp[0] |= ((uint8_t)lis2dh_config->y_enable) << LIS2DH_Y_EN_BIT;
    data_tmp[0] |= ((uint8_t)lis2dh_config->x_enable) << LIS2DH_X_EN_BIT;

    data_tmp[3] &= ~(LIS2DH_BDU_MASK | LIS2DH_FS_MASK | LIS2DH_HR_MASK);
    data_tmp[3] |= ((uint8_t)lis2dh_config->bdu_status) << LIS2DH_BDU_BIT;
    data_tmp[3] |= ((uint8_t)lis2dh_config->fs) << LIS2DH_FS_BIT;
    data_tmp[3] |= ((uint8_t)((lis2dh_config->opt_mode) << LIS2DH_HR_BIT)&LIS2DH_HR_MASK);
//    ERR_ASSERT(TAG, i2c_bus_write_bytes(sens->i2c_dev, LIS2DH_CTRL_REG1 | LIS2DH_I2C_MULTI_REG_ONCE, 6, data_tmp));
    ret = lis2dh_write_reg(me, LIS2DH_CTRL_REG1 | LIS2DH_I2C_MULTI_REG_ONCE, data_tmp, 6);

	return ret;
}

esp_err_t lis2dh_get_config(lis2dh_t * const me, lis2dh_config_t * lis2dh_config) {
	esp_err_t ret;

	uint8_t data_tmp [6];

	ret = lis2dh_read_reg(me, LIS2DH_TEMP_CFG_REG, &data_tmp[0], sizeof(data_tmp[0]));
    lis2dh_config->temp_enable = (lis2dh_temp_en_t)((data_tmp[0] & LIS2DH_TEMP_EN_MASK) >> LIS2DH_TEMP_EN_BIT);

	ret = lis2dh_read_reg(me, LIS2DH_CTRL_REG1 | LIS2DH_I2C_MULTI_REG_ONCE, data_tmp, sizeof(data_tmp));

	lis2dh_config->odr = (lis2dh_odr_t)((data_tmp[0] & LIS2DH_ODR_MASK) >> LIS2DH_ODR_BIT);
    lis2dh_config->z_enable = (lis2dh_state_t)((data_tmp[0] & LIS2DH_Z_EN_MASK) >> LIS2DH_Z_EN_BIT);
    lis2dh_config->y_enable = (lis2dh_state_t)((data_tmp[0] & LIS2DH_Y_EN_MASK) >> LIS2DH_Y_EN_BIT);
    lis2dh_config->x_enable = (lis2dh_state_t)((data_tmp[0] & LIS2DH_X_EN_MASK) >> LIS2DH_X_EN_BIT);
    lis2dh_config->bdu_status = (lis2dh_state_t)((data_tmp[3] & LIS2DH_BDU_MASK) >> LIS2DH_BDU_BIT);
    lis2dh_config->fs = (lis2dh_fs_t)((data_tmp[3] & LIS2DH_FS_MASK) >> LIS2DH_FS_BIT);
    lis2dh_config->opt_mode = (lis2dh_opt_mode_t)((((data_tmp[0] & LIS2DH_LP_EN_MASK) << 1) >> LIS2DH_LP_EN_BIT) | ((data_tmp[3] & LIS2DH_HR_MASK) >> LIS2DH_HR_BIT));

	return ret;
}

//esp_err_t lis2dh_set_temp_enable(lis2dh_t * const me, lis2dh_temp_en_t temp_en) {
//	esp_err_t ret;
//
//	return ret;
//}
//
//esp_err_t lis2dh_set_odr(lis2dh_t * const me, lis2dh_odr_t odr) {
//	esp_err_t ret;
//
//	return ret;
//}
//
//esp_err_t lis2dh_set_z_enable(lis2dh_t * const me, lis2dh_state_t status) {
//	esp_err_t ret;
//
//	return ret;
//}
//
//esp_err_t lis2dh_set_y_enable(lis2dh_t * const me, lis2dh_state_t status) {
//	esp_err_t ret;
//
//	return ret;
//}
//
//esp_err_t lis2dh_set_x_enable(lis2dh_t * const me, lis2dh_state_t status) {
//	esp_err_t ret;
//
//	return ret;
//}
//
//esp_err_t lis2dh_set_bdumode(lis2dh_t * const me, lis2dh_state_t status) {
//	esp_err_t ret;
//
//	return ret;
//}
//
//esp_err_t lis2dh_set_fs(lis2dh_t * const me, lis2dh_fs_t fs) {
//	esp_err_t ret;
//
//	return ret;
//}
//
esp_err_t lis2dh_get_fs(lis2dh_t * const me, lis2dh_fs_t * fs) {
	esp_err_t ret;

    uint8_t data_tmp;
    ret = lis2dh_read_reg(me, LIS2DH_CTRL_REG4, &data_tmp, 1);
    * fs = (lis2dh_fs_t)((data_tmp & LIS2DH_FS_MASK) >> LIS2DH_FS_BIT);

	return ret;
}
//
//esp_err_t lis2dh_set_opt_mode(lis2dh_t * const me, lis2dh_opt_mode_t opt_mode) {
//	esp_err_t ret;
//
//	return ret;
//}

esp_err_t lis2dh_get_x_acc(lis2dh_t * const me, uint16_t * x_acc) {
	esp_err_t ret;
    uint8_t data_tmp[2];

    ret = lis2dh_read_reg(me, LIS2DH_OUT_X_L_REG | LIS2DH_I2C_MULTI_REG_ONCE, data_tmp, 2);
    * x_acc = (int16_t)((((uint16_t)data_tmp[1]) << 8) | (uint16_t)data_tmp[0]);

	return ret;
}

esp_err_t lis2dh_get_y_acc(lis2dh_t * const me, uint16_t * y_acc) {
	esp_err_t ret;
    uint8_t data_tmp[2];

    ret = lis2dh_read_reg(me, LIS2DH_OUT_Y_L_REG | LIS2DH_I2C_MULTI_REG_ONCE, data_tmp, 2);
    * y_acc = (int16_t)((((uint16_t)data_tmp[1]) << 8) | (uint16_t)data_tmp[0]);

	return ret;
}

esp_err_t lis2dh_get_z_acc(lis2dh_t * const me, uint16_t * z_acc) {
	esp_err_t ret;
    uint8_t data_tmp[2];

    ret = lis2dh_read_reg(me, LIS2DH_OUT_Z_L_REG | LIS2DH_I2C_MULTI_REG_ONCE, data_tmp, 2);
    * z_acc = (int16_t)((((uint16_t)data_tmp[1]) << 8) | (uint16_t)data_tmp[0]);

	return ret;
}

esp_err_t lis2dh_get_raw_acce(lis2dh_t * const me, lis2dh_raw_acce_value_t * raw_acce_value) {
	esp_err_t ret;

	uint8_t data_tmp[6];
//    ERR_ASSERT(TAG, i2c_bus_read_bytes(sens->i2c_dev, LIS2DH12_OUT_X_L_REG | LIS2DH12_I2C_MULTI_REG_ONCE, 6, buffer));
    ret = lis2dh_read_reg(me, LIS2DH_OUT_X_L_REG , data_tmp, 6);
    raw_acce_value->raw_acce_x = *(int16_t *)data_tmp;
    raw_acce_value->raw_acce_y = *(int16_t *)(data_tmp + 2);
    raw_acce_value->raw_acce_z = *(int16_t *)(data_tmp + 4);

	return ret;
}

esp_err_t lis2dh_get_acce(lis2dh_t * const me, lis2dh_acce_value_t * acce_value) {
	esp_err_t ret;

    lis2dh_fs_t fs;
    lis2dh_raw_acce_value_t raw_acce_value;
    ret = lis2dh_get_fs(me, &fs);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = lis2dh_get_raw_acce(me, &raw_acce_value);

    if (fs == LIS2DH_FS_2G) {
        acce_value->acce_x = LIS2DH_FROM_FS_2g_HR_TO_mg(raw_acce_value.raw_acce_x) / 1000.0;
        acce_value->acce_y = LIS2DH_FROM_FS_2g_HR_TO_mg(raw_acce_value.raw_acce_y) / 1000.0;
        acce_value->acce_z = LIS2DH_FROM_FS_2g_HR_TO_mg(raw_acce_value.raw_acce_z) / 1000.0;
    } else if (fs == LIS2DH_FS_4G) {
        acce_value->acce_x = LIS2DH_FROM_FS_4g_HR_TO_mg(raw_acce_value.raw_acce_x) / 1000.0;
        acce_value->acce_y = LIS2DH_FROM_FS_4g_HR_TO_mg(raw_acce_value.raw_acce_y) / 1000.0;
        acce_value->acce_z = LIS2DH_FROM_FS_4g_HR_TO_mg(raw_acce_value.raw_acce_z) / 1000.0;
    } else if (fs == LIS2DH_FS_8G) {
        acce_value->acce_x = LIS2DH_FROM_FS_8g_HR_TO_mg(raw_acce_value.raw_acce_x) / 1000.0;
        acce_value->acce_y = LIS2DH_FROM_FS_8g_HR_TO_mg(raw_acce_value.raw_acce_y) / 1000.0;
        acce_value->acce_z = LIS2DH_FROM_FS_8g_HR_TO_mg(raw_acce_value.raw_acce_z) / 1000.0;
    } else if (fs == LIS2DH_FS_16G) {
        acce_value->acce_x = LIS2DH_FROM_FS_16g_HR_TO_mg(raw_acce_value.raw_acce_x) / 1000.0;
        acce_value->acce_y = LIS2DH_FROM_FS_16g_HR_TO_mg(raw_acce_value.raw_acce_y) / 1000.0;
        acce_value->acce_z = LIS2DH_FROM_FS_16g_HR_TO_mg(raw_acce_value.raw_acce_z) / 1000.0;
    }

	return ret;
}

/* Private functions ---------------------------------------------------------*/
esp_err_t lis2dh_read_reg(lis2dh_t * const me, uint8_t reg_addr, uint8_t * data, size_t data_len) {
    return i2c_master_write_read_device(me->i2c_num, me->dev_addr, &reg_addr, 1, data, data_len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t lis2dh_write_reg(lis2dh_t * const me, uint8_t reg_addr, uint8_t * data, size_t data_len) {
	return i2c_master_write_to_device(me->i2c_num, me->dev_addr, data, sizeof(data), I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/***************************** END OF FILE ************************************/
