/**\mainpage
 * Copyright (C) 2017 - 2019 Richtek
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * File		rt3020.c
 * Date		19 Jun 2016
 * Version	0.0.1
 *
 */
/*! @file rt3020.c
 * @brief Sensor driver for RT3020 sensor
 */

#include "rt3020.h"

/************************** Internal macros *******************************/
/********************** Static function declarations ************************/
/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of rt3020_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t null_ptr_check(const struct rt3020_dev *dev);

/*!
 * @brief This internal API is used to set the accel configurations in sensor
 *
 * @param[in] accel_conf : Structure instance with accel configurations
 * @param[in] dev             : Structure instance of rt3020_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_accel_conf(const struct rt3020_acc_conf *accel_conf, const struct rt3020_dev *dev);

/*!
 * @brief This API reads accel data along with sensor time
 *
 * @param[in,out] accel  : Structure instance to store the accel data
 * @param[in] dev        : Structure instance of rt3020_dev
 *
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_accel_data(struct rt3020_sensor_data *accel, const struct rt3020_dev *dev);

/*!
 * @brief This internal API is used to get the accel configurations in sensor
 *
 * @param[in,out] accel_conf  : Structure instance of basic
 *                              accelerometer configuration
 * @param[in] dev             : Structure instance of rt3020_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_accel_conf(struct rt3020_acc_conf *accel_conf, const struct rt3020_dev *dev);

/********************** Global function definitions ************************/
/*!
 *  @brief This API is the entry point, Call this API before using other APIs.
 *  This API reads the chip-id of the sensor which is the first step to
 *  verify the sensor and updates the trim parameters of the sensor.
 */
int8_t rt3020_init(struct rt3020_dev *dev)
{
	int8_t rslt;
	uint8_t chip_id = 0;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == RT3020_OK) {
		/* Initial power-up time */
		dev->delay_ms(5);
		/* Assigning dummy byte value */
		if (dev->intf == RT3020_SPI_INTF) {
			/* Dummy Byte availability */
			dev->dummy_byte = 1;
			/* Dummy read of Chip-ID in SPI mode */
			rslt = rt3020_get_regs(RT3020_WHOAMI_ADDR, &chip_id, 1, dev);
		} else {
			dev->dummy_byte = 0;
		} 
		if (rslt == RT3020_OK) {
			/* Chip ID of the sensor is read */
			rslt = rt3020_get_regs(RT3020_WHOAMI_ADDR, &chip_id, 1, dev);
			/* Proceed if everything is fine until now */
			if (rslt == RT3020_OK) {
				/* Check for chip id validity */
				if (chip_id == RT3020_CHIP_ID) {
					/* Store the chip ID in dev structure */
					dev->chip_id = chip_id;
				} else {
					rslt = RT3020_E_DEV_NOT_FOUND;
				}
			}
		}
	}
	
	return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t rt3020_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, const struct rt3020_dev *dev)
{
	int8_t rslt;
	uint8_t count;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if ((rslt == RT3020_OK) && (reg_data != NULL)) {
		/* Write the data to the reg_addr */
		/* SPI write requires to set The MSB of reg_addr as 0
		   but in default the MSB is always 0 */
		if (len == 1) {
			rslt = dev->write(dev->intf_ptr, dev->dev_id, reg_addr, reg_data, len);
			if (rslt != RT3020_OK) {
				/* Failure case */
				rslt = RT3020_E_COM_FAIL;
			}
		}
		/* Burst write is not allowed thus we split burst case write
		 * into single byte writes Thus user can write multiple bytes
		 * with ease */
		if (len > 1) {
			for (count = 0; count < len; count++) {
				rslt = dev->write(dev->intf_ptr, dev->dev_id, reg_addr, &reg_data[count], 1);
				reg_addr++;
			}
		}
	} else {
		rslt = RT3020_E_NULL_PTR;
	}
	
	return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t rt3020_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, const struct rt3020_dev *dev)
{
	int8_t rslt;
	uint16_t index;
	uint16_t temp_len = len;
	uint8_t temp_buff[temp_len];

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if ((rslt == RT3020_OK) && (reg_data != NULL)) {
		if (dev->intf != RT3020_I2C_INTF) {
			/* If interface selected is SPI */
			reg_addr = reg_addr | RT3020_SPI_RD_MASK;
		}
		/* Read the data from the reg_addr */
		rslt = dev->read(dev->intf_ptr, dev->dev_id, reg_addr, temp_buff, temp_len);
		if (rslt == RT3020_OK) {
			for (index = 0; index < len; index++) {
				/* Parse the data read and store in "reg_data"
				 * buffer so that the dummy byte is removed
				 * and user will get only valid data
				 */
				reg_data[index] = temp_buff[index];
			}
		}
		if (rslt != RT3020_OK) {
			/* Failure case */
			rslt = RT3020_E_COM_FAIL;
		}
	} else {
		rslt = RT3020_E_NULL_PTR;
	}
	
	return rslt;
}

/*!
 * @brief This API is used to perform soft-reset of the sensor
 * where all the registers are reset to their default values.
 */
int8_t rt3020_soft_reset(const struct rt3020_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	
	/* Reset the device */
	data = RT3020_SOFT_RESET_CMD;
	if (rslt == RT3020_OK) {
		rslt = rt3020_set_regs(RT3020_MODE_CFG_ADDR, &data, 1, dev);
	}
	if (rslt == RT3020_OK) {
		rslt = rt3020_set_regs(RT3020_MODE_CFG_ADDR, &data, 1, dev);
	}
	data = RT3020_SOFT_RESET_DONE_CMD;
	if (rslt == RT3020_OK) {
		rslt = rt3020_set_regs(RT3020_MODE_CFG_ADDR, &data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This API is used to set the power mode of the sensor.
 */
int8_t rt3020_set_power_mode(uint8_t power_mode, const struct rt3020_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data = 0;

	rslt = null_ptr_check(dev);

	/* Set the power mode of sensor */
	if (rslt == RT3020_OK) {
		rslt = rt3020_get_regs(RT3020_MODE_CFG_ADDR, &reg_data, 1, dev);
	}
	dev->delay_ms(500); //for debugging
	if (rslt == RT3020_OK) {
		reg_data = RT3020_SET_BITS(reg_data, RT3020_POWER_MODE, power_mode);
		rslt = rt3020_set_regs(RT3020_MODE_CFG_ADDR, &reg_data, 1, dev);
		if (power_mode == RT3020_NORMAL_MODE) {
			dev->delay_ms(60);
		}
		else {
			dev->delay_ms(10);
		}
	}

	return rslt;
}

/*!
 * @brief This API is used to get the power mode of the sensor.
 */
int8_t rt3020_get_power_mode(uint8_t *power_mode, const struct rt3020_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == RT3020_OK) {
		rslt = rt3020_get_regs(RT3020_MODE_STS_ADDR, &reg_data, 1, dev);
		*power_mode = RT3020_GET_BITS(reg_data, RT3020_POWER_MODE_STATUS);
	}
	
	return rslt;
}

/*!
 * @brief This API is used to get the accel data along with the sensor-time
 */
int8_t rt3020_get_accel_data(struct rt3020_sensor_data *accel, const struct rt3020_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if ((rslt == RT3020_OK) && (accel != NULL)) {
		/* Read and store the accel data */
		rslt = get_accel_data(accel, dev);
	} else {
		rslt = RT3020_E_NULL_PTR;
	}
	
	return rslt;
}

/*!
 * @brief This API is used to set the sensor settings like sensor
 * configurations and interrupt configurations
 */
int8_t rt3020_set_sensor_conf(const struct rt3020_sensor_conf *conf, uint16_t n_sett, const struct rt3020_dev *dev)
{
	int8_t rslt;
	uint16_t idx = 0;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	if (rslt == RT3020_OK) {
		for (idx = 0; idx < n_sett; idx++) {
			switch (conf[idx].type) {
			case RT3020_ACCEL:
				/* Setting Accel configurations */
				rslt = set_accel_conf(&conf[idx].param.accel, dev);
				if (rslt == RT3020_OK) {
					/* Int pin mapping settings */
				}
				break;
			default:
				rslt = RT3020_E_INVALID_CONFIG;
			}
		}
	}
	
	return rslt;
}

/*!
 * @brief This API is used to get the sensor settings like sensor
 * configurations and interrupt configurations and store
 * them in the corresponding structure instance
 */
int8_t rt3020_get_sensor_conf(struct rt3020_sensor_conf *conf, uint16_t n_sett, const struct rt3020_dev *dev)
{
	int8_t rslt = RT3020_OK;
	uint16_t idx = 0;

	if (conf == NULL) {
		rslt = RT3020_E_NULL_PTR;
	}

	for (idx = 0; (idx < n_sett) && (rslt == RT3020_OK); idx++) {
		switch (conf[idx].type) {
		case RT3020_ACCEL:
			/* Accel configuration settings */
			rslt = get_accel_conf(&conf[idx].param.accel, dev);
			if (rslt == RT3020_OK) {
				/* Get the INT pin mapping */
			}
			break;
		default:
			rslt = RT3020_E_INVALID_CONFIG;
		}
	}

	return rslt;
}

/****************************************************************************/
/**\name	INTERNAL APIs                                               */
/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct rt3020_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
		/* Device structure pointer is not valid */
		rslt = RT3020_E_NULL_PTR;
	}
	else {
		/* Device structure is fine */
		rslt = RT3020_OK;
	}
	
	return rslt;
}

/*!
 * @brief This internal API is used to set the accel configurations in sensor
 */
static int8_t set_accel_conf(const struct rt3020_acc_conf *accel_conf, const struct rt3020_dev *dev)
{
	int8_t rslt;
	uint8_t data_array[2];

	/* Update the accel configurations from the user structure
	 * accel_conf */
	dev->delay_ms(800); //for debugging
	rslt = rt3020_get_regs(RT3020_CTRL_CFG1_ADDR, data_array, 2, dev);
	if (rslt == RT3020_OK) {
		data_array[0] = RT3020_SET_BITS(data_array[0], RT3020_ACCEL_ODR, accel_conf->odr);
		data_array[0] = RT3020_SET_BITS(data_array[0], RT3020_DATA_FILTER, accel_conf->data_src);
		data_array[1] = RT3020_SET_BITS(data_array[1], RT3020_ACCEL_RANGE, accel_conf->range);
		/* Set the accel configurations in the sensor */
		dev->delay_ms(800); //for debugging
		rslt = rt3020_set_regs(RT3020_CTRL_CFG1_ADDR, data_array, 2, dev);
		dev->delay_ms(800); //for debugging
		dev->delay_ms(160);
	}
	
	return rslt;
}

/*!
 * @brief This internal API is used to set the accel configurations in sensor
 */
static int8_t get_accel_conf(struct rt3020_acc_conf *accel_conf, const struct rt3020_dev *dev)
{
	int8_t rslt;
	uint8_t data_array[2];

	rslt = rt3020_get_regs(RT3020_CTRL_CFG1_ADDR, data_array, 2, dev);
	if (rslt == RT3020_OK) {
		accel_conf->odr = RT3020_GET_BITS(data_array[0], RT3020_ACCEL_ODR);
		accel_conf->data_src = RT3020_GET_BITS(data_array[0], RT3020_DATA_FILTER);
		accel_conf->range = RT3020_GET_BITS(data_array[1], RT3020_ACCEL_RANGE);
	}

	return rslt;
}

/*!
 * @brief This API reads accel data along with sensor time
 */
static int8_t get_accel_data(struct rt3020_sensor_data *accel, const struct rt3020_dev *dev)
{
	int8_t rslt;
	int16_t data_array[3] = { 0 };

	/* Read the sensor data registers only */
	rslt = rt3020_get_regs(RT3020_XDATA_L_ADDR, (uint8_t *)data_array, 6, dev);
	if (rslt == RT3020_OK) {
		accel->x = data_array[0];
		accel->y = data_array[1];
		accel->z = data_array[2];
	}
	
	return rslt;
}

