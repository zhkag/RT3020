/**
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
 * @file	rt3020_defs.h
 * @date	19 Jun 2019
 * @version	0.0.1
 * @brief
 *
 */
/*! \file rt3020_defs.h */
/*!
 * @defgroup RT3020 SENSOR API
 * @brief
 * @{
 */
#ifndef RT3020_DEFS_H_
#define RT3020_DEFS_H_
/*********************************************************************/
/**\ header files */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif
/*********************************************************************/
/** \name		Common macros				     */
/*********************************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)       S8_C(x)
#define UINT8_C(x)      U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)      S16_C(x)
#define UINT16_C(x)     U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)      S32_C(x)
#define UINT32_C(x)     U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)      S64_C(x)
#define UINT64_C(x)     U64_C(x)
#endif
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL   0
#else
#define NULL   ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE     UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE    UINT8_C(0)
#endif
/********************************************************/
/**\name Macro definitions */
/**\name API success code */
#define RT3020_OK       INT8_C(0)
/**\name API error codes */
#define RT3020_E_NULL_PTR          INT8_C(-1)
#define RT3020_E_COM_FAIL          INT8_C(-2)
#define RT3020_E_DEV_NOT_FOUND     INT8_C(-3)
#define RT3020_E_INVALID_CONFIG    INT8_C(-4)
/**\name API warning codes */
#define RT3020_W_SELF_TEST_FAIL    INT8_C(1)
/**\name CHIP ID VALUE */
#define RT3020_CHIP_ID     UINT8_C(0x33)
/********************************************************/
/**\name	USER CONFIGURATION MACROS               */
/**\name RT3020 I2C address macros */
#define RT3020_I2C_ADDRESS_SDO_LOW     UINT8_C(0x18)
#define RT3020_I2C_ADDRESS_SDO_HIGH    UINT8_C(0x19)
/**\name Power mode configurations */
#define RT3020_POWER_DOWN_MODE		UINT8_C(0x00)
#define RT3020_WAKEUP_MODE				UINT8_C(0x01)
#define RT3020_NORMAL_MODE    		UINT8_C(0x02)
/**\name Enable / Disable macros */
#define RT3020_DISABLE        UINT8_C(0)
#define RT3020_ENABLE         UINT8_C(1)
/**\name ODR configurations  */
#define RT3020_ODR_6_25HZ     UINT8_C(0x00)
#define RT3020_ODR_12_5HZ     UINT8_C(0x01)
#define RT3020_ODR_25HZ       UINT8_C(0x02)
#define RT3020_ODR_50HZ       UINT8_C(0x03)
#define RT3020_ODR_100HZ      UINT8_C(0x04)
#define RT3020_ODR_200HZ      UINT8_C(0x05)
#define RT3020_ODR_400HZ      UINT8_C(0x06)
/**\name Accel Range configuration */
#define RT3020_2G_RANGE      UINT8_C(0x00)
#define RT3020_4G_RANGE      UINT8_C(0x01)
#define RT3020_8G_RANGE      UINT8_C(0x02)
#define RT3020_16G_RANGE     UINT8_C(0x03)
/**\name Accel Axes selection settings for
 * DATA SAMPLING
 */
#define RT3020_X_AXIS_EN       UINT8_C(0x01)
#define RT3020_Y_AXIS_EN       UINT8_C(0x02)
#define RT3020_Z_AXIS_EN       UINT8_C(0x04)
#define RT3020_XYZ_AXIS_EN     UINT8_C(0x07)
/**\name Accel filter(data_src_reg) selection settings */
#define RT3020_DATA_SRC_NORMAL      UINT8_C(0x00)
#define RT3020_DATA_SRC_LPF     		UINT8_C(0x01)

/**\name Interrupt pin configuration macros */
#define RT3020_INT_HIGH_ACTIVE     UINT8_C(0x00)
#define RT3020_INT_LOW_ACTIVE      UINT8_C(0x01)

/**********************************************************************/
/**\name RT3020 Register Address */
#define RT3020_WHOAMI_ADDR              UINT8_C(0x00)
#define RT3020_IRQ_STS_ADDR             UINT8_C(0x01)
#define RT3020_MODE_STS_ADDR            UINT8_C(0x02)
#define RT3020_CTRL_CFG0_ADDR           UINT8_C(0x07)
#define RT3020_MODE_CFG_ADDR            UINT8_C(0x08)
#define RT3020_CTRL_CFG1_ADDR           UINT8_C(0x09)
#define RT3020_CTRL_CFG2_ADDR           UINT8_C(0x0A)
#define RT3020_CTRL_CFG3_ADDR           UINT8_C(0x0B)
#define RT3020_XDATA_L_ADDR             UINT8_C(0x10)
#define RT3020_XDATA_H_ADDR             UINT8_C(0x11)
#define RT3020_YDATA_L_ADDR             UINT8_C(0x12)
#define RT3020_YDATA_H_ADDR             UINT8_C(0x13)
#define RT3020_ZDATA_L_ADDR             UINT8_C(0x14)
#define RT3020_ZDATA_H_ADDR             UINT8_C(0x15)
#define RT3020_FIFO_UNRD_ADDR           UINT8_C(0x16)
#define RT3020_FIFO_STS_ADDR            UINT8_C(0x17)
#define RT3020_AOI_STS1_ADDR            UINT8_C(0x18)
#define RT3020_AOI_STS2_ADDR            UINT8_C(0x19)
#define RT3020_TAP_STS_ADDR             UINT8_C(0x1A)
#define RT3020_IRQ1_MAP_ADDR            UINT8_C(0x20)
#define RT3020_IRQ2_MAP_ADDR            UINT8_C(0x21)
#define RT3020_FIFO_CFG_ADDR            UINT8_C(0x22)
#define RT3020_FIFO_THS_ADDR            UINT8_C(0x23)
#define RT3020_FILTER_ADDR              UINT8_C(0x24)
#define RT3020_AOI_CFG_ADDR             UINT8_C(0x30)
#define RT3020_AOI_TIME_ADDR            UINT8_C(0x31)
#define RT3020_AOI_THSX_ADDR            UINT8_C(0x32)
#define RT3020_AOI_THSY_ADDR            UINT8_C(0x33)
#define RT3020_AOI_THSZ_ADDR            UINT8_C(0x34)
#define RT3020_AOI_HYSX_ADDR            UINT8_C(0x35)
#define RT3020_AOI_HYSZY_ADDR           UINT8_C(0x36)
#define RT3020_TAP_CFG_ADDR             UINT8_C(0x37)
#define RT3020_TAP_THS_ADDR             UINT8_C(0x38)
#define RT3020_TAP_LIMIT_ADDR           UINT8_C(0x39)
#define RT3020_TAP_LAT_ADDR             UINT8_C(0x3A)
#define RT3020_TAP_WIN_ADDR             UINT8_C(0x3B)

/**\name RT3020 Command register */
#define RT3020_SOFT_RESET_CMD     			 UINT8_C(0x10)
#define RT3020_SOFT_RESET_DONE_CMD 			 UINT8_C(0x00)
/**\name RT3020 Delay definitions */
#define RT3020_SELF_TEST_DATA_READ_MS    UINT8_C(50)
/**\name Interface selection macro */
#define RT3020_SPI_WR_MASK    UINT8_C(0x7F)
#define RT3020_SPI_RD_MASK    UINT8_C(0x80)
/**\name UTILITY MACROS	*/
#define RT3020_SET_LOW_BYTE     UINT16_C(0x00FF)
#define RT3020_SET_HIGH_BYTE    UINT16_C(0xFF00)

/**\name Interrupt mapping selection */
#define RT3020_DATA_READY_INT_MAP    	   UINT8_C(0x01)
#define RT3020_FIFO_WM_INT_MAP        	 UINT8_C(0x02)
#define RT3020_FIFO_OVERRUN_INT_MAP      UINT8_C(0x03)
#define RT3020_AOI_INT_MAP          	 	 UINT8_C(0x04)
#define RT3020_TAP_INT_MAP          	 	 UINT8_C(0x05)
#define RT3020_SIGM_INT_MAP      				 UINT8_C(0x06)
#define RT3020_INACTIVE_INT_MAP          UINT8_C(0x07)
#define RT3020_ACTIVE_INT_MAP         	 UINT8_C(0x08)

/**\name RT3020 FIFO configurations */
#define RT3020_FIFO_BYPASS_MODE			UINT8_C(0x00)
#define RT3020_FIFO_FIFO_MODE				UINT8_C(0x01)
#define RT3020_FIFO_STREAM_MODE			UINT8_C(0x02)
#define RT3020_FIFO_TRIGGER_MODE	  UINT8_C(0x03)

/**\name RT3020 FIFO data configurations */
#define RT3020_FIFO_EN_X         UINT8_C(0x01)
#define RT3020_FIFO_EN_Y         UINT8_C(0x02)
#define RT3020_FIFO_EN_Z         UINT8_C(0x04)
#define RT3020_FIFO_EN_XY        UINT8_C(0x03)
#define RT3020_FIFO_EN_YZ        UINT8_C(0x06)
#define RT3020_FIFO_EN_XZ        UINT8_C(0x05)
#define RT3020_FIFO_EN_XYZ       UINT8_C(0x07)

/**\name RT3020 Self test configurations */
#define RT3020_DISABLE_SELF_TEST       UINT8_C(0x00)
#define RT3020_ENABLE_SELF_TEST  	   	 UINT8_C(0x10)

/*! Accel width setting either 16/8 bit mode */
#define RT3020_16_BIT_FIFO_DATA       UINT8_C(0x01)
#define RT3020_8_BIT_FIFO_DATA        UINT8_C(0x00)

/**\name RT3020 bit mask definitions */
#define RT3020_POWER_MODE_STATUS_MSK    UINT8_C(0x07)
#define RT3020_POWER_MODE_STATUS_POS    UINT8_C(0)
#define RT3020_SELF_TEST_MSK         		UINT8_C(0x10) //0x07
#define RT3020_SELF_TEST_POS         		UINT8_C(4) 
#define RT3020_SW_RST_MSK         			UINT8_C(0x10) //0x08
#define RT3020_SW_RST_POS         			UINT8_C(4) 
#define RT3020_POWER_MODE_MSK         	UINT8_C(0x03) //0x08
#define RT3020_POWER_MODE_POS         	UINT8_C(0) 
#define RT3020_ACCEL_ODR_MSK      			UINT8_C(0x70)	//0x09
#define RT3020_ACCEL_ODR_POS         		UINT8_C(4)
#define RT3020_ACCEL_RANGE_MSK    			UINT8_C(0x03)	//0x0A
#define RT3020_ACCEL_RANGE_POS    			UINT8_C(0)
#define RT3020_DATA_FILTER_MSK   				UINT8_C(0x08)
#define RT3020_DATA_FILTER_POS   				UINT8_C(3)
#define RT3020_IRQ2_POL_MSK    					UINT8_C(0x08)	//0x0A
#define RT3020_IRQ2_POL_POS    					UINT8_C(3)
#define RT3020_IRQ1_POL_MSK    					UINT8_C(0x04)	//0x0A
#define RT3020_IRQ1_POL_POS    					UINT8_C(2)
#define RT3020_ILATCH2_SEL_MSK   			  UINT8_C(0xF0)	//0x0B
#define RT3020_ILATCH2_SEL_POS    			UINT8_C(4)
#define RT3020_ILATCH1_SEL_MSK    			UINT8_C(0x0F)	//0x0B
#define RT3020_ILATCH1_SEL_POS   			  UINT8_C(0)
#define RT3020_FIFO_EN_MSK      				UINT8_C(0x80)	//0x22
#define RT3020_FIFO_EN_POS      		    UINT8_C(7)
#define RT3020_FIFO_HR_MSK      				UINT8_C(0x40)	//0x22
#define RT3020_FIFO_HR_POS         			UINT8_C(6)
#define RT3020_FIFO_MODE_MSK      			UINT8_C(0x30)	//0x22
#define RT3020_FIFO_MODE_POS         		UINT8_C(4)
#define RT3020_FIFO_SKIP_MSK      			UINT8_C(0x0C)	//0x22
#define RT3020_FIFO_SKIP_POS         		UINT8_C(2)
#define RT3020_TRIG_SRC_MSK     			  UINT8_C(0x03)	//0x22
#define RT3020_TRIG_SRC_POS     		    UINT8_C(0)
#define RT3020_EN_DRDY_MSK  						UINT8_C(0x02)
#define RT3020_EN_DRDY_POS 							UINT8_C(1)
#define RT3020_EN_FIFO_WM_MSK  					UINT8_C(0x04)
#define RT3020_EN_FIFO_WM_POS  					UINT8_C(2)
#define RT3020_EN_FIFO_OVERRUN_MSK  		UINT8_C(0x08)
#define RT3020_EN_FIFO_OVERRUN_POS  		UINT8_C(3)
#define RT3020_EN_AOI_MSK  							UINT8_C(0x10)
#define RT3020_EN_AOI_POS  							UINT8_C(4)
#define RT3020_EN_TAP_MSK  							UINT8_C(0x20)
#define RT3020_EN_TAP_POS  							UINT8_C(5)
#define RT3020_EN_SIGM_MSK  						UINT8_C(0x40)
#define RT3020_EN_SIGM_POS  						UINT8_C(6)
#define RT3020_TAP_AXES_EN_MSK          UINT8_C(0x3F)
#define RT3020_TAP_AXES_EN_POS       	  UINT8_C(0)
#define RT3020_TAP_THRESH_MSK           UINT8_C(0x7F)
#define RT3020_TAP_THRESH_POS           UINT8_C(0)
#define RT3020_TAP_LIMIT_MSK            UINT8_C(0xFF)
#define RT3020_TAP_LIMIT_POS            UINT8_C(0)
#define RT3020_TAP_LAT_MSK            	UINT8_C(0xFF)
#define RT3020_TAP_LAT_POS            	UINT8_C(0)
#define RT3020_TAP_WIN_MSK            	UINT8_C(0xFF)
#define RT3020_TAP_WIN_POS           		UINT8_C(0)
#define RT3020_SELF_TEST_MSK            UINT8_C(0x10)
#define RT3020_SELF_TEST_POS            UINT8_C(4)

/**\name Macro to SET and GET BITS of a register */
#define RT3020_SET_BITS(reg_data, bitname, data)  ((reg_data & ~(bitname ## _MSK)) | ((data << bitname ## _POS) & bitname ## _MSK))

#define RT3020_GET_BITS(reg_data, bitname)  ((reg_data & (bitname ## _MSK)) >> (bitname ## _POS))

#define RT3020_SET_BITS_POS_0(reg_data, bitname, data)  ((reg_data & ~(bitname ## _MSK)) | (data & bitname ## _MSK))

#define RT3020_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname ## _MSK))

#define RT3020_SET_BIT_VAL_0(reg_data, bitname)  (reg_data & ~(bitname ## _MSK))

#define RT3020_GET_LSB(var)     (uint8_t)(var & RT3020_SET_LOW_BYTE)
#define RT3020_GET_MSB(var)     (uint8_t)((var & RT3020_SET_HIGH_BYTE) >> 8)
/********************************************************/
/*!
 * @brief Interface selection enums
 */
enum rt3020_intf {
	/*! SPI interface */
	RT3020_SPI_INTF,
	/*! I2C interface */
	RT3020_I2C_INTF
};
/********************************************************/
/**\name	TYPE DEFINITIONS */
/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read and write functions of the user
 */
typedef int8_t (*rt3020_com_fptr_t)(void *intf_ptr, uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
				    uint16_t length);
/*!	Delay function pointer */
typedef void (*rt3020_delay_fptr_t)(uint32_t period);
/********************************************************/
/**\name	STRUCTURE DEFINITIONS*/
/*!
 * @brief Sensor selection enums
 */
enum rt3020_sensor {
	RT3020_ACCEL,
	RT3020_TAP_INT,
	RT3020_ACTIVITY_CHANGE_INT,
};

/*!
 * @brief Interrupt channel selection enums
 */
enum rt3020_int_chan {
	RT3020_UNMAP_INT_PIN,
	RT3020_INT_CHANNEL_1,
	RT3020_INT_CHANNEL_2,
	RT3020_MAP_BOTH_INT_PINS
};

/*!
 * @brief Interrupt channel selection enums
 */
enum rt3020_pin_latch {
	RT3020_LATCH_DURATION_NONLATCHED,
	RT3020_LATCH_DURATION_LATCHED,
	RT3020_LATCH_DURATION_2_5_MS,
	RT3020_LATCH_DURATION_5_MS,
	RT3020_LATCH_DURATION_10_MS,
	RT3020_LATCH_DURATION_20_MS,
	RT3020_LATCH_DURATION_40_MS,
	RT3020_LATCH_DURATION_80_MS,
	RT3020_LATCH_DURATION_160_MS,
	RT3020_LATCH_DURATION_320_MS,
	RT3020_LATCH_DURATION_640_MS,
	RT3020_LATCH_DURATION_1_3_S,
	RT3020_LATCH_DURATION_2_5_S,
	RT3020_LATCH_DURATION_5_1_S,
	RT3020_LATCH_DURATION_10_25_S,
	RT3020_LATCH_DURATION_20_5_S
};

/*!
 * @brief Interrupt pin hardware configurations
 */
struct rt3020_int_pin_conf {
	/*! Interrupt channel selection enums */
	enum rt3020_int_chan int_chan;
	/*! Interrupt pin polarity configuration
	 * Assignable Macros :
	 *  - RT3020_INT_HIGH_ACTIVE
	 *  - RT3020_INT_LOW_ACTIVE
	 */
	uint8_t pin_pol;
	/*! Interrupt pin latch duration configuration*/
	uint8_t pin_latch;
};

/*!
 * @brief Accel basic configuration
 */
struct rt3020_acc_conf {
	/*! Output data rate
	 * Assignable macros :
	 *  - RT3020_ODR_6_25HZ  - RT3020_ODR_12_5HZ 	-	RT3020_ODR_25HZ 
			- RT3020_ODR_50HZ		 - RT3020_ODR_100HZ   - RT3020_ODR_200HZ
	 *  - RT3020_ODR_400HZ
	 */
	uint8_t odr;
	/*! Range of sensor
	 * Assignable macros :
	 *  - RT3020_2G_RANGE   - RT3020_8G_RANGE
	 *  - RT3020_4G_RANGE   - RT3020_16G_RANGE
	 */
	uint8_t range;
	/*! Filter setting for data source
	 * Assignable Macros :
	 * - RT3020_DATA_SRC_NORMAL
	 * - RT3020_DATA_SRC_LPF 
	 */
	uint8_t data_src;
};

/*!
 * @brief Tap interrupt configurations
 */
struct rt3020_tap_conf {
	uint8_t axes_sel;
	uint8_t threshold;
	uint8_t limit;
	uint8_t latency;
	uint8_t window;
	/*! Interrupt channel to be mapped */
	enum rt3020_int_chan int_chan;
};

/*!
 * @brief Union of sensor Configurations
 */
union rt3020_set_param {
	/* Accel configurations */
	struct rt3020_acc_conf accel;
	/* TAP configurations */
	struct rt3020_tap_conf tap;
};

/*!
 * @brief Sensor selection and their configurations
 */
struct rt3020_sensor_conf {
	/* Sensor selection */
	enum rt3020_sensor type;
	/* Sensor configuration */
	union rt3020_set_param param;
};

/*!
 * @brief enum to select device settings
 */
enum rt3020_device {
	RT3020_AUTOWAKEUP_TIMEOUT,
	RT3020_AUTOWAKEUP_INT,
	RT3020_AUTO_LOW_POWER,
	RT3020_INT_PIN_CONF,
	RT3020_INT_OVERRUN_CONF,
	RT3020_FIFO_CONF
};

/*!
 * @brief FIFO configurations
 */
struct rt3020_fifo_conf {
	/*! Select FIFO configurations to enable/disable*/
	uint8_t conf_regs;
	/*! Enable/ disable selected FIFO configurations
	 * Assignable Macros :
	 *   - RT3020_ENABLE
	 *   - RT3020_DISABLE
	 */
	uint8_t conf_status;
	/*! Value to set the water-mark */
	uint8_t fifo_watermark;
	/*! Interrupt pin mapping for FIFO full interrupt */
	enum rt3020_int_chan fifo_full_channel;
	/*! Interrupt pin mapping for FIFO water-mark interrupt */
	enum rt3020_int_chan fifo_wm_channel;
};

/*!
 * @brief Interrupt overrun configurations
 */
struct rt3020_int_overrun {
	/*! Interrupt pin mapping for interrupt overrun */
	enum rt3020_int_chan int_chan;
};

/*!
 * @brief Union of device configuration parameters
 */
union rt3020_device_params {
	/* Interrupt pin configurations */
	struct rt3020_int_pin_conf int_conf;
	/* FIFO configuration */
	struct rt3020_fifo_conf fifo_conf;
	/* Interrupt overrun configuration */
	struct rt3020_int_overrun overrun_int;
};

/*!
 * @brief RT3020 device configuration
 */
struct rt3020_device_conf {
	/* Device feature selection */
	enum rt3020_device type;
	/* Device feature configuration */
	union rt3020_device_params param;
};

/*!
 * @brief RT3020 sensor data
 */
struct rt3020_sensor_data {
	/*! X-axis sensor data */
	int16_t x;
	/*! Y-axis sensor data */
	int16_t y;
	/*! Z-axis sensor data */
	int16_t z;
	/*! sensor time */
	uint32_t sensortime;
};

/*!
 * @brief RT3020 interrupt selection
 */
enum rt3020_int_type {
	/* DRDY interrupt */
	RT3020_DRDY_INT_EN,
	/* FIFO watermark interrupt */
	RT3020_FIFO_WM_INT_EN,
	/* FIFO overrun interrupt */
	RT3020_FIFO_OVERRUN_INT_EN,
	/* AOI interrupt */
	RT3020_AOI_INT_EN,
	/* TAP interrupt */
	RT3020_TAP_INT_EN,
	/* SIGM interrupt */
	RT3020_SIGM_INT_EN,
	/* Inactivity interrupt */
	RT3020_INACTIVITY_INT_EN,
	/* Activity interrupt */
	RT3020_ACTIVITY_INT_EN,
};

/*!
 * @brief Interrupt enable/disable configurations
 */
struct rt3020_int_enable {
	/*! Enum to choose the interrupt to be enabled */
	enum rt3020_int_type type;
	/*! Enable/ disable selected interrupts
	 * Assignable Macros :
	 *   - RT3020_ENABLE
	 *   - RT3020_DISABLE
	 */
	uint8_t conf;
};

/*!
 * @brief rt3020 device structure
 */
struct rt3020_dev {
	/*! Chip Id */
	uint8_t chip_id;
	/*! Device Id */
	uint8_t dev_id;
	/*! SPI/I2C Interface selection */
	enum rt3020_intf intf;
	/*! Interface handle pointer */
	void *intf_ptr;
	/*! Decide SPI or I2C read mechanism */
	uint8_t dummy_byte;
	/*! Bus read function pointer */
	rt3020_com_fptr_t read;
	/*! Bus write function pointer */
	rt3020_com_fptr_t write;
	/*! delay(in ms) function pointer */
	rt3020_delay_fptr_t delay_ms;
};

#endif /* RT3020_DEFS_H_ */
/** @}*/
/** @}*/
