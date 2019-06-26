
#include "sensor_rt_rt3020.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.rt.rt3020"
#define DBG_COLOR
#include <rtdbg.h>

#define GRAVITY_EARTH (9.80665f)

#define PKG_USING_RT3020_ACCE

static void rt_delay_ms(uint32_t period)
{
    rt_thread_mdelay(period);
}

static int8_t rt_i2c_write_reg(void *intf_ptr, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

	if (rt_i2c_transfer(intf_ptr, msgs, 2) != 2)
		{
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int8_t rt_i2c_read_reg(void *intf_ptr, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

	if (rt_i2c_transfer(intf_ptr, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static struct rt3020_dev *_rt3020_create(struct rt_sensor_intf *intf)
{
    struct rt3020_dev *_rt3020_dev = RT_NULL;
    struct rt_i2c_bus_device *i2c_bus_dev = RT_NULL;
    int8_t rslt = RT3020_OK;
    struct rt3020_sensor_conf conf;

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        LOG_E("can not find device %s", intf->dev_name);
        return RT_NULL;
    }

    _rt3020_dev = rt_calloc(1, sizeof(struct rt3020_dev));
    if (_rt3020_dev == RT_NULL)
    {
        LOG_E("rt3020 dev memory allocation failed");
        return RT_NULL;
    }

    _rt3020_dev->dev_id   = (rt_uint32_t)(intf->user_data) & 0xff;
    _rt3020_dev->intf     = RT3020_I2C_INTF;
    _rt3020_dev->intf_ptr = i2c_bus_dev;
    _rt3020_dev->read     = rt_i2c_read_reg;
    _rt3020_dev->write    = rt_i2c_write_reg;
    _rt3020_dev->delay_ms = rt_delay_ms;

    rslt = rt3020_init(_rt3020_dev);
    if (rslt == RT3020_OK)
    {
        rslt = rt3020_soft_reset(_rt3020_dev);
				rt3020_set_power_mode(RT3020_NORMAL_MODE, _rt3020_dev);
        /* Select the type of configuration to be modified */
        conf.type = RT3020_ACCEL;

        /* Get the accelerometer configurations which are set in the sensor */
        rslt = rt3020_get_sensor_conf(&conf, 1, _rt3020_dev);
        /* Modify the desired configurations as per macros
         * available in rt3020_defs.h file */
        conf.param.accel.odr = RT3020_ODR_25HZ;
        conf.param.accel.range = RT3020_4G_RANGE;
        conf.param.accel.data_src = RT3020_DATA_SRC_NORMAL;
				//LOG_D("odr=%2x, range=%2x, src=%2x\r\n", conf.param.accel.odr, conf.param.accel.range,  conf.param.accel.data_src);

        /* Set the desired configurations to the sensor */
        rslt = rt3020_set_sensor_conf(&conf, 1, _rt3020_dev);
        //rt3020_set_power_mode(RT3020_POWER_DOWN_MODE, _rt3020_dev);

        return _rt3020_dev;
    }
    else
    {
        LOG_E("rt3020 init failed");
        rt_free(_rt3020_dev);
        return RT_NULL;
    }
}

static rt_err_t _rt3020_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    struct rt3020_dev *_rt3020_dev = sensor->parent.user_data;
    struct rt3020_sensor_conf conf;
    uint8_t odr_ctr;

    if (odr <= 6)
        odr_ctr = RT3020_ODR_6_25HZ;
    else if (odr <= 12)
        odr_ctr = RT3020_ODR_12_5HZ;
    else if (odr <= 25)
        odr_ctr = RT3020_ODR_25HZ;
    else if (odr <= 50)
        odr_ctr = RT3020_ODR_50HZ;
    else if (odr <= 100)
        odr_ctr = RT3020_ODR_100HZ;
    else if (odr <= 200)
        odr_ctr = RT3020_ODR_200HZ;
    else 
        odr_ctr = RT3020_ODR_400HZ;

    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        conf.type = RT3020_ACCEL;

        /* Get the accelerometer configurations which are set in the sensor */
        rt3020_get_sensor_conf(&conf, 1, _rt3020_dev);

        conf.param.accel.odr = odr_ctr;

        /* Set the desired configurations to the sensor */
        rt3020_set_sensor_conf(&conf, 1, _rt3020_dev);
        return RT_EOK;
    }

    return RT_EOK;
}

static rt_err_t _rt3020_set_range(rt_sensor_t sensor, rt_uint16_t range)
{
    struct rt3020_dev *_rt3020_dev = sensor->parent.user_data;
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        struct rt3020_sensor_conf conf;
        uint8_t range_ctr;

        if (range <= 2000)
            range_ctr = RT3020_2G_RANGE;
        else if (range <= 4000)
            range_ctr = RT3020_4G_RANGE;
        else if (range <= 8000)
            range_ctr = RT3020_8G_RANGE;
        else
            range_ctr = RT3020_16G_RANGE;

        conf.type = RT3020_ACCEL;

        /* Get the accelerometer configurations which are set in the sensor */
        rt3020_get_sensor_conf(&conf, 1, _rt3020_dev);

        conf.param.accel.range = range_ctr;

        /* Set the desired configurations to the sensor */
        rt3020_set_sensor_conf(&conf, 1, _rt3020_dev);
        return RT_EOK;
    }

    return RT_EOK;
}

static rt_err_t _rt3020_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    struct rt3020_dev *_rt3020_dev = sensor->parent.user_data;
    int8_t rslt = 0;
    if (power == RT_SENSOR_POWER_DOWN)
    {
        rslt = rt3020_set_power_mode(RT3020_POWER_DOWN_MODE, _rt3020_dev);
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        rslt = rt3020_set_power_mode(RT3020_NORMAL_MODE, _rt3020_dev);
    }
    else if (power == RT_SENSOR_POWER_LOW)
    {
        rslt = rt3020_set_power_mode(RT3020_WAKEUP_MODE, _rt3020_dev);
    }
    else
    {
        LOG_W("Unsupported mode, code is %d", power);
        return -RT_ERROR;
    }
    return rslt;
}


static rt_size_t rt3020_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct rt3020_dev *_rt3020_dev = sensor->parent.user_data;
    struct rt_sensor_data *data = buf;

    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        struct rt3020_sensor_data comp_data;
        rt3020_get_accel_data(&comp_data, _rt3020_dev);

        data->type = RT_SENSOR_CLASS_ACCE;
        data->data.acce.x = comp_data.x;
        data->data.acce.y = comp_data.y;
        data->data.acce.z = comp_data.z;
        data->timestamp = rt_sensor_get_ts();
    }
    return 1;
}

static rt_err_t rt3020_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    struct rt3020_dev *_rt3020_dev = sensor->parent.user_data;
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
				LOG_D("RT_SENSOR_CTRL_GET_ID! \n");
        *(rt_uint8_t *)args = _rt3020_dev->chip_id;
        break;
    case RT_SENSOR_CTRL_SET_ODR:
				LOG_D("RT_SENSOR_CTRL_SET_ODR! \n");
        result = _rt3020_set_odr(sensor, (rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
				LOG_D("RT_SENSOR_CTRL_SET_RANGE! \n");
        result = _rt3020_set_range(sensor, (rt_uint32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
				LOG_D("RT_SENSOR_CTRL_SET_POWER! \n");
        result = _rt3020_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    default:
        return -RT_EINVAL;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    rt3020_fetch_data,
    rt3020_control
};

int rt_hw_rt3020_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_acce = RT_NULL;//, sensor_step = RT_NULL;
    struct rt3020_dev *_rt3020_dev = RT_NULL;

    _rt3020_dev = _rt3020_create(&cfg->intf);
    if (_rt3020_dev == RT_NULL)
    {
        LOG_E("sensor create failed");
        return -RT_ERROR;
    }	

#ifdef PKG_USING_RT3020_ACCE
    /* accelerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            return -RT_ERROR;

        sensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_acce->info.model      = "rt3020_acce";
        sensor_acce->info.unit       = RT_SENSOR_UNIT_PA;
        sensor_acce->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_acce->info.range_max  = 16000;
        sensor_acce->info.range_min  = 2000;
        sensor_acce->info.period_min = 100;

        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));
        sensor_acce->ops = &sensor_ops;
        result = rt_hw_sensor_register(sensor_acce, name, RT_DEVICE_FLAG_RDWR, _rt3020_dev);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            rt_free(sensor_acce);
            return -RT_ERROR;
        }
				LOG_I("rt3020 hw sensor register done \r\n");
    }
#endif
    return RT_EOK;
}
