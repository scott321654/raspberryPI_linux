/*********************************************************************
 * Author: Scott Lin <digitalcamper@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 ********************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/jiffies.h>

/* I2C command bytes */
#define HM3301_I2C_MODE_ADDR 0x80
#define HM3301_I2C_MODE_CMD 0x88
#define HM3301_READ_DATA_CMD 0x81

typedef struct SensorData {
	u16 reserved;
	u16 sensorNumber;
	u16 pm1_cf1;
	u16 pm25_cf1;
	u16 pm10_cf1;
	u16 pm1_atmos;
	u16 pm25_atmos;
	u16 pm10_atmos;
	u16 particles_03um;
	u16 particles_05um;
	u16 particles_10um;
	u16 particles_25um;
	u16 particles_50um;
	u16 particles_100um;
	u8 checksum;
} SensorData;

typedef struct hm3301 {
	struct i2c_client *client;
	struct mutex lock;
	unsigned long last_update;
	SensorData sensor_data;
	u32 pm25;
	u32 pm100;
	bool valid;
} hm3301;

// Parse the received data
void parse_sensor_data(u8 *data, struct SensorData *sensor_data)
{
	sensor_data->reserved = (data[0] << 8) | data[1];
	sensor_data->sensorNumber = (data[2] << 8) | data[3];
	sensor_data->pm1_cf1 = (data[4] << 8) | data[5];
	sensor_data->pm25_cf1 = (data[6] << 8) | data[7];
	sensor_data->pm10_cf1 = (data[8] << 8) | data[9];
	sensor_data->pm1_atmos = (data[10] << 8) | data[11];
	sensor_data->pm25_atmos = (data[12] << 8) | data[13];
	sensor_data->pm10_atmos = (data[14] << 8) | data[15];
	sensor_data->particles_03um = (data[16] << 8) | data[17];
	sensor_data->particles_05um = (data[18] << 8) | data[19];
	sensor_data->particles_10um = (data[20] << 8) | data[21];
	sensor_data->particles_25um = (data[22] << 8) | data[23];
	sensor_data->particles_50um = (data[24] << 8) | data[25];
	sensor_data->particles_100um = (data[26] << 8) | data[27];
	sensor_data->checksum = (data[28]);
}

static int hm3301_measure_concentration(struct device *dev)
{
	__u8 buffer[29];
	hm3301 *hm3301 = dev_get_drvdata(dev);
	struct i2c_client *client = hm3301->client;
	int ret;

	mutex_lock(&hm3301->lock);

	ret = i2c_smbus_read_i2c_block_data(client, HM3301_READ_DATA_CMD,
					    sizeof(buffer), buffer);
	if (ret < 0) {
		// pr_err("Failed to read block data from register 0x81, error: %d\n",
		//        ret);
		goto out;
	}

	parse_sensor_data(buffer, &hm3301->sensor_data);
	pr_info("Reserved: %u\n", hm3301->sensor_data.reserved);
	pr_info("Sensor Number: %u\n", hm3301->sensor_data.sensorNumber);
	pr_info("PM1.0 concentration (CF=1): %u\n",
		hm3301->sensor_data.pm1_cf1);
	pr_info("PM2.5 concentration (CF=1): %u\n",
		hm3301->sensor_data.pm25_cf1);
	pr_info("PM10 concentration (CF=1): %u\n",
		hm3301->sensor_data.pm10_cf1);
	pr_info("PM1.0 concentration (Atmospheric environment): %u\n",
		hm3301->sensor_data.pm1_atmos);
	pr_info("PM2.5 concentration (Atmospheric environment): %u\n",
		hm3301->sensor_data.pm25_atmos);
	pr_info("PM10 concentration (Atmospheric environment): %u\n",
		hm3301->sensor_data.pm10_atmos);
	pr_info("Particles with diameter 0.3um or above: %u\n",
		hm3301->sensor_data.particles_03um);
	pr_info("Particles with diameter 0.5um or above: %u\n",
		hm3301->sensor_data.particles_05um);
	pr_info("Particles with diameter 1.0um or above: %u\n",
		hm3301->sensor_data.particles_10um);
	pr_info("Particles with diameter 2.5um or above: %u\n",
		hm3301->sensor_data.particles_25um);
	pr_info("Particles with diameter 5.0um or above: %u\n",
		hm3301->sensor_data.particles_50um);
	pr_info("Particles with diameter 10um or above: %u\n",
		hm3301->sensor_data.particles_100um);
	pr_info("Checksum: %u\n", hm3301->sensor_data.checksum);
out:
	mutex_unlock(&hm3301->lock);

	return 0;
}

static ssize_t hm3301_pm25_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	hm3301 *hm3301 = dev_get_drvdata(dev);
	int ret;

	ret = hm3301_measure_concentration(dev);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%u\n", hm3301->sensor_data.pm25_atmos);
}

static ssize_t hm3301_pm100_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	hm3301 *hm3301 = dev_get_drvdata(dev);
	int ret;

	ret = hm3301_measure_concentration(dev);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%u\n", hm3301->sensor_data.pm10_atmos);
}

/* sysfs attributes */
static SENSOR_DEVICE_ATTR_RO(temp1_input, hm3301_pm25, 0);
static SENSOR_DEVICE_ATTR_RO(humidity1_input, hm3301_pm100, 0);

static struct attribute *hm3301_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_humidity1_input.dev_attr.attr, NULL
};

ATTRIBUTE_GROUPS(hm3301);

static int hm3301_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	hm3301 *hm3301;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev,
			"adapter does not support SMBus word transactions\n");
		return -ENODEV;
	}

	ret = i2c_smbus_write_byte_data(client, HM3301_I2C_MODE_ADDR,
					HM3301_I2C_MODE_CMD);

	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to write initialization command\n");
		return ret;
	} else {
		printk(KERN_DEBUG "FUNC:%s Slave Addr|0x%2x\n", __func__, client->addr);
	}

	hm3301 = devm_kzalloc(dev, sizeof(*hm3301), GFP_KERNEL);
	if (!hm3301)
		return -ENOMEM;

	hm3301->client = client;

	mutex_init(&hm3301->lock);

	hwmon_dev = devm_hwmon_device_register_with_groups(
		dev, client->name, hm3301, hm3301_groups);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

/* Device ID table */
static const struct i2c_device_id hm3301_id[] = { { "hm3301", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, hm3301_id);

static struct i2c_driver hm3301_driver = {
	.driver.name = "hm3301",
	.probe_new = hm3301_probe,
	.id_table = hm3301_id,
};

module_i2c_driver(hm3301_driver);

MODULE_AUTHOR("Scott Lin <digitalcamper@gmail.com>");
MODULE_DESCRIPTION("hm3301 Laser PM2.5 sensor driver");
MODULE_LICENSE("GPL");
