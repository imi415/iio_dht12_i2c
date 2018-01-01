#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define DHT12_ALL_CHANNEL_MASK GENMASK(1, 0)

struct dht12_data {
	struct i2c_client *client;
	struct mutex lock;
	s16 buffer[2]; /* 2x16-bit channels */
};

struct dht12_sensor_data {
	s16 hum_data;
	s16 temp_data;
};

static const struct iio_chan_spec dht12_channels[] = {
	{
		.type = IIO_HUMIDITYRELATIVE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				     BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static int dht12_read_data(struct dht12_data *data,
			   struct dht12_sensor_data *sensor_data)
{
	int ret;
	u8 tx_buf[1] = {0x00};
	u8 rx_buf[5]; /* Humid MSB, Humid LSB, Temp MSB, Temp LSB, CRC */
	u8 crc;

	mutex_lock(&data->lock);
	ret = i2c_master_send(data->client, tx_buf, sizeof(tx_buf));
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to send read request\n");
		goto exit_unlock;
	}
	usleep_range(10000, 20000);
	ret = i2c_master_recv(data->client, rx_buf, sizeof(rx_buf));
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to read sensor data\n");
		goto exit_unlock;
	}
	usleep_range(10000, 20000);
	mutex_unlock(&data->lock);
	crc = rx_buf[0] + rx_buf[1] + rx_buf[2] + rx_buf[3];
	if (crc != rx_buf[4]) {
		dev_err(&data->client->dev, "failed to verify sensor data\n");
		return -EIO;
	}

	sensor_data->hum_data = (rx_buf[0] * 100) + rx_buf[1];
	sensor_data->temp_data = (rx_buf[2] * 100) + rx_buf[3];

	return ret;

exit_unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static irqreturn_t dht12_trigger_handler(int irq, void *p)
{
	int i;
	int ret;
	int bit;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct dht12_data *data = iio_priv(indio_dev);
	struct dht12_sensor_data sensor_data;

	ret = dht12_read_data(data, &sensor_data);
	/* Read twice to get real-time data. */
	ret = dht12_read_data(data, &sensor_data);
	if (ret < 0)
		goto err;
	mutex_lock(&data->lock);
	if (*(indio_dev->active_scan_mask) == DHT12_ALL_CHANNEL_MASK) {
		data->buffer[0] = sensor_data.hum_data;
		data->buffer[1] = sensor_data.temp_data;
	} else {
		i = 0;
		for_each_set_bit(bit, indio_dev->active_scan_mask,
				 indio_dev->masklength) {
			data->buffer[i] = (bit ? sensor_data.temp_data :
						 sensor_data.hum_data);
			i++;
		}
	}
	mutex_unlock(&data->lock);

	iio_push_to_buffers_with_timestamp(indio_dev, data->buffer, pf->timestamp);
err:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int dht12_read_raw(struct iio_dev *indio_dev,
		      struct iio_chan_spec const *chan,
		      int *val, int *val2, long mask)
{
	int ret;
	struct dht12_sensor_data sensor_data;
	struct dht12_data * data = iio_priv(indio_dev);

	switch (mask) {
		case IIO_CHAN_INFO_RAW:
			ret = dht12_read_data(data, &sensor_data);
			if (ret < 0)
				return ret;
			*val = (chan->type == IIO_HUMIDITYRELATIVE) ?
					sensor_data.hum_data : sensor_data.temp_data;
			return IIO_VAL_INT;
		case IIO_CHAN_INFO_SCALE:
			*val = 100;
			return IIO_VAL_INT;
	}

	return -EINVAL;
}

static const struct iio_info dht12_info = {
	.read_raw = dht12_read_raw,
};

static int dht12_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct iio_dev *indio_dev;
	struct dht12_data *data;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev) {
		dev_err(&client->dev, "iio allocation failed!\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	data->client = client;
	i2c_set_clientdata(client, indio_dev);
	mutex_init(&data->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &dht12_info;
	indio_dev->name = "dht12";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = dht12_channels;
	indio_dev->num_channels = ARRAY_SIZE(dht12_channels);

	ret = iio_triggered_buffer_setup(indio_dev, iio_pollfunc_store_time,
					  dht12_trigger_handler, NULL);
	if (ret < 0) {
		dev_err(&client->dev, "iio triggered buffer setup failed\n");
		return ret;
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto err_buffer_cleanup;
	return 0;

err_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
	return ret;
}

static int dht12_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);

	return 0;
}

static const struct i2c_device_id dht12_i2c_id[] = {
	{"dht12", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, dht12_i2c_id);

static const struct acpi_device_id dht12_acpi_id[] = {
	{"AOS0012", 0},
	{}
};

MODULE_DEVICE_TABLE(acpi, dht12_acpi_id);

static struct i2c_driver dht12_driver = {
	.driver = {
		.name = "dht12",
		.acpi_match_table = ACPI_PTR(dht12_acpi_id),
	},
	.probe = dht12_probe,
	.remove = dht12_remove,
	.id_table = dht12_i2c_id,
};

module_i2c_driver(dht12_driver);

MODULE_AUTHOR("imi415 <imi415@imi.moe>");
MODULE_DESCRIPTION("Aosong DHT12 relative humidity and temperature");
MODULE_LICENSE("GPL v2");

