#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <assert.h>

#include "imu.h"
#include "usb.h"

static inline float encode(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

static void fetch_and_display(const struct device *dev)
{
  static imu_t imu_processed_data;
	static struct sensor_value a_x, a_y, a_z;
	static struct sensor_value r_x, r_y, r_z;
	static int trig_cnt;
  int rc;

	trig_cnt++;

	/* accel */
	rc = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &a_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &a_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &a_z);

	/* gyro */
	rc |= sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &r_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &r_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &r_z);

  // Only send the data if we have both the gyro and accel data
  if(0 == rc)
  {
    // x/p, y and y/r and z/y are on the same "axis"
    imu_processed_data.a_x = encode(&a_x);
    imu_processed_data.a_y = encode(&a_y);
    imu_processed_data.a_z = encode(&a_z);

    imu_processed_data.r_p = encode(&r_x);
    imu_processed_data.r_r = encode(&r_y);
    imu_processed_data.r_y = encode(&r_z);
    
    imu_processed_data.cpu_cycles_since_boot = k_cycle_get_32();

    if(0)
    {
      printk("X (A:%f) (R:%f)\n", imu_processed_data.a_x, imu_processed_data.r_p);
      printk("Y (A:%f) (R:%f)\n", imu_processed_data.a_y, imu_processed_data.r_r);
      printk("Z (A:%f) (R:%f)\n", imu_processed_data.a_z, imu_processed_data.r_y);
      printk("\n");
    }
      
    k_msgq_put(&imu_ouput_msgq, &imu_processed_data, K_NO_WAIT);
  }
}

static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trig)
{
	fetch_and_display(dev);
}

void init_imu()
{ 
	const struct device * volatile const dev_imu = DEVICE_DT_GET(DT_NODELABEL(imu_0));
	if (!device_is_ready(dev_imu))
  {
		printk("%s: device not ready.\n", dev_imu->name);
		assert(0);
	}

	struct sensor_trigger trig;
	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

  if (sensor_trigger_set(dev_imu, &trig, trigger_handler) != 0)
  {
		printk("Could not set sensor type and channel\n");
    assert(0);
		return;
	}
}
