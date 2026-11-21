/*
 * korz-imu: MPU6050 bring-up. Prints acceleration (milli-g) and rotation
 * (milli-degrees per second) over the USB serial log (CONFIG_ZMK_USB_LOGGING).
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(korz_imu, LOG_LEVEL_INF);

#if DT_HAS_COMPAT_STATUS_OKAY(invensense_mpu6050)

static void imu_thread(void *a, void *b, void *c)
{
	const struct device *imu = DEVICE_DT_GET_ANY(invensense_mpu6050);
	struct sensor_value acc[3], gyr[3];

	if (!device_is_ready(imu)) {
		LOG_ERR("MPU6050 not ready: check SDA/SCL, 3.3 V and the address (AD0 to GND = 0x68)");
		return;
	}
	LOG_INF("MPU6050 ready");

	while (true) {
		if (sensor_sample_fetch(imu) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyr) == 0) {
			/* m/s^2 -> milli-g: * 1000 / 9.80665 */
			LOG_INF("acc mg x=%ld y=%ld z=%ld | gyro mdps x=%ld y=%ld z=%ld",
				(long)(sensor_value_to_milli(&acc[0]) * 1000 / 9807),
				(long)(sensor_value_to_milli(&acc[1]) * 1000 / 9807),
				(long)(sensor_value_to_milli(&acc[2]) * 1000 / 9807),
				(long)(sensor_value_to_milli(&gyr[0]) * 57296 / 1000),
				(long)(sensor_value_to_milli(&gyr[1]) * 57296 / 1000),
				(long)(sensor_value_to_milli(&gyr[2]) * 57296 / 1000));
		} else {
			LOG_ERR("MPU6050 read failed");
		}
		k_msleep(CONFIG_KORZ_IMU_PERIOD_MS);
	}
}

K_THREAD_DEFINE(korz_imu_tid, 1536, imu_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO,
		0, 2000);

#endif
