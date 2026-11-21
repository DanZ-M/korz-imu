/*
 * korz-imu: motion sensor (MPU6050) as a pointing device.
 *   TILT: the tilt away from the neutral pose moves the pointer, like the joystick.
 *   GYRO: air mouse, the rotation speed moves the pointer.
 * The neutral pose / gyro offsets are measured about a second after power-up: keep the
 * dongle still then. ZMK's `zmk,input-listener` turns the events into HID mouse reports.
 */
#define DT_DRV_COMPAT korz_imu_mouse

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(korz_imu_mouse, LOG_LEVEL_INF);

#if DT_HAS_COMPAT_STATUS_OKAY(korz_imu_mouse)

#define CAL_SAMPLES 40

struct imu_cfg {
	const struct device *sensor;
	struct gpio_dt_spec en;
	bool has_en;
	bool invert[2];
	bool swap;
};

struct imu_data {
	const struct device *dev;
	struct k_work_delayable work;
	int32_t sum[5]; /* ax ay gx gy gz */
	int32_t base[5];
	uint16_t cal_n;
	int32_t acc[2];
	bool en_prev;
	uint16_t dbg_n;
};

static int32_t to_mg(const struct sensor_value *v)
{
	return (int32_t)(sensor_value_to_milli(v) * 1000 / 9807);
}

static int32_t to_mdps(const struct sensor_value *v)
{
	return (int32_t)(sensor_value_to_milli(v) * 57296 / 1000);
}

#ifdef CONFIG_KORZ_IMU_MODE_TILT
/* tilt (mg away from neutral) -> 0..1000 after the dead zone */
static int32_t tilt_deflection(int32_t d)
{
	const int32_t range = CONFIG_KORZ_IMU_TILT_RANGE;
	const int32_t dz = range * CONFIG_KORZ_IMU_TILT_DEADZONE / 100;
	int32_t a = abs(d);

	if (a <= dz) {
		return 0;
	}
	a = (a - dz) * 1000 / (range - dz);
	return a > 1000 ? 1000 : a;
}
#endif

static int32_t whole_steps(int32_t milli, int32_t *acc)
{
	*acc += milli;
	int32_t out = *acc / 1000;

	*acc -= out * 1000;
	return out;
}

static void imu_tick(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct imu_data *data = CONTAINER_OF(dwork, struct imu_data, work);
	const struct device *dev = data->dev;
	const struct imu_cfg *cfg = dev->config;
	struct sensor_value acc[3], gyr[3];
	int32_t v[5];

	k_work_reschedule(&data->work, K_MSEC(CONFIG_KORZ_IMU_POLL_MS));

	if (!device_is_ready(cfg->sensor) || sensor_sample_fetch(cfg->sensor) != 0 ||
	    sensor_channel_get(cfg->sensor, SENSOR_CHAN_ACCEL_XYZ, acc) != 0 ||
	    sensor_channel_get(cfg->sensor, SENSOR_CHAN_GYRO_XYZ, gyr) != 0) {
		return;
	}
	v[0] = to_mg(&acc[0]);
	v[1] = to_mg(&acc[1]);
	v[2] = to_mdps(&gyr[0]);
	v[3] = to_mdps(&gyr[1]);
	v[4] = to_mdps(&gyr[2]);

	if (data->cal_n < CAL_SAMPLES) {
		for (int i = 0; i < 5; i++) {
			data->sum[i] += v[i];
		}
		if (++data->cal_n == CAL_SAMPLES) {
			for (int i = 0; i < 5; i++) {
				data->base[i] = data->sum[i] / CAL_SAMPLES;
			}
			LOG_INF("imu neutral: ax=%d ay=%d mg, gyro offset %d %d %d mdps",
				data->base[0], data->base[1], data->base[2], data->base[3],
				data->base[4]);
		}
		return;
	}

	/* optional clutch: move only while the button is held, neutral = pose at the press */
	if (cfg->has_en) {
		bool en = gpio_pin_get_dt(&cfg->en) > 0;

		if (en && !data->en_prev) {
			data->base[0] = v[0];
			data->base[1] = v[1];
		}
		data->en_prev = en;
		if (!en) {
			data->acc[0] = data->acc[1] = 0;
			return;
		}
	}

#ifdef CONFIG_KORZ_IMU_DEBUG
	if (++data->dbg_n >= 500 / CONFIG_KORZ_IMU_POLL_MS) {
		data->dbg_n = 0;
		LOG_INF("imu ax=%d ay=%d mg | gyro %d %d %d mdps", v[0], v[1], v[2], v[3], v[4]);
	}
#endif

	int32_t milli[2] = {0, 0};

#ifdef CONFIG_KORZ_IMU_MODE_TILT
	int32_t d[2] = {v[0] - data->base[0], v[1] - data->base[1]};

	if (cfg->swap) {
		int32_t t = d[0];

		d[0] = d[1];
		d[1] = t;
	}
	for (int i = 0; i < 2; i++) {
		int32_t n = tilt_deflection(d[i]);
		int32_t n2 = n * n / 1000;
		int32_t m = n2 * CONFIG_KORZ_IMU_SPEED * CONFIG_KORZ_IMU_POLL_MS / 1000;

		milli[i] = n == 0 ? 0 : (((d[i] < 0) != cfg->invert[i]) ? -m : m);
	}
#else
	/* yaw (gyro z) moves X, pitch (gyro x) moves Y */
	int32_t r[2] = {v[4] - data->base[4], v[2] - data->base[2]};

	if (cfg->swap) {
		int32_t t = r[0];

		r[0] = r[1];
		r[1] = t;
	}
	for (int i = 0; i < 2; i++) {
		if (abs(r[i]) < CONFIG_KORZ_IMU_GYRO_DEADZONE) {
			milli[i] = 0;
			continue;
		}
		/* mdps * ms / 1000 = milli-degrees turned in this tick; times px per degree */
		int32_t m = r[i] * CONFIG_KORZ_IMU_POLL_MS / 1000 * CONFIG_KORZ_IMU_GYRO_GAIN;

		milli[i] = cfg->invert[i] ? -m : m;
	}
#endif

	int32_t sx = whole_steps(milli[0], &data->acc[0]);
	int32_t sy = whole_steps(milli[1], &data->acc[1]);

	if (sx != 0) {
		input_report_rel(dev, INPUT_REL_X, sx, sy == 0, K_FOREVER);
	}
	if (sy != 0) {
		input_report_rel(dev, INPUT_REL_Y, sy, true, K_FOREVER);
	}
}

static int imu_init(const struct device *dev)
{
	const struct imu_cfg *cfg = dev->config;
	struct imu_data *data = dev->data;

	data->dev = dev;
	if (cfg->has_en) {
		if (!gpio_is_ready_dt(&cfg->en) || gpio_pin_configure_dt(&cfg->en, GPIO_INPUT) != 0) {
			LOG_ERR("enable GPIO failed");
			return -ENODEV;
		}
	}
	k_work_init_delayable(&data->work, imu_tick);
	k_work_schedule(&data->work, K_MSEC(1500)); /* let the sensor and the dongle settle */
	return 0;
}

#define IMU_INIT(n)                                                                                \
	static struct imu_data imu_data_##n;                                                       \
	static const struct imu_cfg imu_cfg_##n = {                                                \
		.sensor = DEVICE_DT_GET(DT_INST_PHANDLE(n, sensor)),                               \
		.en = GPIO_DT_SPEC_INST_GET_OR(n, enable_gpios, {0}),                              \
		.has_en = DT_INST_NODE_HAS_PROP(n, enable_gpios),                                  \
		.invert = {DT_INST_PROP(n, invert_x), DT_INST_PROP(n, invert_y)},                  \
		.swap = DT_INST_PROP(n, swap_xy),                                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, imu_init, NULL, &imu_data_##n, &imu_cfg_##n, POST_KERNEL,         \
			      CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IMU_INIT)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(korz_imu_mouse) */
