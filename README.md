# korz-imu

ZMK module for the MPU6050 (or any Zephyr accelerometer + gyroscope sensor).

| File | What it is |
|---|---|
| `src/imu_mouse.c` | `korz,imu-mouse`: the sensor as a pointing device. Tilt mode (neutral pose taken at start, optional clutch button that re-zeroes) or air-mouse mode (rotation speed). Emits relative X/Y; a `zmk,input-listener` makes mouse reports. |
| `src/imu_log.c` | Bring-up test: prints acceleration (mg) and rotation speed (mdps) over the USB serial log. |

Wiring and nodes belong to the shield (the korz config repo has them in
`boards/shields/korzdongle_addons/korzdongle_imumouse.overlay`).

Settings: `KORZ_IMU_MOUSE`, `KORZ_IMU_MODE_TILT` / `_GYRO`, `KORZ_IMU_SPEED`, `KORZ_IMU_TILT_RANGE`,
`KORZ_IMU_TILT_DEADZONE`, `KORZ_IMU_GYRO_GAIN`, `KORZ_IMU_GYRO_DEADZONE`, `KORZ_IMU_POLL_MS`,
`KORZ_IMU_DEBUG`; `KORZ_IMU` / `KORZ_IMU_PERIOD_MS` for the log test.

Planned here (not built yet): gesture engine (rotation and taps to ZMK behaviors, per layer), gyro
bias calibration, temperature channel, 3D-mouse layer.
