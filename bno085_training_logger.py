#!/usr/bin/env python3
# BNO085 Live Monitor - Output semplice riga-per-riga (CSV)

import time
import sys
import math
from adafruit_extended_bus import ExtendedI2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)

def quaternion_to_euler(i, j, k, r):
    roll = math.atan2(2*(r*i + j*k), 1 - 2*(i*i + j*j))
    sinp = 2*(r*j - k*i)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2*(r*k + i*j), 1 - 2*(j*j + k*k))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class BNO085Simple:
    def __init__(self, bus=3, address=0x4A, hz=5):
        self.bus = bus
        self.address = address
        self.hz = hz
        self.i2c = None
        self.sensor = None
        self.start_time = None

    def initialize(self):
        try:
            self.i2c = ExtendedI2C(self.bus)
            time.sleep(0.1)
            self.sensor = BNO08X_I2C(self.i2c, address=self.address)
            time.sleep(0.3)

            # Abilita solo ciò che stampiamo
            for f in (
                BNO_REPORT_LINEAR_ACCELERATION,
                BNO_REPORT_GYROSCOPE,
                BNO_REPORT_MAGNETOMETER,
                BNO_REPORT_ROTATION_VECTOR,
            ):
                self.sensor.enable_feature(f)
                time.sleep(0.05)

            self.start_time = time.time()
            return True
        except Exception as e:
            print(f"init_error,{e}", file=sys.stderr)
            return False

    def read_once(self):
        lin = self.sensor.linear_acceleration  # m/s^2
        gyro = self.sensor.gyro                # rad/s
        mag = self.sensor.magnetic             # uT
        quat = self.sensor.quaternion          # i,j,k,r
        roll, pitch, yaw = quaternion_to_euler(*quat)
        t = time.time() - self.start_time
        return (
            t,
            lin[0], lin[1], lin[2],
            gyro[0], gyro[1], gyro[2],
            mag[0], mag[1], mag[2],
            roll, pitch, yaw
        )

    def cleanup(self):
        try:
            if self.i2c:
                self.i2c.deinit()
        except:
            pass

def main():
    mon = BNO085Simple(bus=3, address=0x4A, hz=5)
    if not mon.initialize():
        sys.exit(1)

    # Intestazione CSV (una volta sola)
    print("t_s,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,roll_deg,pitch_deg,yaw_deg")
    interval = 1.0 / mon.hz

    try:
        next_t = time.time()
        while True:
            data = mon.read_once()
            # Riga CSV
            print(
                "{:.3f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}"
                .format(*data),
                flush=True
            )
            next_t += interval
            # sleep preciso senza drift grossolano
            now = time.time()
            if next_t > now:
                time.sleep(next_t - now)
    except KeyboardInterrupt:
        pass
    finally:
        mon.cleanup()

if __name__ == "__main__":
    main()
