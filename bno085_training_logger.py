#!/usr/bin/env python3
# BNO085 Live Monitor - Versione con salvataggio su CSV

import time
import sys
import math
import csv
import os
from adafruit_extended_bus import ExtendedI2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

def quaternion_to_euler(i, j, k, r):
    """Converte quaternioni in Eulero (gradi)"""
    roll = math.atan2(2*(r*i + j*k), 1 - 2*(i*i + j*j))
    sinp = 2*(r*j - k*i)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2*(r*k + i*j), 1 - 2*(j*j + k*k))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class BNO085MonitorSimple:
    def __init__(self, bus=3, address=0x4A, log_filename=None):
        self.bus = bus
        self.address = address
        self.sensor = None
        self.i2c = None
        self.start_time = None
        self.last_time = None
        self.velocity = [0.0, 0.0, 0.0]  # vx, vy, vz

        # logging
        self.log_filename = log_filename or "bno085_data.csv"
        self.log_file = None
        self.csv_writer = None

    def initialize(self):
        """Inizializza il sensore"""
        try:
            print(f"Inizializzo BNO085 su bus {self.bus} ...")
            self.i2c = ExtendedI2C(self.bus)
            time.sleep(0.1)
            self.sensor = BNO08X_I2C(self.i2c, address=self.address)
            time.sleep(0.2)

            for f, name in [
                (BNO_REPORT_LINEAR_ACCELERATION, "Accelerazione"),
                (BNO_REPORT_GYROSCOPE, "Giroscopio"),
                (BNO_REPORT_MAGNETOMETER, "Magnetometro"),
                (BNO_REPORT_ROTATION_VECTOR, "Orientamento"),
            ]:
                self.sensor.enable_feature(f)
                print(f"  ✓ {name} abilitato")
                time.sleep(0.1)

            print("Sensore pronto!\n")
            self.start_time = time.time()
            self.last_time = self.start_time
            return True
        except Exception as e:
            print(f"Errore inizializzazione: {e}")
            return False

    def start_logging(self):
        """Apre (o crea) un file CSV per salvare i dati"""
        try:
            filename = self.log_filename
            new_file = not os.path.exists(filename)
            self.log_file = open(filename, "a", newline="")
            self.csv_writer = csv.writer(self.log_file)
            if new_file:
                # intestazione del file
                self.csv_writer.writerow([
                    "time_s", "ax", "ay", "az",
                    "gx", "gy", "gz",
                    "mx", "my", "mz",
                    "yaw_deg", "pitch_deg", "roll_deg",
                    "vx", "vy", "vz", "speed"
                ])
            print(f"📁 Log attivo su: {filename}")
        except Exception as e:
            print(f"⚠️  Impossibile aprire il file di log: {e}")
            self.log_file = None
            self.csv_writer = None

    def log_data(self, elapsed, lin, gyro, mag, roll, pitch, yaw, velocity, speed):
        """Scrive una riga di dati nel CSV"""
        if not self.csv_writer:
            return
        try:
            self.csv_writer.writerow([
                round(elapsed, 3),
                *[round(x, 4) for x in lin],
                *[round(x, 4) for x in gyro],
                *[round(x, 2) for x in mag],
                round(yaw, 2), round(pitch, 2), round(roll, 2),
                *[round(v, 4) for v in velocity],
                round(speed, 4),
            ])
            self.log_file.flush()  # scrivi subito su disco
        except Exception as e:
            print(f"⚠️  Errore scrittura CSV: {e}")

    def estimate_velocity(self, accel):
        """Stima velocità con integrazione semplice"""
        now = time.time()
        dt = now - self.last_time
        for i in range(3):
            self.velocity[i] += accel[i] * dt
        self.last_time = now
        speed = math.sqrt(sum(v**2 for v in self.velocity))
        return self.velocity, speed

    def read_and_print(self):
        """Legge e stampa i dati"""
        try:
            lin = self.sensor.linear_acceleration
            gyro = self.sensor.gyro
            mag = self.sensor.magnetic
            quat = self.sensor.quaternion
            roll, pitch, yaw = quaternion_to_euler(*quat)

            velocity, speed = self.estimate_velocity(lin)
            elapsed = time.time() - self.start_time

            print(f"Tempo: {elapsed:6.1f} s")
            print(f"  Accel (m/s²):   X={lin[0]:7.3f}  Y={lin[1]:7.3f}  Z={lin[2]:7.3f}")
            print(f"  Gyro  (rad/s):  X={gyro[0]:7.3f}  Y={gyro[1]:7.3f}  Z={gyro[2]:7.3f}")
            print(f"  Mag   (µT):     X={mag[0]:7.2f}  Y={mag[1]:7.2f}  Z={mag[2]:7.2f}")
            print(f"  Yaw/Pitch/Roll: {yaw:7.2f}°  {pitch:7.2f}°  {roll:7.2f}°")
            print(f"  Velocità stimata: Vx={velocity[0]:7.3f}  Vy={velocity[1]:7.3f}  Vz={velocity[2]:7.3f}  |v|={speed:7.3f} m/s")
            if elapsed > 5:
                print(f"  ⚠️  Deriva in aumento (t={elapsed:.0f}s)")
            if elapsed > 30:
                print("  🔴 Dati velocità non più affidabili! Usa GPS.")
            print("-" * 60)
            sys.stdout.flush()

            # Salvataggio su file
            self.log_data(elapsed, lin, gyro, mag, roll, pitch, yaw, velocity, speed)

        except Exception as e:
            print(f"Errore lettura: {e}")
            time.sleep(0.5)

    def cleanup(self):
        try:
            if self.log_file:
                self.log_file.close()
        except:
            pass
        try:
            if self.i2c:
                self.i2c.deinit()
        except:
            pass

def main():
    mon = BNO085MonitorSimple(bus=3, address=0x4A, log_filename="bno085_data.csv")
    if not mon.initialize():
        sys.exit(1)

    # Avvia logging su CSV
    mon.start_logging()

    print("Premi Ctrl+C per uscire.\n")

    try:
        while True:
            mon.read_and_print()
            time.sleep(0.5)  # aggiorna 2 volte al secondo
    except KeyboardInterrupt:
        print("\nTerminato.")
    finally:
        mon.cleanup()

if __name__ == "__main__":
    main()
