#!/usr/bin/env python3
# BNO085 Live Monitor con grafico velocità in tempo reale

import time
import sys
import math
import signal
from collections import deque

import matplotlib
matplotlib.use("TkAgg")  # prova a forzare un backend interattivo comune
import matplotlib.pyplot as plt

from adafruit_extended_bus import ExtendedI2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

# ---------- Utility ----------
def quaternion_to_euler(i, j, k, r):
    """Converte quaternioni in Eulero (gradi)"""
    roll = math.atan2(2*(r*i + j*k), 1 - 2*(i*i + j*j))
    sinp = 2*(r*j - k*i)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2*(r*k + i*j), 1 - 2*(j*j + k*k))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# ---------- Classe Monitor ----------
class BNO085MonitorSimple:
    def __init__(self, bus=3, address=0x4A, window_seconds=20.0, sample_period=0.5):
        self.bus = bus
        self.address = address
        self.sensor = None
        self.i2c = None
        self.start_time = None
        self.last_time = None
        self.velocity = [0.0, 0.0, 0.0]  # vx, vy, vz
        self.sample_period = sample_period

        # buffer per grafico a finestra scorrevole
        self.window_seconds = window_seconds
        self.t_buf = deque()
        self.vx_buf = deque()
        self.vy_buf = deque()
        self.vz_buf = deque()
        self.s_buf  = deque()  # |v|

        # filtro high-pass leggero (per ridurre deriva da bias di accel)
        self.accel_bias = [0.0, 0.0, 0.0]
        self.bias_alpha = 0.001  # più alto = converge più in fretta, ma rischia di togliere vero segnale

        # grafico
        self.fig = None
        self.ax = None
        self.line_speed = None
        self.line_vx = None
        self.line_vy = None
        self.line_vz = None
        self.key_reset_requested = False

    def initialize(self):
        """Inizializza sensore e grafico"""
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
                time.sleep(0.05)

            print("Sensore pronto!\n")

            self.start_time = time.time()
            self.last_time = self.start_time

            # setup grafico
            plt.ion()
            self.fig, self.ax = plt.subplots()
            self.fig.canvas.mpl_connect("key_press_event", self._on_key)
            self.ax.set_title("Velocità stimata (BNO085)")
            self.ax.set_xlabel("Tempo (s)")
            self.ax.set_ylabel("Velocità (m/s)")

            # linee (senza specificare colori: usa la palette di default)
            (self.line_speed,) = self.ax.plot([], [], label="|v|")
            (self.line_vx,) = self.ax.plot([], [], label="Vx")
            (self.line_vy,) = self.ax.plot([], [], label="Vy")
            (self.line_vz,) = self.ax.plot([], [], label="Vz")
            self.ax.legend(loc="upper left")

            self.ax.grid(True)
            self.fig.tight_layout()

            return True
        except Exception as e:
            print(f"Errore inizializzazione: {e}")
            return False

    def _on_key(self, event):
        # premi 'r' per resettare velocità (utile contro la deriva)
        if event.key and event.key.lower() == "r":
            self.key_reset_requested = True

    def _apply_highpass_and_integrate(self, accel, dt):
        # aggiorna stima bias lenta e sottrae (filtro high-pass semplice)
        for i in range(3):
            self.accel_bias[i] = (1 - self.bias_alpha) * self.accel_bias[i] + self.bias_alpha * accel[i]
            a_hp = accel[i] - self.accel_bias[i]
            self.velocity[i] += a_hp * dt

    def estimate_velocity(self, accel):
        """Stima velocità con integrazione e high-pass semplice"""
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            dt = 1e-3
        self._apply_highpass_and_integrate(accel, dt)
        self.last_time = now
        speed = math.sqrt(sum(v**2 for v in self.velocity))
        return self.velocity[:], speed, now - self.start_time

    def _append_to_buffers(self, t, vx, vy, vz, s):
        self.t_buf.append(t)
        self.vx_buf.append(vx)
        self.vy_buf.append(vy)
        self.vz_buf.append(vz)
        self.s_buf.append(s)

        # taglia finestra scorrevole
        tmin = t - self.window_seconds
        while self.t_buf and self.t_buf[0] < tmin:
            self.t_buf.popleft()
            self.vx_buf.popleft()
            self.vy_buf.popleft()
            self.vz_buf.popleft()
            self.s_buf.popleft()

    def _update_plot(self):
        if not self.t_buf:
            return

        t0 = self.t_buf[0]
        ts = [ti - t0 for ti in self.t_buf]  # porta l'asse tempo a partire da 0 nella finestra

        self.line_speed.set_data(ts, list(self.s_buf))
        self.line_vx.set_data(ts, list(self.vx_buf))
        self.line_vy.set_data(ts, list(self.vy_buf))
        self.line_vz.set_data(ts, list(self.vz_buf))

        # aggiorna assi
        self.ax.set_xlim(0, max(1.0, ts[-1]))
        # autoscale y in base ai dati in finestra con margine
        vals = list(self.s_buf) + list(self.vx_buf) + list(self.vy_buf) + list(self.vz_buf)
        vmin = min(vals)
        vmax = max(vals)
        if vmin == vmax:
            vmin -= 0.5
            vmax += 0.5
        pad = 0.1 * (vmax - vmin)
        self.ax.set_ylim(vmin - pad, vmax + pad)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def read_and_step(self):
        """Legge, aggiorna velocità, stampa e aggiorna grafico"""
        try:
            lin = self.sensor.linear_acceleration  # m/s^2, gravity-free
            _ = self.sensor.gyro
            _ = self.sensor.magnetic
            quat = self.sensor.quaternion
            roll, pitch, yaw = quaternion_to_euler(*quat)

            if self.key_reset_requested:
                self.velocity = [0.0, 0.0, 0.0]
                self.key_reset_requested = False
                print("↻ Velocità azzerata (tasto 'r').")

            (vx, vy, vz), speed, elapsed = self.estimate_velocity(lin)

            # stampa compatta
            print(f"t={elapsed:6.1f}s  v=({vx:7.3f},{vy:7.3f},{vz:7.3f})  |v|={speed:7.3f} m/s   yaw/pitch/roll=({yaw:6.1f},{pitch:6.1f},{roll:6.1f})")
            if elapsed > 5:
                print("  ⚠️  Possibile deriva in aumento")
            if elapsed > 30:
                print("  🔴 Dati velocità meno affidabili: considera un reset ('r') o un riferimento esterno (GPS).")

            # aggiorna buffer e grafico
            self._append_to_buffers(elapsed, vx, vy, vz, speed)
            self._update_plot()

        except Exception as e:
            print(f"Errore lettura: {e}")
            time.sleep(0.5)

    def cleanup(self):
        try:
            if self.i2c:
                self.i2c.deinit()
        except:
            pass

# ---------- Main ----------
def main():
    mon = BNO085MonitorSimple(bus=3, address=0x4A, window_seconds=20.0, sample_period=0.5)
    if not mon.initialize():
        sys.exit(1)

    print("Premi Ctrl+C per uscire. Nella finestra del grafico premi 'r' per azzerare la velocità.\n")

    # chiusura pulita con SIGINT
    def handle_sigint(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, handle_sigint)

    try:
        while True:
            start = time.time()
            mon.read_and_step()
            # mantieni frequenza target (~2 Hz di default)
            dt = time.time() - start
            to_sleep = max(0.0, mon.sample_period - dt)
            time.sleep(to_sleep)
    except KeyboardInterrupt:
        print("\nTerminato.")
    finally:
        mon.cleanup()
        # lascia la finestra grafico aperta finché non viene chiusa
        if mon.fig:
            print("Chiudi la finestra del grafico per uscire definitivamente.")
            plt.ioff()
            try:
                plt.show()
            except Exception:
                pass

if __name__ == "__main__":
    main()
