#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPS-IMU Sensor Fusion con Extended Kalman Filter
Invia pacchetti UDP con posizione fusa e dead reckoning quando GPS assente.

Formato pacchetto:
    MAC/±DD.dddddd7/±DDD.dddddd7/ss/q/vv.v/YYMMDDhhmmss/tms/ax/ay/az/mode
    mode: 0=GPS+IMU, 1=Dead Reckoning (solo IMU)
"""

import os
import argparse
import socket
import threading
import time
import datetime
import math

import serial
from serial.tools import list_ports
import pynmea2
import numpy as np

# Tentativo di importare librerie IMU
try:
    from adafruit_extended_bus import ExtendedI2C
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_LINEAR_ACCELERATION,
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE
    )
    IMU_AVAILABLE = True
    print("[IMU] Librerie BNO08x disponibili")
except ImportError:
    IMU_AVAILABLE = False
    print("[IMU] Librerie BNO08x NON disponibili - continuo senza fusione")

# ────────────────────────── CONFIGURAZIONE ──────────────────────────
CONFIG = {
    "gps_port_hint": "/dev/ttyACM0",
    "gps_baud": 115200,
    "destinations": [("193.70.113.55", 8888)],

    # GPIO per abilitazione GPS
    "gpio_enable_pin": 26,
    "gpio_active_high": True,
    "gpio_warmup_sec": 2,

    # IMU Configuration
    "imu_bus": 3,
    "imu_address": 0x4A,
    "imu_enabled": True,

    # Riconnessione seriale
    "serial_reopen_min": 1.0,
    "serial_reopen_max": 10.0,
    "serial_fast_retry_window_sec": 5.0,
    "serial_fast_retry_interval": 0.1,

    # Identificazione u-blox
    "ublox_vid_pid": {("1546", "01a8"), ("1546", "01a9")},
    "ublox_name_keywords": ("u-blox", "UBLOX", "GNSS"),
    
    # Kalman Filter parameters
    "gps_timeout_sec": 3.0,  # Dopo quanto tempo passare a dead reckoning
    "dt_imu": 0.01,  # 100 Hz IMU sampling
}

MAC_ADDR = "5"
print(f"[INFO] ID DEVICE: {MAC_ADDR}")

# ───────────── variabili globali condivise ─────────────
running = True
udp_socks = []

gps_lock = threading.Lock()
ser_lock = threading.Lock()
imu_lock = threading.Lock()
ekf_lock = threading.Lock()

gps_data = {
    'speed_kmh': 0.0,
    'timestamp': datetime.datetime.utcnow().strftime("%y%m%d%H%M%S"),
    'tms': 0,
    'latitude': None,
    'longitude': None,
    'satellites': 0,
    'quality': 0,
    'last_gps_time': None
}

imu_data = {
    'accel_x': 0.0,  # m/s² (world frame)
    'accel_y': 0.0,
    'accel_z': 0.0,
    'gyro_z': 0.0,   # rad/s
    'quat_i': 0.0,
    'quat_j': 0.0,
    'quat_k': 0.0,
    'quat_real': 1.0,
    'available': False
}

last_print_ts = 0.0

# ─────────────────────────── Extended Kalman Filter ───────────────────────────
class GPSIMUKF:
    """
    Extended Kalman Filter semplificato per fusione GPS-IMU (2D).
    Stato: [lat, lon, vx, vy, heading]
    """
    def __init__(self):
        # Stato: [lat, lon, vx, vy, heading]
        self.x = np.zeros(5)
        
        # Matrice di covarianza
        self.P = np.eye(5) * 1.0
        
        # Process noise
        self.Q = np.diag([1e-5, 1e-5, 0.1, 0.1, 0.01])
        
        # GPS measurement noise (più alto = meno fiducia in GPS)
        self.R_gps = np.diag([1e-6, 1e-6, 0.5, 0.5])  # [lat, lon, vx, vy]
        
        # Dead reckoning mode
        self.dr_mode = False
        
        self.last_update = time.time()
        
    def predict(self, ax, ay, gyro_z, dt):
        """Prediction step usando IMU"""
        lat, lon, vx, vy, heading = self.x
        
        # Converti accelerazioni da body frame a world frame usando heading
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        
        ax_world = ax * cos_h - ay * sin_h
        ay_world = ax * sin_h + ay * cos_h
        
        # Predict velocity
        vx_new = vx + ax_world * dt
        vy_new = vy + ay_world * dt
        
        # Predict position (approssimazione flat earth per brevi distanze)
        EARTH_RADIUS = 6371000.0  # metri
        dlat = (vy_new * dt) / EARTH_RADIUS
        dlon = (vx_new * dt) / (EARTH_RADIUS * np.cos(lat))
        
        lat_new = lat + dlat
        lon_new = lon + dlon
        
        # Predict heading
        heading_new = heading + gyro_z * dt
        heading_new = np.arctan2(np.sin(heading_new), np.cos(heading_new))  # normalize
        
        # Update state
        self.x = np.array([lat_new, lon_new, vx_new, vy_new, heading_new])
        
        # Jacobian della funzione di transizione
        F = np.eye(5)
        F[0, 3] = dt / EARTH_RADIUS
        F[1, 2] = dt / (EARTH_RADIUS * np.cos(lat))
        
        # Update covariance
        self.P = F @ self.P @ F.T + self.Q
        
    def update_gps(self, lat_gps, lon_gps, vx_gps, vy_gps):
        """Update step con misura GPS"""
        # Measurement matrix H (osserviamo lat, lon, vx, vy)
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0]
        ])
        
        # Innovation
        z = np.array([lat_gps, lon_gps, vx_gps, vy_gps])
        y = z - H @ self.x
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_gps
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        self.P = (np.eye(5) - K @ H) @ self.P
        
        self.dr_mode = False
        self.last_update = time.time()
        
    def get_position(self):
        """Ritorna posizione fusa e modalità"""
        return self.x[0], self.x[1], self.dr_mode
        
    def check_gps_timeout(self):
        """Passa a dead reckoning se GPS assente"""
        if time.time() - self.last_update > CONFIG["gps_timeout_sec"]:
            self.dr_mode = True

ekf = GPSIMUKF()

# ─────────────────────────── IMU Manager ───────────────────────────
class IMUManager:
    """Gestisce il sensore IMU BNO08x con più report"""
    def __init__(self):
        self.sensor = None
        self.i2c = None
        self.available = False
        self.stop_evt = threading.Event()
        
    def initialize(self):
        """Inizializza il sensore IMU"""
        if not IMU_AVAILABLE or not CONFIG.get("imu_enabled", True):
            print("[IMU] Disabilitato o librerie non disponibili")
            return False
            
        try:
            print(f"[IMU] Inizializzazione bus I2C {CONFIG['imu_bus']} @ 0x{CONFIG['imu_address']:02x}")
            self.i2c = ExtendedI2C(CONFIG['imu_bus'])
            time.sleep(0.1)
            
            self.sensor = BNO08X_I2C(self.i2c, address=CONFIG['imu_address'])
            time.sleep(0.2)
            
            # Abilita report necessari per fusion
            self.sensor.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            self.sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.sensor.enable_feature(BNO_REPORT_GYROSCOPE)
            time.sleep(0.1)
            
            self.available = True
            print("[IMU] ✓ Sensore BNO08x inizializzato con fusion reports")
            
            with imu_lock:
                imu_data['available'] = True
                
            return True
            
        except Exception as e:
            print(f"[IMU] ✗ Errore inizializzazione: {e}")
            self.available = False
            return False
    
    def quaternion_to_euler_z(self, qw, qx, qy, qz):
        """Estrae heading (yaw) da quaternione"""
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw
    
    def read_loop(self):
        """Loop di lettura IMU continuo"""
        if not self.available:
            return
            
        print("[IMU] Avvio loop di lettura con EKF...")
        
        while not self.stop_evt.is_set() and running:
            try:
                # Leggi accelerazione lineare
                lin_accel = self.sensor.linear_acceleration
                
                # Leggi quaternione (orientamento)
                quat = self.sensor.quaternion
                
                # Leggi gyro
                gyro = self.sensor.gyro
                
                if lin_accel and quat:
                    # Converti accelerazione da body frame a world frame
                    qw, qx, qy, qz = quat
                    ax_body, ay_body, az_body = lin_accel
                    
                    # Rotazione usando quaternione
                    # (semplificazione: usiamo solo heading per 2D)
                    heading = self.quaternion_to_euler_z(qw, qx, qy, qz)
                    
                    with imu_lock:
                        imu_data['accel_x'] = ax_body
                        imu_data['accel_y'] = ay_body
                        imu_data['accel_z'] = az_body
                        imu_data['quat_real'] = qw
                        imu_data['quat_i'] = qx
                        imu_data['quat_j'] = qy
                        imu_data['quat_k'] = qz
                        if gyro:
                            imu_data['gyro_z'] = gyro[2]
                        imu_data['available'] = True
                    
                    # EKF predict step
                    with ekf_lock:
                        gyro_z = gyro[2] if gyro else 0.0
                        ekf.predict(ax_body, ay_body, gyro_z, CONFIG["dt_imu"])
                        ekf.check_gps_timeout()
                
                time.sleep(CONFIG["dt_imu"])
                
            except Exception as e:
                print(f"[IMU] Errore lettura: {e}")
                time.sleep(0.1)
    
    def stop(self):
        """Ferma il loop e chiude le risorse"""
        self.stop_evt.set()
        try:
            if self.i2c:
                self.i2c.deinit()
        except Exception:
            pass
        print("[IMU] Chiuso")

imu_mgr = IMUManager()

# ─────────────────────────── UTIL ───────────────────────────
def init_udp():
    global udp_socks
    for s, *_ in udp_socks:
        try:
            s.close()
        except Exception:
            pass
    udp_socks.clear()
    for host, port in CONFIG["destinations"]:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socks.append((sock, host, port))
        print(f"[UDP] destinazione {host}:{port}")

def send_udp(msg: str):
    data = msg.encode()
    for sock, host, port in udp_socks:
        try:
            sock.sendto(data, (host, port))
        except OSError as e:
            print(f"[!] UDP error {host}:{port} – {e}")

def update_timestamp_from_msg(msg):
    current_dt = None
    tms = None
    if hasattr(msg, 'datestamp') and hasattr(msg, 'timestamp') and msg.datestamp and msg.timestamp:
        try:
            current_dt = datetime.datetime.combine(msg.datestamp, msg.timestamp)
            tms = getattr(msg.timestamp, 'microsecond', 0) // 1000
        except Exception:
            current_dt = None
    if not current_dt and hasattr(msg, 'timestamp') and msg.timestamp:
        try:
            utc_today = datetime.datetime.utcnow().date()
            current_dt = datetime.datetime.combine(utc_today, msg.timestamp)
            tms = getattr(msg.timestamp, 'microsecond', 0) // 1000
        except Exception:
            current_dt = None
    if not current_dt:
        now = datetime.datetime.utcnow()
        with gps_lock:
            gps_data['timestamp'] = now.strftime("%y%m%d%H%M%S")
            gps_data['tms'] = int(now.microsecond / 1000)
        return False
    with gps_lock:
        gps_data['timestamp'] = current_dt.strftime("%y%m%d%H%M%S")
        gps_data['tms'] = int(tms) if tms is not None else int((datetime.datetime.utcnow().microsecond) / 1000)
    return True

# ───────────────────── Serial Manager ───────────────────────
class SerialManager:
    """Gestisce la seriale GPS"""
    def __init__(self, baud):
        self.baud = baud
        self.ser = None
        self.stop_evt = threading.Event()
        self.reopen_delay = CONFIG["serial_reopen_min"]
        self.last_error_ts = None

    def _is_ublox(self, p):
        try:
            vid = (p.vid and f"{p.vid:04x}") or ""
            pid = (p.pid and f"{p.pid:04x}") or ""
            if (vid, pid) in CONFIG["ublox_vid_pid"]:
                return True
        except Exception:
            pass
        name = f"{p.manufacturer or ''} {p.product or ''} {p.description or ''}"
        for kw in CONFIG["ublox_name_keywords"]:
            if kw.lower() in name.lower():
                return True
        return False

    def _discover_port(self):
        hint = CONFIG.get("gps_port_hint")
        if hint:
            for p in list_ports.comports():
                if p.device == hint:
                    return hint
        candidates = []
        for p in list_ports.comports():
            if self._is_ublox(p):
                candidates.append(p.device)
        if candidates:
            candidates.sort()
            return candidates[0]
        for p in list_ports.comports():
            if p.device.startswith("/dev/ttyACM"):
                return p.device
        return None

    def open(self):
        while not self.stop_evt.is_set():
            port = self._discover_port()
            if not port:
                print("[SER] Nessuna porta u-blox trovata; ritento…")
                self._sleep_backoff()
                continue
            try:
                print(f"[SER] Apro {port} @ {self.baud}")
                self.ser = serial.Serial(port, self.baud, timeout=0.2, exclusive=True)
                self.reopen_delay = CONFIG["serial_reopen_min"]
                self.last_error_ts = None
                print("[SER] OK")
                return
            except serial.SerialException as e:
                print(f"[SER] {e}; ritento…")
                self._sleep_backoff()

    def _sleep_backoff(self):
        now = time.monotonic()
        if self.last_error_ts is None:
            self.last_error_ts = now
        elapsed = now - self.last_error_ts
        if elapsed < CONFIG["serial_fast_retry_window_sec"]:
            time.sleep(CONFIG["serial_fast_retry_interval"])
        else:
            time.sleep(self.reopen_delay)
            self.reopen_delay = min(CONFIG["serial_reopen_max"], self.reopen_delay * 1.5)

    def _mark_error(self):
        if self.last_error_ts is None:
            self.last_error_ts = time.monotonic()

    def readlines(self):
        while not self.stop_evt.is_set():
            if self.ser is None or not self.ser.is_open:
                self.open()
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                yield raw
            except (serial.SerialException, OSError) as e:
                print(f"[SER] read error: {e}; riapro…")
                self._mark_error()
                self._safe_close()
                self._sleep_backoff()

    def _safe_close(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def stop(self):
        self.stop_evt.set()
        self._safe_close()

serial_mgr = SerialManager(CONFIG["gps_baud"])

# ───────────────────────── THREAD - GPS ─────────────────────────────
def gps_worker():
    """Legge GPS e aggiorna EKF"""
    global last_print_ts
    print("[GPS] avvio lettura NMEA con EKF fusion...")
    
    for raw in serial_mgr.readlines():
        if not running:
            break
        try:
            line = raw.decode("ascii", errors="replace").strip()
        except UnicodeDecodeError:
            continue
        if not line.startswith('$'):
            continue
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            continue

        # VTG: velocità
        if isinstance(msg, pynmea2.VTG):
            with gps_lock:
                try:
                    if msg.spd_over_grnd_kmph is not None:
                        gps_data['speed_kmh'] = float(msg.spd_over_grnd_kmph)
                    elif msg.spd_over_grnd_kts is not None:
                        gps_data['speed_kmh'] = float(msg.spd_over_grnd_kts) * 1.852
                    else:
                        gps_data['speed_kmh'] = 0.0
                except (ValueError, TypeError):
                    gps_data['speed_kmh'] = 0.0

        # RMC: velocità + lat/lon + orario
        elif isinstance(msg, pynmea2.RMC):
            with gps_lock:
                try:
                    if msg.spd_over_grnd is not None:
                        gps_data['speed_kmh'] = float(msg.spd_over_grnd) * 1.852
                    else:
                        gps_data['speed_kmh'] = 0.0
                except (ValueError, TypeError):
                    gps_data['speed_kmh'] = 0.0
                if msg.status == 'A' and msg.latitude and msg.longitude:
                    gps_data['latitude'] = msg.latitude
                    gps_data['longitude'] = msg.longitude
                    gps_data['last_gps_time'] = time.time()
            update_timestamp_from_msg(msg)

        # GGA: fix, satelliti, qualità, update EKF
        elif isinstance(msg, pynmea2.GGA):
            should_send = False
            with gps_lock:
                try:
                    gps_data['satellites'] = int(msg.num_sats) if msg.num_sats else 0
                except (ValueError, TypeError):
                    gps_data['satellites'] = 0
                try:
                    gps_data['quality'] = int(msg.gps_qual) if msg.gps_qual else 0
                except (ValueError, TypeError):
                    gps_data['quality'] = 0

                if msg.gps_qual and int(msg.gps_qual) > 0 and msg.latitude and msg.longitude:
                    lat_gps = msg.latitude
                    lon_gps = msg.longitude
                    gps_data['latitude'] = lat_gps
                    gps_data['longitude'] = lon_gps
                    gps_data['last_gps_time'] = time.time()
                    
                    # Stima velocità in m/s (approssimazione)
                    speed_ms = gps_data['speed_kmh'] / 3.6
                    
                    # Update EKF con GPS
                    with ekf_lock:
                        # Inizializza EKF se prima volta
                        if ekf.x[0] == 0:
                            ekf.x[0] = np.radians(lat_gps)
                            ekf.x[1] = np.radians(lon_gps)
                        else:
                            ekf.update_gps(
                                np.radians(lat_gps),
                                np.radians(lon_gps),
                                speed_ms,  # vx (approssimazione)
                                0.0        # vy
                            )
                    
                    should_send = True

            update_timestamp_from_msg(msg)

            if should_send:
                # Ottieni posizione fusa da EKF
                with ekf_lock:
                    lat_fused, lon_fused, dr_mode = ekf.get_position()
                
                lat_fused_deg = np.degrees(lat_fused)
                lon_fused_deg = np.degrees(lon_fused)
                
                with gps_lock:
                    # Costruisci pacchetto con dati fusi
                    compact = (
                        f"{MAC_ADDR}/"
                        f"{lat_fused_deg:+09.7f}/"
                        f"{lon_fused_deg:+010.7f}/"
                        f"{gps_data['satellites']:02d}/"
                        f"{gps_data['quality']}/"
                        f"{gps_data['speed_kmh']:.1f}/"
                        f"{gps_data['timestamp']}/"
                        f"{int(gps_data['tms'])%1000:03d}"
                    )
                    
                    # Aggiungi dati IMU se disponibili
                    with imu_lock:
                        if imu_data['available']:
                            compact += (
                                f"/{imu_data['accel_x']:+.3f}"
                                f"/{imu_data['accel_y']:+.3f}"
                                f"/{imu_data['accel_z']:+.3f}"
                                f"/{int(dr_mode)}"  # 0=GPS+IMU, 1=Dead Reckoning
                            )
                    
                    compact += "\n"

                send_udp(compact)
                now = time.time()
                if now - last_print_ts >= 1.0:
                    mode_str = "DR" if dr_mode else "GPS+IMU"
                    print(f"[{mode_str}] {compact.strip()}")
                    last_print_ts = now

# ───────────────────────── THREAD - IMU ─────────────────────────────
def imu_worker():
    """Thread per gestire la lettura IMU"""
    if imu_mgr.initialize():
        imu_mgr.read_loop()
    else:
        print("[IMU] Thread non avviato - sensore non disponibile")

# ────────────── GPIO enable prima di GPS ───────────────
def enable_gpio_before_start():
    pin = CONFIG["gpio_enable_pin"]
    active_high = CONFIG["gpio_active_high"]
    warmup = CONFIG["gpio_warmup_sec"]
    try:
        import RPi.GPIO as GPIO
    except ImportError:
        print("[GPIO] Modulo RPi.GPIO non disponibile. Proseguo senza pilotare il pin.")
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH if active_high else GPIO.LOW)
    GPIO.output(pin, GPIO.HIGH if active_high else GPIO.LOW)
    print(f"[GPIO] Pin BCM {pin} impostato a {'HIGH' if active_high else 'LOW'}. Attesa {warmup}s…")
    time.sleep(warmup)

# ───────────────────────────── CLI / ENV ─────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="GPS-IMU Fusion con Extended Kalman Filter")
    i = p.add_mutually_exclusive_group()
    i.add_argument("--no-imu", action="store_true", help="Disabilita IMU")
    i.add_argument("--imu", action="store_true", help="Forza abilitazione IMU")
    return p.parse_args()

def resolve_imu_enabled(args) -> bool:
    if args.no_imu:
        return False
    if args.imu:
        return True
    env = os.getenv("IMU")
    if env is not None:
        return env not in ("0", "false", "False", "NO", "no")
    return CONFIG.get("imu_enabled", True)

# ───────────────────────────── MAIN ────────────────────────────────
if __name__ == "__main__":
    args = parse_args()
    enable_imu = resolve_imu_enabled(args) and IMU_AVAILABLE
    
    print(f"[CFG] GPS-IMU FUSION {'ABILITATO' if enable_imu else 'DISABILITATO'}")
    
    CONFIG["imu_enabled"] = enable_imu

    try:
        enable_gpio_before_start()
        init_udp()

        t_gps = threading.Thread(target=gps_worker, daemon=True)
        t_gps.start()

        if enable_imu:
            t_imu = threading.Thread(target=imu_worker, daemon=True)
            t_imu.start()

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[MAIN] interrompo…")
    finally:
        running = False
        try:
            serial_mgr.stop()
        except Exception:
            pass
        try:
            imu_mgr.stop()
        except Exception:
            pass
        for s, *_ in udp_socks:
            try:
                s.close()
            except Exception:
                pass
        try:
            import RPi.GPIO as GPIO
            GPIO.output(CONFIG["gpio_enable_pin"], GPIO.LOW if CONFIG["gpio_active_high"] else GPIO.HIGH)
            GPIO.cleanup()
        except Exception:
            pass
