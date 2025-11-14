#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Invia una singola riga compatta con i dati RTK/GNSS
su tutte le destinazioni UDP configurate e ne stampa
una al secondo sul terminale.

Formato pacchetto (con IMU opzionale):
    MAC/±DD.dddddd7/±DDD.dddddd7/ss/q/vv.v/YYMMDDhhmmss/tms[/ax/ay]
    
Se IMU disponibile, aggiunge /ax/ay (accelerometro X e Y in g)
"""

import os
import argparse
import socket
import threading
import time
import base64
import datetime
import subprocess
import re
import sys

import serial
from serial.tools import list_ports
import pynmea2

# Tentativo di importare librerie IMU
try:
    from adafruit_extended_bus import ExtendedI2C
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import BNO_REPORT_LINEAR_ACCELERATION
    IMU_AVAILABLE = True
    print("[IMU] Librerie BNO08x disponibili")
except ImportError:
    IMU_AVAILABLE = False
    print("[IMU] Librerie BNO08x NON disponibili - continuo senza IMU")

# ────────────────────────── CONFIGURAZIONE ──────────────────────────
CONFIG = {
    # Usa, se puoi, il path stabile /dev/serial/by-id/... come hint:
    "gps_port_hint": "/dev/ttyACM0",
    "gps_baud": 115200,

    "destinations": [("193.70.113.55", 8888)],

    # Abilita/disabilita NTRIP (può essere sovrascritto da CLI/env)
    "enable_ntrip": True,

    # Parametri NTRIP (opzionale):
    "ntrip_host": "213.209.192.165",
    "ntrip_port": 2101,
    "mount": "NEXTER",
    "user": "nexter",
    "password": "nexter25",

    # GPIO da abilitare prima di aprire la seriale GPS / NTRIP
    "gpio_enable_pin": 26,     # BCM 26 (pin fisico 37)
    "gpio_active_high": True,  # True = porta alta, False = bassa
    "gpio_warmup_sec": 2,      # attesa prima di avviare GPS/NTRIP

    # IMU Configuration (opzionale)
    "imu_bus": 3,              # I2C bus number
    "imu_address": 0x4A,       # BNO08x default address
    "imu_enabled": True,       # Abilita IMU se disponibile

    # Riconnessione seriale
    "serial_reopen_min": 1.0,
    "serial_reopen_max": 10.0,

    # Nuovo: finestra di retry rapidi (per ridurre la latenza di ripartenza)
    "serial_fast_retry_window_sec": 5.0,   # per i primi 5 s dopo un errore
    "serial_fast_retry_interval": 0.1,     # prova ogni 100 ms nella finestra

    # Identificazione u-blox (VID/PID classici ZED-F9P CDC-ACM; cambiali se diverso sul tuo dmesg/lsusb)
    "ublox_vid_pid": {("1546", "01a8"), ("1546", "01a9")},  # alcune varianti
    "ublox_name_keywords": ("u-blox", "UBLOX", "GNSS"),
}

MAC_ADDR = "5"
print(f"[INFO] ID DEVICE: {MAC_ADDR}")

# ───────────── variabili globali condivise fra i thread ─────────────
running = True
udp_socks = []

gps_lock = threading.Lock()
ser_lock = threading.Lock()  # per scritture RTCM mentre si legge
imu_lock = threading.Lock()  # per dati IMU

gps_data = {
    'speed_kmh': 0.0,
    'timestamp': datetime.datetime.utcnow().strftime("%y%m%d%H%M%S"),
    'tms': 0,
    'latitude': None,
    'longitude': None,
    'satellites': 0,
    'quality': 0,
    'last_valid_time': None
}

imu_data = {
    'accel_x': 0.0,  # g
    'accel_y': 0.0,  # g
    'available': False
}

last_print_ts = 0.0

# ─────────────────────────── IMU Manager ───────────────────────────
class IMUManager:
    """Gestisce il sensore IMU BNO08x opzionale"""
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
            
            # Abilita solo accelerazione lineare (necessaria per accel X/Y)
            self.sensor.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            time.sleep(0.1)
            
            self.available = True
            print("[IMU] ✓ Sensore BNO08x inizializzato")
            
            with imu_lock:
                imu_data['available'] = True
                
            return True
            
        except Exception as e:
            print(f"[IMU] ✗ Errore inizializzazione: {e}")
            self.available = False
            return False
    
    def read_loop(self):
        """Loop di lettura IMU continuo"""
        if not self.available:
            return
            
        print("[IMU] Avvio loop di lettura...")
        
        while not self.stop_evt.is_set() and running:
            try:
                # Leggi accelerazione lineare
                lin_accel = self.sensor.linear_acceleration
                
                if lin_accel:
                    # Converti in g (dividi per 9.81 m/s²)
                    accel_x_g = lin_accel[0] / 9.81
                    accel_y_g = lin_accel[1] / 9.81
                    
                    with imu_lock:
                        imu_data['accel_x'] = accel_x_g
                        imu_data['accel_y'] = accel_y_g
                        imu_data['available'] = True
                
                time.sleep(0.01)  # 100 Hz sampling
                
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
            gps_data['last_valid_time'] = now
        return False
    with gps_lock:
        gps_data['timestamp'] = current_dt.strftime("%y%m%d%H%M%S")
        gps_data['tms'] = int(tms) if tms is not None else int((datetime.datetime.utcnow().microsecond) / 1000)
        gps_data['last_valid_time'] = current_dt
    return True

# ───────────────────── Serial Manager ───────────────────────
class SerialManager:
    """
    Mantiene UNA sola seriale aperta verso il F9P.
    Se cade, prova a riaprirla cercando la porta u-blox (VID/PID o nome).
    Retry molto rapidi nella prima finestra per minimizzare la latenza.
    """
    def __init__(self, baud):
        self.baud = baud
        self.ser = None
        self.stop_evt = threading.Event()
        self.reopen_delay = CONFIG["serial_reopen_min"]
        self.last_error_ts = None  # per la finestra di retry veloci

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
        # 1) Se hint esiste, provalo prima (meglio se è /dev/serial/by-id/…)
        hint = CONFIG.get("gps_port_hint")
        if hint:
            for p in list_ports.comports():
                if p.device == hint:
                    return hint
        # 2) Cerca per u-blox
        candidates = []
        for p in list_ports.comports():
            if self._is_ublox(p):
                candidates.append(p.device)
        if candidates:
            candidates.sort()
            return candidates[0]
        # 3) fallback: prima ttyACM disponibile
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
                self.ser = serial.Serial(
                    port,
                    self.baud,
                    timeout=0.2,  # ↓ timeout ridotto per reazione più rapida
                    exclusive=True
                )
                self.reopen_delay = CONFIG["serial_reopen_min"]
                self.last_error_ts = None
                print("[SER] OK")
                return
            except serial.SerialException as e:
                print(f"[SER] {e}; ritento…")
                self._sleep_backoff()

    def _sleep_backoff(self):
        # Retry rapidi per i primi N secondi dopo l'errore,
        # poi passa all'exponential backoff.
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
        """
        Generatore di righe NMEA (bytes). Se cade, tenta riapertura.
        """
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

    def write(self, data: bytes):
        """
        Scrittura thread-safe (RTCM) con lock; se fallisce tenta riapertura.
        """
        while not self.stop_evt.is_set():
            if self.ser is None or not self.ser.is_open:
                self.open()
            try:
                with ser_lock:
                    self.ser.write(data)
                return
            except (serial.SerialException, OSError) as e:
                print(f"[SER] write error: {e}; riapro…")
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
    """Legge la seriale (unico handle) e invia pacchetti su GGA; aggiorna da VTG/RMC."""
    global last_print_ts
    print("[GPS] avvio lettura NMEA…")
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
            update_timestamp_from_msg(msg)

        # GGA: fix, satelliti, qualità, invio pacchetto
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
                    gps_data['latitude'] = msg.latitude
                    gps_data['longitude'] = msg.longitude
                    should_send = True

                if should_send and gps_data['latitude'] is not None and gps_data['longitude'] is not None:
                    # Costruisci pacchetto base
                    compact = (
                        f"{MAC_ADDR}/"
                        f"{gps_data['latitude']:+09.7f}/"
                        f"{gps_data['longitude']:+010.7f}/"
                        f"{gps_data['satellites']:02d}/"
                        f"{gps_data['quality']}/"
                        f"{gps_data['speed_kmh']:.1f}/"
                        f"{gps_data['timestamp']}/"
                        f"{int(gps_data['tms'])%1000:03d}"
                    )
                    
                    # Aggiungi dati IMU se disponibili
                    with imu_lock:
                        if imu_data['available']:
                            compact += f"/{imu_data['accel_x']:+.3f}/{imu_data['accel_y']:+.3f}"
                    
                    compact += "\n"

            update_timestamp_from_msg(msg)

            if should_send:
                send_udp(compact)
                now = time.time()
                if now - last_print_ts >= 1.0:
                    print(compact.strip())
                    last_print_ts = now

# ─────────────────────── THREAD - NTRIP (opz.) ──────────────────────
def ntrip_worker():
    """Riceve correzioni RTCM dal caster NTRIP e le inoltra alla seriale (unico handle)."""
    while running:
        try:
            creds = base64.b64encode(f"{CONFIG['user']}:{CONFIG['password']}".encode()).decode()
            req = (f"GET /{CONFIG['mount']} HTTP/1.0\r\n"
                   f"User-Agent: NTRIP python\r\n"
                   f"Authorization: Basic {creds}\r\n\r\n")

            with socket.create_connection((CONFIG["ntrip_host"], CONFIG["ntrip_port"]), 10) as s:
                s.sendall(req.encode())
                header = s.recv(1024)
                if b"ICY 200 OK" not in header:
                    print("[NTRIP] risposta non valida")
                    raise ConnectionError("bad header")
                print("[NTRIP] connesso")
                while running:
                    data = s.recv(1024)
                    if not data:
                        raise ConnectionError("stream chiuso")
                    serial_mgr.write(data)

        except Exception as e:
            print(f"[NTRIP] {e}; riconnessione in 5 s")
            time.sleep(5)

# ───────────────────── THREAD - IMU ─────────────────────────────────
def imu_worker():
    """Thread per gestire la lettura IMU"""
    if imu_mgr.initialize():
        imu_mgr.read_loop()
    else:
        print("[IMU] Thread non avviato - sensore non disponibile")

# ────────────── GPIO enable prima di GPS/NTRIP ───────────────
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
    p = argparse.ArgumentParser(description="GNSS RTK sender con IMU opzionale")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--no-ntrip", action="store_true", help="Disabilita NTRIP")
    g.add_argument("--ntrip", action="store_true", help="Forza abilitazione NTRIP")
    
    i = p.add_mutually_exclusive_group()
    i.add_argument("--no-imu", action="store_true", help="Disabilita IMU")
    i.add_argument("--imu", action="store_true", help="Forza abilitazione IMU")
    
    return p.parse_args()

def resolve_ntrip_enabled(args) -> bool:
    # Ordine di priorità: CLI > ENV > CONFIG
    if args.no_ntrip:
        return False
    if args.ntrip:
        return True
    env = os.getenv("NTRIP")
    if env is not None:
        return env not in ("0", "false", "False", "NO", "no")
    return CONFIG.get("enable_ntrip", True)

def resolve_imu_enabled(args) -> bool:
    # Ordine di priorità: CLI > ENV > CONFIG
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
    enable_ntrip = resolve_ntrip_enabled(args)
    enable_imu = resolve_imu_enabled(args) and IMU_AVAILABLE
    
    print(f"[CFG] NTRIP {'ABILITATO' if enable_ntrip else 'DISABILITATO'}")
    print(f"[CFG] IMU {'ABILITATO' if enable_imu else 'DISABILITATO'}")
    
    # Aggiorna configurazione
    CONFIG["imu_enabled"] = enable_imu

    try:
        enable_gpio_before_start()
        init_udp()

        t_gps = threading.Thread(target=gps_worker, daemon=True)
        t_gps.start()

        if enable_ntrip:
            t_ntrip = threading.Thread(target=ntrip_worker, daemon=True)
            t_ntrip.start()

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
