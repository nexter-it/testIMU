#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Invia una singola riga compatta con i dati RTK/GNSS + IMU BNO085
su tutte le destinazioni UDP configurate e ne stampa
una al secondo sul terminale.

Formato pacchetto:
    MAC/±DD.dddddd7/±DDD.dddddd7/ss/q/vv.v/YYMMDDhhmmss/ax/ay/az/gx/gy/gz/mx/my/mz/qi/qj/qk/qr/roll/pitch/yaw
"""

import socket
import serial
import threading
import time
import base64
import datetime
import subprocess
import re
import math

import pynmea2

# Importa librerie IMU
try:
    from adafruit_extended_bus import ExtendedI2C
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_LINEAR_ACCELERATION,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_ROTATION_VECTOR,
    )
    IMU_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] Librerie IMU non disponibili: {e}")
    IMU_AVAILABLE = False

# ────────────────────────── CONFIGURAZIONE ──────────────────────────
CONFIG = {
    "gps_port":     "/dev/ttyACM0",
    "gps_baud":     115200,
    "destinations": [("193.70.113.55", 8888)],
    
    # Parametri NTRIP
    "ntrip_host": "213.209.192.165",
    "ntrip_port":   2101,
    "mount":        "NEXTER",
    "user":         "nexter",
    "password":     "nexter25",

    # GPIO da abilitare prima di aprire la seriale GPS / NTRIP
    "gpio_enable_pin": 26,     # BCM 26 (pin fisico 37)
    "gpio_active_high": True,  # True = porta alta, False = bassa
    "gpio_warmup_sec": 2,      # attesa prima di avviare GPS/NTRIP
    
    # IMU BNO085
    "imu_bus": 3,
    "imu_address": 0x4A,
    "imu_enabled": True,  # Metti False per disabilitare IMU
}
# --------------------------------------------------------------------

def get_wlan0_mac():
    """Ottiene il MAC address dell'interfaccia wlan0."""
    try:
        with open('/sys/class/net/wlan0/address', 'r') as f:
            mac = f.read().strip().replace(':', '').upper()
            return mac
    except FileNotFoundError:
        pass
    
    try:
        result = subprocess.run(['ip', 'link', 'show', 'wlan0'], 
                              capture_output=True, text=True, check=True)
        match = re.search(r'link/ether ([0-9a-f:]{17})', result.stdout)
        if match:
            mac = match.group(1).replace(':', '').upper()
            return mac
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    
    try:
        result = subprocess.run(['ifconfig', 'wlan0'], 
                              capture_output=True, text=True, check=True)
        match = re.search(r'ether ([0-9a-f:]{17})', result.stdout)
        if match:
            mac = match.group(1).replace(':', '').upper()
            return mac
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    
    import uuid
    print("[WARNING] wlan0 non trovata, uso MAC generico")
    return f"{uuid.getnode():012X}"

# MAC address dell'interfaccia wlan0
MAC_ADDR = get_wlan0_mac()
print(f"[INFO] MAC wlan0: {MAC_ADDR}")

# ───────────── variabili globali condivise fra i thread ─────────────
running          = True
udp_socks        = []

# Dati GPS più recenti
gps_data = {
    'speed_kmh': 0.0,
    'timestamp': datetime.datetime.utcnow().strftime("%y%m%d%H%M%S"),
    'latitude': None,
    'longitude': None,
    'satellites': 0,
    'quality': 0,
    'last_valid_time': None
}
gps_lock = threading.Lock()

# Dati IMU più recenti
imu_data = {
    'linear_accel': (0.0, 0.0, 0.0),
    'gyro': (0.0, 0.0, 0.0),
    'mag': (0.0, 0.0, 0.0),
    'quaternion': (0.0, 0.0, 0.0, 1.0),
    'euler': (0.0, 0.0, 0.0),  # roll, pitch, yaw
}
imu_lock = threading.Lock()

last_print_ts = 0.0    # per limitare la stampa a 1 Hz
# --------------------------------------------------------------------

def quaternion_to_euler(i, j, k, r):
    """Converte quaternioni in Eulero (gradi)"""
    roll = math.atan2(2*(r*i + j*k), 1 - 2*(i*i + j*j))
    sinp = 2*(r*j - k*i)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2*(r*k + i*j), 1 - 2*(j*j + k*k))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# ─────────────────────────── FUNZIONI UTILI ─────────────────────────
def init_udp():
    """Inizializza i socket UDP indicati in CONFIG."""
    global udp_socks
    for s, *_ in udp_socks:
        s.close()
    udp_socks.clear()
    for host, port in CONFIG["destinations"]:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socks.append((sock, host, port))
        print(f"[UDP] destinazione {host}:{port}")


def send_udp(msg: str):
    """Invia msg a tutte le destinazioni UDP configurate."""
    data = msg.encode()
    for sock, host, port in udp_socks:
        try:
            sock.sendto(data, (host, port))
        except OSError as e:
            print(f"[!] UDP error {host}:{port} – {e}")


def build_packet():
    """Costruisce il pacchetto UDP completo con GPS + IMU."""
    with gps_lock:
        gps_snap = gps_data.copy()
    
    with imu_lock:
        imu_snap = imu_data.copy()
    
    # Pacchetto base GPS (come prima)
    if gps_snap['latitude'] is None or gps_snap['longitude'] is None:
        return None
    
    packet = (
        f"{MAC_ADDR}/"
        f"{gps_snap['latitude']:+09.7f}/"
        f"{gps_snap['longitude']:+010.7f}/"
        f"{gps_snap['satellites']:02d}/"
        f"{gps_snap['quality']}/"
        f"{gps_snap['speed_kmh']:.1f}/"
        f"{gps_snap['timestamp']}"
    )
    
    # Aggiungi dati IMU se disponibili
    if CONFIG["imu_enabled"] and IMU_AVAILABLE:
        packet += (
            f"/{imu_snap['linear_accel'][0]:.3f}"
            f"/{imu_snap['linear_accel'][1]:.3f}"
            f"/{imu_snap['linear_accel'][2]:.3f}"
            f"/{imu_snap['gyro'][0]:.4f}"
            f"/{imu_snap['gyro'][1]:.4f}"
            f"/{imu_snap['gyro'][2]:.4f}"
            f"/{imu_snap['mag'][0]:.2f}"
            f"/{imu_snap['mag'][1]:.2f}"
            f"/{imu_snap['mag'][2]:.2f}"
            f"/{imu_snap['quaternion'][0]:.4f}"
            f"/{imu_snap['quaternion'][1]:.4f}"
            f"/{imu_snap['quaternion'][2]:.4f}"
            f"/{imu_snap['quaternion'][3]:.4f}"
            f"/{imu_snap['euler'][0]:.2f}"
            f"/{imu_snap['euler'][1]:.2f}"
            f"/{imu_snap['euler'][2]:.2f}"
        )
    
    return packet + "\n"


def update_timestamp_from_msg(msg):
    """Aggiorna il timestamp dai dati del messaggio NMEA."""
    current_time = None

    # Prova con datestamp + timestamp (RMC)
    if hasattr(msg, 'datestamp') and hasattr(msg, 'timestamp') and msg.datestamp and msg.timestamp:
        try:
            current_time = datetime.datetime.combine(msg.datestamp, msg.timestamp)
        except (AttributeError, ValueError, TypeError):
            pass

    # Infine prova solo con timestamp (GGA)
    if not current_time and hasattr(msg, 'timestamp') and msg.timestamp:
        try:
            today = datetime.date.today()
            current_time = datetime.datetime.combine(today, msg.timestamp)
        except (AttributeError, ValueError, TypeError):
            pass

    if current_time:
        with gps_lock:
            gps_data['timestamp'] = current_time.strftime("%y%m%d%H%M%S")
            gps_data['last_valid_time'] = current_time
        return True

    return False
# --------------------------------------------------------------------

# ───────────────────────── THREAD - IMU ─────────────────────────────
def imu_worker():
    """Legge continuamente dal BNO085 e aggiorna imu_data."""
    if not CONFIG["imu_enabled"]:
        print("[IMU] Disabilitato in configurazione")
        return
        
    if not IMU_AVAILABLE:
        print("[IMU] Librerie non disponibili, thread terminato")
        return
    
    print(f"[IMU] Inizializzo BNO085 su bus {CONFIG['imu_bus']}")
    
    try:
        i2c = ExtendedI2C(CONFIG["imu_bus"])
        time.sleep(0.1)
        sensor = BNO08X_I2C(i2c, address=CONFIG["imu_address"])
        time.sleep(0.2)
        
        sensor.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        sensor.enable_feature(BNO_REPORT_MAGNETOMETER)
        sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        print("[IMU] BNO085 pronto")
        
        while running:
            try:
                lin = sensor.linear_acceleration
                gyro = sensor.gyro
                mag = sensor.magnetic
                quat = sensor.quaternion
                
                if lin and gyro and mag and quat:
                    roll, pitch, yaw = quaternion_to_euler(*quat)
                    
                    with imu_lock:
                        imu_data['linear_accel'] = lin
                        imu_data['gyro'] = gyro
                        imu_data['mag'] = mag
                        imu_data['quaternion'] = quat
                        imu_data['euler'] = (roll, pitch, yaw)
                    
            except Exception as e:
                print(f"[IMU] Errore lettura: {e}")
                time.sleep(0.5)
                continue
            
            time.sleep(0.02)  # ~50Hz
            
    except Exception as e:
        print(f"[IMU] Errore init: {e}")
        return
    finally:
        try:
            i2c.deinit()
        except:
            pass

# ───────────────────────── THREAD - GPS ─────────────────────────────
def gps_worker():
    """Legge la seriale GPS, estrae dati da RMC/GGA/VTG e invia pacchetti."""
    global last_print_ts

    print(f"[GPS] seriale {CONFIG['gps_port']} @ {CONFIG['gps_baud']}")
    while running:
        try:
            with serial.Serial(CONFIG["gps_port"], CONFIG["gps_baud"], timeout=1) as ser:
                for raw in ser:
                    if not running:
                        break
                        
                    try:
                        line = raw.decode("ascii", errors="replace").strip()
                    except UnicodeDecodeError:
                        continue
                    if not line.startswith('$'):
                        continue

                    # prova a parsare la sentenza NMEA
                    try:
                        msg = pynmea2.parse(line)
                    except pynmea2.ParseError:
                        continue

                    # ------------------- VTG -------------------
                    if isinstance(msg, pynmea2.VTG):
                        with gps_lock:
                            try:
                                if msg.spd_over_grnd_kmph is not None:
                                    gps_data['speed_kmh'] = float(msg.spd_over_grnd_kmph)
                                elif msg.spd_over_grnd_kts is not None:
                                    speed_knots = float(msg.spd_over_grnd_kts)
                                    gps_data['speed_kmh'] = speed_knots * 1.852
                                else:
                                    gps_data['speed_kmh'] = 0.0
                            except (ValueError, TypeError):
                                gps_data['speed_kmh'] = 0.0

                    # ------------------- RMC -------------------
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

                    # ------------------- GGA -------------------
                    elif isinstance(msg, pynmea2.GGA):
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
                        
                        update_timestamp_from_msg(msg)
                        
                        # Costruisci e invia pacchetto completo GPS+IMU
                        packet = build_packet()
                        if packet:
                            send_udp(packet)

                            now = time.time()
                            if now - last_print_ts >= 1.0:
                                print(packet.strip())
                                last_print_ts = now

        except serial.SerialException as e:
            print(f"[GPS] {e}; ritento in 3 s")
            time.sleep(3)
# --------------------------------------------------------------------

# ─────────────────────── THREAD - NTRIP (opz.) ──────────────────────
def ntrip_worker():
    """Riceve correzioni RTCM dal caster NTRIP e le inoltra alla seriale GPS."""
    while running:
        try:
            creds = base64.b64encode(f"{CONFIG['user']}:{CONFIG['password']}".encode()).decode()
            req = (f"GET /{CONFIG['mount']} HTTP/1.0\r\n"
                   f"User-Agent: NTRIP python\r\n"
                   f"Authorization: Basic {creds}\r\n\r\n")

            with socket.create_connection((CONFIG["ntrip_host"], CONFIG["ntrip_port"]), 10) as s:
                s.sendall(req.encode())
                if b"ICY 200 OK" not in s.recv(1024):
                    print("[NTRIP] risposta non valida")
                    raise ConnectionError
                print("[NTRIP] connesso")

                with serial.Serial(CONFIG["gps_port"], CONFIG["gps_baud"], timeout=1) as ser:
                    while running:
                        data = s.recv(1024)
                        if not data:
                            raise ConnectionError("stream chiuso")
                        ser.write(data)

        except Exception as e:
            print(f"[NTRIP] {e}; riconnessione in 5 s")
            time.sleep(5)
# --------------------------------------------------------------------

# ────────────── GPIO: abilitazione prima di GPS/NTRIP ───────────────
def enable_gpio_before_start():
    """
    Imposta il GPIO indicato in CONFIG come uscita e lo porta
    al livello attivo (alto di default), poi attende warmup.
    """
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
    # Non facciamo cleanup qui per mantenere il livello durante l'esecuzione.
# --------------------------------------------------------------------

# ───────────────────────────── MAIN ────────────────────────────────
if __name__ == "__main__":
    print("=" * 70)
    print("Sistema GPS RTK + IMU BNO085 - Trasmissione UDP")
    print("=" * 70)
    
    # 1) Abilita GPIO e attende
    enable_gpio_before_start()

    # 2) Dopo l'attesa, inizializza UDP e avvia i thread
    init_udp()

    t_imu   = threading.Thread(target=imu_worker,   daemon=True)
    t_gps   = threading.Thread(target=gps_worker,   daemon=True)
    t_ntrip = threading.Thread(target=ntrip_worker, daemon=True)
    
    if CONFIG["imu_enabled"] and IMU_AVAILABLE:
        t_imu.start()
        print("[INFO] Thread IMU avviato")
    
    t_gps.start()
    t_ntrip.start()
    
    print("\n[MAIN] Sistema avviato. Premi Ctrl+C per terminare.\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        running = False
        print("\n[MAIN] interrompo…")
        for s, *_ in udp_socks:
            s.close()
        # porta il pin a livello inattivo e rilascia le risorse GPIO (best-effort)
        try:
            import RPi.GPIO as GPIO
            GPIO.output(CONFIG["gpio_enable_pin"], GPIO.LOW if CONFIG["gpio_active_high"] else GPIO.HIGH)
            GPIO.cleanup()
        except Exception:
            pass
