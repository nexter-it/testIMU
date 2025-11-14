#!/usr/bin/env python3
"""
Telemetria Kart - Sender UDP per Dashboard Remota
Invia dati telemetria a server via rete
VERSIONE SENSIBILE - Soglie ottimizzate per rilevamento precoce
"""

import time
import sys
import math
import json
import socket
import threading
import serial
from collections import deque
from datetime import datetime
from adafruit_extended_bus import ExtendedI2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER,
)

try:
    import pynmea2
    PYNMEA2_AVAILABLE = True
except ImportError:
    PYNMEA2_AVAILABLE = False

# GPIO Configuration
GPIO_PIN = 26
GPIO_ACTIVE_HIGH = True
GPIO_WARMUP_SEC = 2

# GPS Configuration
GPS_PORT = "/dev/ttyACM0"
GPS_BAUD = 115200


def quaternion_to_euler(i, j, k, r):
    """Converte quaternioni in angoli Eulero"""
    roll = math.atan2(2*(r*i + j*k), 1 - 2*(i*i + j*j))
    sinp = 2*(r*j - k*i)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2*(r*k + i*j), 1 - 2*(j*j + k*k))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def calcola_heading(mag_x, mag_y):
    """Calcola heading da magnetometro"""
    heading = math.atan2(mag_y, mag_x)
    heading = math.degrees(heading)
    if heading < 0:
        heading += 360
    return heading


def enable_gpio():
    """Abilita il GPIO pin per il modem GNSS"""
    try:
        import RPi.GPIO as GPIO
    except ImportError:
        print("[GPIO] RPi.GPIO non disponibile, proseguo senza GPIO")
        return
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(GPIO_PIN, GPIO.OUT, initial=GPIO.HIGH if GPIO_ACTIVE_HIGH else GPIO.LOW)
        GPIO.output(GPIO_PIN, GPIO.HIGH if GPIO_ACTIVE_HIGH else GPIO.LOW)
        print(f"[GPIO] Pin BCM {GPIO_PIN} impostato a {'HIGH' if GPIO_ACTIVE_HIGH else 'LOW'}")
        print(f"[GPIO] Warmup {GPIO_WARMUP_SEC}s...\n")
        time.sleep(GPIO_WARMUP_SEC)
    except Exception as e:
        print(f"[GPIO] Errore: {e}")


class TelemetriaKartSender:
    def __init__(self, bus=3, indirizzo=0x4A, server_ip="193.70.113.55", server_port=5005, save_data=True):
        self.bus = bus
        self.indirizzo = indirizzo
        self.sensore = None
        self.i2c = None
        
        # Network settings
        self.server_ip = server_ip
        self.server_port = server_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        
        # Data logging
        self.save_data = save_data
        self.data_file = None
        if self.save_data:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.data_file = f"/home/pi/testIMU/data_log_{timestamp}.jsonl"
        
        # Record sessione
        self.inizio_sessione = None
        self.max_g_lat = 0
        self.max_g_freno = 0
        self.max_g_accel = 0
        self.pacchetti_inviati = 0
        
        # Media mobile (aumentata per più stabilità)
        self.storico_gx = deque(maxlen=3)  # Ridotto per risposta più rapida
        self.storico_gy = deque(maxlen=3)  # Ridotto per risposta più rapida
        self.storico_heading = deque(maxlen=10)

        # ===== AGGIUNTO PER METODO 2 =====
        self.storico_pitch = deque(maxlen=5)
        self.storico_pitch_rate = deque(maxlen=5)
        self.storico_speed_delta = deque(maxlen=5)
        self.last_speed = 0
        self.last_speed_time = time.time()

        self.heading_iniziale = None

        # GPS Data (thread-safe)
        self.gps_data = {
            'latitude': None,
            'longitude': None,
            'speed_kmh': 0.0,
            'satellites': 0,
            'quality': 0,
        }
        self.gps_lock = threading.Lock()
        self.running = True
        self.gps_thread = None

        # Velocità precedente per calcolo variazione (fusion method)
        self.prev_speed_kmh = 0.0
        
    def inizializza(self):
        """Inizializza sensore"""
        try:
            print("\n" + "═"*60)
            print("  🏁 TELEMETRIA KART - SENDER UDP [SENSIBILITÀ ALTA]")
            print("═"*60)
            print(f"\n📡 Server: {self.server_ip}:{self.server_port}")
            print("⏳ Inizializzazione sensore BNO085...")
            
            self.i2c = ExtendedI2C(self.bus)
            time.sleep(0.1)
            self.sensore = BNO08X_I2C(self.i2c, address=self.indirizzo)
            time.sleep(0.2)

            self.sensore.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            time.sleep(0.05)
            self.sensore.enable_feature(BNO_REPORT_GYROSCOPE)
            time.sleep(0.05)
            self.sensore.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.05)
            self.sensore.enable_feature(BNO_REPORT_MAGNETOMETER)
            time.sleep(0.05)

            self.inizio_sessione = datetime.now()
            print("✅ Sensore pronto!")
            
            # Calibra heading
            print("📍 Calibrazione bussola...")
            time.sleep(1)
            for _ in range(10):
                try:
                    mag = self.sensore.magnetic
                    heading = calcola_heading(mag[0], mag[1])
                    self.storico_heading.append(heading)
                    time.sleep(0.1)
                except:
                    pass
            
            if self.storico_heading:
                self.heading_iniziale = sum(self.storico_heading) / len(self.storico_heading)
                print(f"✅ Heading iniziale: {self.heading_iniziale:.1f}°")
            
            print("\n✅ Pronto per inviare dati!")
            print("🔥 SENSIBILITÀ ALTA: soglie ridotte per rilevamento precoce\n")
            return True
            
        except Exception as e:
            print(f"❌ Errore: {e}")
            return False

    def media(self, storico):
        return sum(storico) / len(storico) if storico else 0
    
    def salva_dati(self, pacchetto):
        """Salva il pacchetto JSON in formato JSONL (una riga per pacchetto)"""
        if not self.save_data or not self.data_file:
            return
        try:
            with open(self.data_file, 'a') as f:
                f.write(json.dumps(pacchetto) + '\n')
        except Exception as e:
            print(f"⚠️  Errore salvataggio dati: {e}")
    
    
    def gps_worker(self):
        """Legge dati GPS da seriale"""
        if not PYNMEA2_AVAILABLE:
            return
        
        print(f"[GPS] seriale {GPS_PORT} @ {GPS_BAUD}")
        while self.running:
            try:
                with serial.Serial(GPS_PORT, GPS_BAUD, timeout=1) as ser:
                    for raw in ser:
                        if not self.running:
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
                        
                        # VTG - Velocità
                        if isinstance(msg, pynmea2.VTG):
                            with self.gps_lock:
                                try:
                                    if msg.spd_over_grnd_kmph is not None:
                                        self.gps_data['speed_kmh'] = float(msg.spd_over_grnd_kmph)
                                    elif msg.spd_over_grnd_kts is not None:
                                        self.gps_data['speed_kmh'] = float(msg.spd_over_grnd_kts) * 1.852
                                except (ValueError, TypeError):
                                    pass
                        
                        # RMC - Posizione
                        elif isinstance(msg, pynmea2.RMC):
                            with self.gps_lock:
                                if msg.status == 'A' and msg.latitude and msg.longitude:
                                    self.gps_data['latitude'] = msg.latitude
                                    self.gps_data['longitude'] = msg.longitude
                        
                        # GGA - Qualità e satelliti
                        elif isinstance(msg, pynmea2.GGA):
                            with self.gps_lock:
                                try:
                                    self.gps_data['satellites'] = int(msg.num_sats) if msg.num_sats else 0
                                    self.gps_data['quality'] = int(msg.gps_qual) if msg.gps_qual else 0
                                except (ValueError, TypeError):
                                    pass
                                
                                if msg.gps_qual and int(msg.gps_qual) > 0 and msg.latitude and msg.longitude:
                                    self.gps_data['latitude'] = msg.latitude
                                    self.gps_data['longitude'] = msg.longitude
            
            except serial.SerialException:
                time.sleep(3)
            except Exception as e:
                print(f"[GPS] Errore: {e}")
                time.sleep(3)

    def leggi_e_invia(self):
        """Leggi sensori e invia DATI RAW via UDP"""
        try:
            # Leggi sensori raw
            lin = self.sensore.linear_acceleration
            gyro = self.sensore.gyro
            quat = self.sensore.quaternion
            
            # Magnetometro
            try:
                mag = self.sensore.magnetic
                mag_x, mag_y, mag_z = mag
                heading = calcola_heading(mag_x, mag_y)
                self.storico_heading.append(heading)
                heading_smooth = self.media(self.storico_heading)
                intensita_campo = math.sqrt(mag_x**2 + mag_y**2 + mag_z**2)
            except:
                mag_x = mag_y = mag_z = 0
                heading_smooth = 0
                intensita_campo = 0
            
            # Calcola forze G
            gx = lin[0] / 9.81
            gy = lin[1] / 9.81
            gz = lin[2] / 9.81
            
            # Velocità angolari
            vel_yaw = math.degrees(gyro[2])
            vel_roll = math.degrees(gyro[0])
            vel_pitch = math.degrees(gyro[1])
            
            # Orientamento da quaternione
            roll, pitch, yaw = quaternion_to_euler(*quat)
            
            # Media mobile per smoothing (facoltativo ma utile)
            self.storico_gx.append(gx)
            self.storico_gy.append(gy)
            gx_smooth = self.media(self.storico_gx)
            gy_smooth = self.media(self.storico_gy)

            # ===== AGGIUNTO PER METODO 2 =====
            self.storico_pitch.append(pitch)
            self.storico_pitch_rate.append(vel_pitch)
            pitch_smooth = self.media(self.storico_pitch)
            pitch_rate_smooth = self.media(self.storico_pitch_rate)
            
            # Aggiorna record massimi
            self.max_g_lat = max(self.max_g_lat, abs(gx))
            if gy < 0:
                self.max_g_freno = max(self.max_g_freno, abs(gy))
            else:
                self.max_g_accel = max(self.max_g_accel, gy)
            
            # Leggi dati GPS
            with self.gps_lock:
                gps_snap = self.gps_data.copy()

            current_speed_kmh = gps_snap['speed_kmh']
            speed_variation = (current_speed_kmh - self.prev_speed_kmh) / 10.0
            self.prev_speed_kmh = current_speed_kmh

            # ===== AGGIUNTO PER METODO 2: Speed variation smoothed =====
            now = time.time()
            dt_speed = now - self.last_speed_time
            if dt_speed > 0 and dt_speed < 1.0:
                speed_delta = (gps_snap['speed_kmh'] - self.last_speed) / dt_speed
                self.storico_speed_delta.append(speed_delta)
                speed_variation_smooth = self.media(self.storico_speed_delta)
            else:
                speed_variation_smooth = 0

            self.last_speed = gps_snap['speed_kmh']
            self.last_speed_time = now
            
            # Tempo sessione
            trascorso = (datetime.now() - self.inizio_sessione).total_seconds()
            
            # JSON CON DATI COMPLETI PER METODO 2 - Fusion sul server
            pacchetto = {
                "timestamp": datetime.now().isoformat(),
                "session_time": trascorso,

                # ===== DATI RAW SENSORES =====
                "raw_sensors": {
                    "accel_x": round(gx, 3),
                    "accel_y": round(gy, 3),
                    "accel_z": round(gz, 3),
                    "gyro_yaw": round(vel_yaw, 2),
                    "gyro_roll": round(vel_roll, 2),
                    "gyro_pitch": round(vel_pitch, 2),
                    "yaw_rate": round(vel_yaw, 2),
                    "pitch_angle": round(pitch, 2),
                    "roll_angle": round(roll, 2),
                    "yaw_angle": round(yaw, 2)
                },

                # ===== VALORI ELABORATI =====
                "processed": {
                    "accel_y_smooth": round(gy_smooth, 3),
                    "pitch_smooth": round(pitch_smooth, 2),
                    "pitch_rate_smooth": round(pitch_rate_smooth, 2),
                    "speed_variation": round(speed_variation_smooth, 3)
                },

                # ===== FORZE G =====
                "g_force": {
                    "lateral": round(gx, 3),
                    "longitudinal": round(gy, 3),
                    "vertical": round(gz, 3),
                    "lateral_smooth": round(gx_smooth, 3),
                    "longitudinal_smooth": round(gy_smooth, 3)
                },

                # ===== GPS =====
                "gps": {
                    "latitude": gps_snap['latitude'],
                    "longitude": gps_snap['longitude'],
                    "speed_kmh": round(gps_snap['speed_kmh'], 1),
                    "speed_variation": round(speed_variation_smooth, 3),
                    "satellites": gps_snap['satellites'],
                    "quality": gps_snap['quality']
                },

                # ===== MAGNETOMETRO =====
                "magnetometer": {
                    "x": round(mag_x, 2),
                    "y": round(mag_y, 2),
                    "z": round(mag_z, 2),
                    "heading": round(heading_smooth, 1),
                    "field_strength": round(intensita_campo, 1)
                },

                # ===== RECORD SESSIONE =====
                "records": {
                    "max_lateral_g": round(self.max_g_lat, 2),
                    "max_brake_g": round(self.max_g_freno, 2),
                    "max_accel_g": round(self.max_g_accel, 2)
                },

                # ===== METADATA =====
                "packet_number": self.pacchetti_inviati
            }
            
            # Salva dati su file
            self.salva_dati(pacchetto)
            
            # Invia JSON via UDP
            json_data = json.dumps(pacchetto)
            self.sock.sendto(json_data.encode('utf-8'), (self.server_ip, self.server_port))
            
            self.pacchetti_inviati += 1
            
            # Status locale
            if self.pacchetti_inviati % 10 == 0:
                gps_str = ""
                if gps_snap['latitude'] is not None:
                    gps_str = f"GPS: {gps_snap['latitude']:.5f},{gps_snap['longitude']:.5f} Sat:{gps_snap['satellites']} | "
                
                print(f"📡 Inviati {self.pacchetti_inviati} pacchetti | "
                      f"G: Lat {gx:+.2f} Long {gy:+.2f} | "
                      f"{gps_str}"
                      f"V: {gps_snap['speed_kmh']:.1f} km/h | "
                      f"Heading: {heading_smooth:.0f}°")
            
        except Exception as e:
            print(f"⚠️  Errore: {e}")
            time.sleep(0.1)

    def avvia_gps(self):
        """Avvia il thread GPS"""
        if PYNMEA2_AVAILABLE:
            self.gps_thread = threading.Thread(target=self.gps_worker, daemon=True)
            self.gps_thread.start()
    
    def chiudi(self):
        """Chiudi connessioni"""
        try:
            self.running = False
            if self.sock:
                self.sock.close()
            if self.i2c:
                self.i2c.deinit()
            if self.gps_thread and self.gps_thread.is_alive():
                self.gps_thread.join(timeout=2)
        except:
            pass


def main():
    # Abilita GPIO per modem GNSS
    enable_gpio()
    
    # CONFIGURAZIONE DI BASE
    SERVER_IP = "193.70.113.55"
    SERVER_PORT = 5005
    
    print("\n" + "═"*60)
    print("  📡 TELEMETRIA KART - CONFIG LOCALE")
    print("═"*60)
    print(f"\n📡 Server: {SERVER_IP}:{SERVER_PORT}\n")
    
    telemetria = TelemetriaKartSender(
        bus=3, 
        indirizzo=0x4A,
        server_ip=SERVER_IP,
        server_port=SERVER_PORT,
        save_data=True  # Abilita salvataggio dati
    )
    
    if not telemetria.inizializza():
        sys.exit(1)
    
    print(f"💾 Salvataggio dati abilitato: {telemetria.data_file}\n")
    
    # Avvia thread GPS
    telemetria.avvia_gps()

    try:
        while True:
            telemetria.leggi_e_invia()
            time.sleep(0.1)  # 10Hz = 10 pacchetti/secondo
            
    except KeyboardInterrupt:
        trascorso = (datetime.now() - telemetria.inizio_sessione).total_seconds()
        
        print("\n\n" + "═"*60)
        print("  🏁 SESSIONE TERMINATA")
        print("═"*60)
        print(f"\n  Durata:           {int(trascorso//60)}m {int(trascorso%60)}s")
        print(f"  Pacchetti inviati: {telemetria.pacchetti_inviati}")
        print(f"  Max Frenata:      {telemetria.max_g_freno:.2f}g")
        print(f"  Max Accel:        {telemetria.max_g_accel:.2f}g")
        print(f"  Max Laterali:     {telemetria.max_g_lat:.2f}g")
        print("\n" + "═"*60 + "\n")
        
    finally:
        telemetria.chiudi()


if __name__ == "__main__":
    main()
