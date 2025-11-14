#!/usr/bin/env python3
"""
Telemetria Kart - Sender UDP con 5 Metodi di Rilevamento
Invia dati comparativi per analisi frontend
VERSIONE COMPLETA - Tutti i metodi in parallelo
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


class KalmanFilter1D:
    """Filtro di Kalman 1D semplificato"""
    def __init__(self, process_variance=0.02, measurement_variance=0.15):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0
        self.error_estimate = 1
    
    def update(self, measurement):
        # Prediction
        prediction = self.estimate
        error_prediction = self.error_estimate + self.process_variance
        
        # Update
        kalman_gain = error_prediction / (error_prediction + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.error_estimate = (1 - kalman_gain) * error_prediction
        
        return self.estimate


class TelemetriaKartSender:
    def __init__(self, bus=3, indirizzo=0x4A, server_ip="193.70.113.55", server_port=5005):
        self.bus = bus
        self.indirizzo = indirizzo
        self.sensore = None
        self.i2c = None
        
        # Network settings
        self.server_ip = server_ip
        self.server_port = server_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Record sessione
        self.inizio_sessione = None
        self.max_g_lat = 0
        self.max_g_freno = 0
        self.max_g_accel = 0
        self.pacchetti_inviati = 0
        
        # Storico base
        self.storico_gx = deque(maxlen=3)
        self.storico_gy = deque(maxlen=3)
        self.storico_heading = deque(maxlen=10)
        self.heading_iniziale = None
        
        # ===== METODO 1: Base (già presente) =====
        # usa storico_gy
        
        # ===== METODO 2: Fusione Multi-Sensore =====
        self.storico_pitch_rate = deque(maxlen=5)
        self.storico_speed_delta = deque(maxlen=5)
        self.last_speed = 0
        self.last_speed_time = time.time()
        
        # ===== METODO 3: Jerk Analysis =====
        self.storico_gy_jerk = deque(maxlen=5)
        self.ultimo_gy = 0
        self.ultimo_timestamp = time.time()
        
        # ===== METODO 4: Pitch Angle =====
        self.storico_pitch = deque(maxlen=5)
        
        # ===== METODO 5: GPS Acceleration =====
        self.storico_speed_gps = deque(maxlen=5)
        self.storico_speed_time = deque(maxlen=5)
        
        # Filtro Kalman per metodo avanzato
        self.kalman_gy = KalmanFilter1D()
        
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
        
    def inizializza(self):
        """Inizializza sensore"""
        try:
            print("\n" + "═"*60)
            print("  🏁 TELEMETRIA KART - 5 METODI DI RILEVAMENTO")
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
            
            print("\n✅ Pronto per inviare dati con 5 metodi comparativi!")
            print("📊 Metodi attivi:")
            print("   1. Base (Accelerazione Y smoothed)")
            print("   2. Fusione Multi-Sensore (Accel + Gyro + GPS)")
            print("   3. Jerk Analysis (Derivata accelerazione)")
            print("   4. Pitch Angle (Angolo beccheggio)")
            print("   5. GPS Acceleration (Velocità GPS derivata)")
            print()
            return True
            
        except Exception as e:
            print(f"❌ Errore: {e}")
            return False

    def media(self, storico):
        return sum(storico) / len(storico) if storico else 0
    
    # ===== METODO 1: BASE =====
    def metodo1_base(self, gy_smooth):
        """Metodo originale con accelerazione Y smoothed"""
        threshold = 0.15
        if gy_smooth >= threshold:
            stato = "ACCEL"
            confidence = min(abs(gy_smooth) / threshold, 1.0)
        elif gy_smooth <= -threshold:
            stato = "FRENATA"
            confidence = min(abs(gy_smooth) / threshold, 1.0)
        else:
            stato = "COSTANTE"
            confidence = 0.0
        
        return {
            "state": stato,
            "confidence": round(confidence, 3),
            "raw_value": round(gy_smooth, 3),
            "threshold": threshold
        }
    
    # ===== METODO 2: FUSIONE MULTI-SENSORE =====
    def metodo2_fusion(self, gy_smooth, pitch_rate, speed_variation):
        """Fusione: accelerometro + giroscopio + GPS"""
        # Pesi
        accel_score = gy_smooth * 0.5
        pitch_score = (pitch_rate / 50.0) * 0.3
        speed_score = speed_variation * 0.2
        
        combined_score = accel_score + pitch_score + speed_score
        
        threshold = 0.12
        if combined_score >= threshold:
            stato = "ACCEL"
            confidence = min(abs(combined_score) / threshold, 1.0)
        elif combined_score <= -threshold:
            stato = "FRENATA"
            confidence = min(abs(combined_score) / threshold, 1.0)
        else:
            stato = "COSTANTE"
            confidence = 0.0
        
        return {
            "state": stato,
            "confidence": round(confidence, 3),
            "combined_score": round(combined_score, 3),
            "accel_component": round(accel_score, 3),
            "pitch_component": round(pitch_score, 3),
            "speed_component": round(speed_score, 3),
            "threshold": threshold
        }
    
    # ===== METODO 3: JERK ANALYSIS =====
    def calcola_jerk(self, gy_attuale):
        """Calcola jerk (derivata accelerazione)"""
        now = time.time()
        dt = now - self.ultimo_timestamp
        
        if dt > 0 and dt < 1.0:  # Evita spike da delay
            jerk = (gy_attuale - self.ultimo_gy) / dt
            self.storico_gy_jerk.append(jerk)
            
            self.ultimo_gy = gy_attuale
            self.ultimo_timestamp = now
            
            return self.media(self.storico_gy_jerk)
        
        self.ultimo_gy = gy_attuale
        self.ultimo_timestamp = now
        return 0
    
    def metodo3_jerk(self, jerk):
        """Analisi del jerk per rilevamento precoce"""
        threshold_accel = 0.8  # g/s
        threshold_brake = -1.2  # g/s
        
        if jerk >= threshold_accel:
            stato = "ACCEL"
            confidence = min(abs(jerk) / threshold_accel, 1.0)
        elif jerk <= threshold_brake:
            stato = "FRENATA"
            confidence = min(abs(jerk) / abs(threshold_brake), 1.0)
        else:
            stato = "COSTANTE"
            confidence = 0.0
        
        return {
            "state": stato,
            "confidence": round(confidence, 3),
            "jerk_value": round(jerk, 3),
            "threshold_accel": threshold_accel,
            "threshold_brake": threshold_brake
        }
    
    # ===== METODO 4: PITCH ANGLE =====
    def metodo4_pitch(self, pitch, pitch_rate):
        """Analisi angolo di beccheggio"""
        pitch_threshold = 2.5  # gradi
        rate_threshold = 5.0   # deg/s
        
        # Pitch positivo = muso alto (accelerazione)
        # Pitch negativo = muso basso (frenata)
        
        if pitch > pitch_threshold and pitch_rate > rate_threshold:
            stato = "ACCEL"
            confidence = min(abs(pitch) / (pitch_threshold * 2), 1.0)
        elif pitch < -pitch_threshold and pitch_rate < -rate_threshold:
            stato = "FRENATA"
            confidence = min(abs(pitch) / (pitch_threshold * 2), 1.0)
        else:
            stato = "COSTANTE"
            confidence = 0.0
        
        return {
            "state": stato,
            "confidence": round(confidence, 3),
            "pitch_angle": round(pitch, 2),
            "pitch_rate": round(pitch_rate, 2),
            "threshold_angle": pitch_threshold,
            "threshold_rate": rate_threshold
        }
    
    # ===== METODO 5: GPS ACCELERATION =====
    def calcola_accel_gps(self, speed_kmh):
        """Accelerazione calcolata da variazione velocità GPS"""
        now = time.time()
        speed_ms = speed_kmh / 3.6
        
        self.storico_speed_gps.append(speed_ms)
        self.storico_speed_time.append(now)
        
        if len(self.storico_speed_gps) >= 3:
            dt = self.storico_speed_time[-1] - self.storico_speed_time[0]
            dv = self.storico_speed_gps[-1] - self.storico_speed_gps[0]
            
            if dt > 0:
                accel_ms2 = dv / dt
                return accel_ms2 / 9.81  # Converti in g
        
        return 0
    
    def metodo5_gps(self, gps_accel):
        """Accelerazione da GPS"""
        threshold = 0.18
        
        if gps_accel >= threshold:
            stato = "ACCEL"
            confidence = min(abs(gps_accel) / threshold, 1.0)
        elif gps_accel <= -threshold:
            stato = "FRENATA"
            confidence = min(abs(gps_accel) / threshold, 1.0)
        else:
            stato = "COSTANTE"
            confidence = 0.0
        
        return {
            "state": stato,
            "confidence": round(confidence, 3),
            "gps_accel": round(gps_accel, 3),
            "threshold": threshold
        }
    
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
        """Leggi sensori, calcola tutti i metodi e invia via UDP"""
        try:
            # Leggi sensori
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
            
            # Calcola valori base
            gx = lin[0] / 9.81
            gy = lin[1] / 9.81
            gz = lin[2] / 9.81
            
            vel_yaw = math.degrees(gyro[2])
            vel_roll = math.degrees(gyro[0])
            vel_pitch = math.degrees(gyro[1])
            
            roll, pitch, yaw = quaternion_to_euler(*quat)
            
            # Aggiorna storico base
            self.storico_gx.append(gx)
            self.storico_gy.append(gy)
            self.storico_pitch.append(pitch)
            self.storico_pitch_rate.append(vel_pitch)
            
            gx_smooth = self.media(self.storico_gx)
            gy_smooth = self.media(self.storico_gy)
            pitch_smooth = self.media(self.storico_pitch)
            pitch_rate_smooth = self.media(self.storico_pitch_rate)
            
            # Filtro Kalman
            gy_kalman = self.kalman_gy.update(gy)
            
            # Leggi GPS
            with self.gps_lock:
                gps_snap = self.gps_data.copy()
            
            # Calcola variazione velocità GPS per fusione
            now = time.time()
            dt_speed = now - self.last_speed_time
            if dt_speed > 0 and dt_speed < 1.0:
                speed_delta = (gps_snap['speed_kmh'] - self.last_speed) / dt_speed
                self.storico_speed_delta.append(speed_delta)
                speed_variation = self.media(self.storico_speed_delta)
            else:
                speed_variation = 0
            
            self.last_speed = gps_snap['speed_kmh']
            self.last_speed_time = now
            
            # Calcola JERK
            jerk = self.calcola_jerk(gy)
            
            # Calcola accelerazione GPS
            gps_accel = self.calcola_accel_gps(gps_snap['speed_kmh'])
            
            # ===== APPLICA TUTTI E 5 I METODI =====
            method1 = self.metodo1_base(gy_smooth)
            method2 = self.metodo2_fusion(gy_smooth, pitch_rate_smooth, speed_variation)
            method3 = self.metodo3_jerk(jerk)
            method4 = self.metodo4_pitch(pitch_smooth, pitch_rate_smooth)
            method5 = self.metodo5_gps(gps_accel)
            
            # Aggiorna record
            self.max_g_lat = max(self.max_g_lat, abs(gx))
            if gy < 0:
                self.max_g_freno = max(self.max_g_freno, abs(gy))
            else:
                self.max_g_accel = max(self.max_g_accel, gy)
            
            # Curva
            in_curva = False
            direzione_curva = None
            if abs(gx) > 0.15:
                in_curva = True
                direzione_curva = "SINISTRA" if gx < 0 else "DESTRA"
            
            # Tempo sessione
            trascorso = (datetime.now() - self.inizio_sessione).total_seconds()
            
            # ===== CREA PACCHETTO JSON COMPLETO =====
            pacchetto = {
                "timestamp": datetime.now().isoformat(),
                "session_time": trascorso,
                
                # Dati RAW sensori
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
                
                # Valori processati
                "processed": {
                    "accel_y_smooth": round(gy_smooth, 3),
                    "accel_y_kalman": round(gy_kalman, 3),
                    "pitch_smooth": round(pitch_smooth, 2),
                    "pitch_rate_smooth": round(pitch_rate_smooth, 2),
                    "jerk": round(jerk, 3),
                    "speed_variation": round(speed_variation, 3),
                    "gps_acceleration": round(gps_accel, 3)
                },
                
                # METODI DI RILEVAMENTO (5)
                "detection_methods": {
                    "method1_base": method1,
                    "method2_fusion": method2,
                    "method3_jerk": method3,
                    "method4_pitch": method4,
                    "method5_gps": method5
                },
                
                # Forze G
                "g_force": {
                    "lateral": round(gx, 3),
                    "longitudinal": round(gy, 3),
                    "vertical": round(gz, 3),
                    "lateral_smooth": round(gx_smooth, 3),
                    "longitudinal_smooth": round(gy_smooth, 3)
                },
                
                # GPS RTK
                "gps": {
                    "latitude": gps_snap['latitude'],
                    "longitude": gps_snap['longitude'],
                    "speed_kmh": round(gps_snap['speed_kmh'], 1),
                    "satellites": gps_snap['satellites'],
                    "quality": gps_snap['quality']
                },
                
                # Magnetometro
                "magnetometer": {
                    "x": round(mag_x, 2),
                    "y": round(mag_y, 2),
                    "z": round(mag_z, 2),
                    "heading": round(heading_smooth, 1),
                    "field_strength": round(intensita_campo, 1)
                },
                
                # Stato guida (dal metodo 1 come default)
                "state": {
                    "driving": method1["state"],
                    "in_corner": in_curva,
                    "corner_direction": direzione_curva
                },
                
                # Record sessione
                "records": {
                    "max_lateral_g": round(self.max_g_lat, 2),
                    "max_brake_g": round(self.max_g_freno, 2),
                    "max_accel_g": round(self.max_g_accel, 2)
                },
                
                # Metadata
                "packet_number": self.pacchetti_inviati
            }
            
            # Converti in JSON e invia via UDP
            json_data = json.dumps(pacchetto)
            self.sock.sendto(json_data.encode('utf-8'), (self.server_ip, self.server_port))
            
            self.pacchetti_inviati += 1
            
            # Mostra status locale
            if self.pacchetti_inviati % 10 == 0:
                gps_info = ""
                if gps_snap['latitude'] and gps_snap['longitude']:
                    gps_info = f" | GPS: {gps_snap['latitude']} {gps_snap['longitude']} Sat:{gps_snap['satellites']}"
                print(f"📡 #{self.pacchetti_inviati} | "
                      f"M1:{method1['state'][:1]} M2:{method2['state'][:1]} "
                      f"M3:{method3['state'][:1]} M4:{method4['state'][:1]} M5:{method5['state'][:1]} | "
                      f"G: {gy:+.2f} | Jerk: {jerk:+.2f} | "
                      f"Pitch: {pitch:.1f}° | Speed: {gps_snap['speed_kmh']:.0f}km/h{gps_info}")
            
        except Exception as e:
            print(f"⚠️  Errore: {e}")
            import traceback
            traceback.print_exc()
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
    
    # CONFIGURAZIONE
    SERVER_IP = "193.70.113.55"
    SERVER_PORT = 5005
    
    print("\n" + "═"*60)
    print("  📡 TELEMETRIA KART - COMPARAZIONE 5 METODI")
    print("═"*60)
    print(f"\n📡 Server: {SERVER_IP}:{SERVER_PORT}")
    print("📊 Inviando tutti e 5 i metodi in parallelo\n")
    
    telemetria = TelemetriaKartSender(
        bus=3, 
        indirizzo=0x4A,
        server_ip=SERVER_IP,
        server_port=SERVER_PORT
    )
    
    if not telemetria.inizializza():
        sys.exit(1)
    
    # Avvia thread GPS
    telemetria.avvia_gps()

    try:
        while True:
            telemetria.leggi_e_invia()
            time.sleep(0.1)  # 10Hz
            
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