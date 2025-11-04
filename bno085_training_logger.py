#!/usr/bin/env python3
"""
BNO085 Training Data Logger
Registra tutti i dati IMU necessari per training del modello AI
Frequenza: 20 Hz (50ms tra campioni)
"""

import time
import csv
import sys
import math
from datetime import datetime
from adafruit_extended_bus import ExtendedI2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)


class BNO085TrainingLogger:
    """Logger per dati training track mapping"""
    
    def __init__(self, bus=3, address=0x4A, sample_rate_hz=20):
        self.bus = bus
        self.address = address
        self.sample_rate_hz = sample_rate_hz
        self.sample_interval = 1.0 / sample_rate_hz
        self.sensor = None
        self.i2c = None
        
        # Crea nome file con timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f"track_training_{timestamp}.csv"
        
        # Contatori
        self.sample_count = 0
        self.start_time = None
        
    def initialize(self):
        """Inizializza connessione I2C e sensore"""
        try:
            print(f"Connessione BNO085 su bus {self.bus}, indirizzo 0x{self.address:02X}...")
            
            # Crea connessione I2C software
            self.i2c = ExtendedI2C(self.bus)
            time.sleep(0.1)
            
            # Inizializza sensore
            self.sensor = BNO08X_I2C(self.i2c, address=self.address)
            time.sleep(0.5)
            
            # Abilita SOLO i sensori necessari per training
            sensors = [
                (BNO_REPORT_LINEAR_ACCELERATION, "Accelerazione Lineare"),
                (BNO_REPORT_GYROSCOPE, "Giroscopio"),
                (BNO_REPORT_MAGNETOMETER, "Magnetometro"),
                (BNO_REPORT_ROTATION_VECTOR, "Quaternioni"),
            ]
            
            print("Abilitazione sensori...")
            for sensor_type, name in sensors:
                try:
                    self.sensor.enable_feature(sensor_type)
                    print(f"  ✓ {name}")
                    time.sleep(0.3)
                except RuntimeError as e:
                    print(f"  ✗ {name}: {e}")
                    return False
            
            print("✓ BNO085 inizializzato!\n")
            return True
            
        except Exception as e:
            print(f"✗ Errore inizializzazione: {e}")
            return False
    
    def init_csv(self):
        """Crea file CSV con header"""
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',      # Timestamp Unix (secondi)
                'accel_x',       # Accelerazione lineare X (m/s²)
                'accel_y',       # Accelerazione lineare Y (m/s²)
                'accel_z',       # Accelerazione lineare Z (m/s²)
                'gyro_x',        # Velocità angolare X (rad/s)
                'gyro_y',        # Velocità angolare Y (rad/s)
                'gyro_z',        # Velocità angolare Z (rad/s)
                'mag_x',         # Campo magnetico X (µT)
                'mag_y',         # Campo magnetico Y (µT)
                'mag_z',         # Campo magnetico Z (µT)
                'quat_i',        # Quaternione i
                'quat_j',        # Quaternione j
                'quat_k',        # Quaternione k
                'quat_r',        # Quaternione real
                'yaw',           # Angolo yaw (gradi)
                'pitch',         # Angolo pitch (gradi)
                'roll',          # Angolo roll (gradi)
            ])
        print(f"File CSV creato: {self.filename}\n")
    
    def quaternion_to_euler(self, i, j, k, r):
        """Converte quaternioni in angoli di Eulero (gradi)"""
        # Roll (X-axis rotation)
        roll = math.atan2(2*(r*i + j*k), 1 - 2*(i**2 + j**2))
        
        # Pitch (Y-axis rotation)
        sinp = 2*(r*j - k*i)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (Z-axis rotation)
        yaw = math.atan2(2*(r*k + i*j), 1 - 2*(j**2 + k**2))
        
        # Converti in gradi
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def log_sample(self):
        """Legge e salva un campione di dati"""
        try:
            # Leggi tutti i sensori
            lin_accel = self.sensor.linear_acceleration
            gyro = self.sensor.gyro
            mag = self.sensor.magnetic
            quat = self.sensor.quaternion
            
            # Verifica che tutti i dati siano validi
            if not all([lin_accel, gyro, mag, quat]):
                return False
            
            # Calcola angoli di Eulero
            roll, pitch, yaw = self.quaternion_to_euler(*quat)
            
            # Timestamp
            ts = time.time()
            
            # Scrivi su CSV
            with open(self.filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f"{ts:.6f}",
                    f"{lin_accel[0]:.6f}",
                    f"{lin_accel[1]:.6f}",
                    f"{lin_accel[2]:.6f}",
                    f"{gyro[0]:.6f}",
                    f"{gyro[1]:.6f}",
                    f"{gyro[2]:.6f}",
                    f"{mag[0]:.6f}",
                    f"{mag[1]:.6f}",
                    f"{mag[2]:.6f}",
                    f"{quat[0]:.6f}",
                    f"{quat[1]:.6f}",
                    f"{quat[2]:.6f}",
                    f"{quat[3]:.6f}",
                    f"{yaw:.6f}",
                    f"{pitch:.6f}",
                    f"{roll:.6f}",
                ])
            
            self.sample_count += 1
            return True
            
        except Exception as e:
            print(f"\n⚠ Errore lettura: {e}")
            return False
    
    def print_stats(self):
        """Stampa statistiche correnti"""
        if self.start_time:
            elapsed = time.time() - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0
            print(f"\r📊 Campioni: {self.sample_count:6d} | "
                  f"Freq: {rate:5.1f} Hz | "
                  f"Tempo: {elapsed:6.1f}s", end='', flush=True)
    
    def cleanup(self):
        """Chiude connessione"""
        try:
            if self.i2c:
                self.i2c.deinit()
        except:
            pass


def main():
    """Main function"""
    print("="*70)
    print("🏁 BNO085 TRAINING DATA LOGGER")
    print("="*70)
    print()
    
    # Inizializza logger
    logger = BNO085TrainingLogger(bus=3, address=0x4A, sample_rate_hz=20)
    
    # Inizializza sensore
    if not logger.initialize():
        print("✗ Impossibile inizializzare il sensore")
        sys.exit(1)
    
    # Crea file CSV
    logger.init_csv()
    
    print("="*70)
    print("📝 REGISTRAZIONE AVVIATA")
    print("="*70)
    print(f"File output: {logger.filename}")
    print(f"Frequenza campionamento: {logger.sample_rate_hz} Hz")
    print()
    print("Istruzioni:")
    print("  1. Porta il sensore sulla pista")
    print("  2. Fai giri completi della pista")
    print("  3. Premi Ctrl+C quando hai finito")
    print()
    print("="*70)
    print()
    
    logger.start_time = time.time()
    last_sample_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            
            # Mantieni frequenza campionamento costante
            if current_time - last_sample_time >= logger.sample_interval:
                logger.log_sample()
                last_sample_time = current_time
                
                # Stampa statistiche ogni 10 campioni
                if logger.sample_count % 10 == 0:
                    logger.print_stats()
            
            # Piccolo delay per non saturare CPU
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("✓ REGISTRAZIONE TERMINATA")
        print("="*70)
        
        elapsed = time.time() - logger.start_time
        avg_rate = logger.sample_count / elapsed if elapsed > 0 else 0
        
        print(f"Campioni totali:     {logger.sample_count}")
        print(f"Tempo registrazione: {elapsed:.1f} secondi")
        print(f"Frequenza media:     {avg_rate:.1f} Hz")
        print(f"File salvato:        {logger.filename}")
        print("="*70)
        print()
        
        # Mostra preview dei dati
        print("📊 Preview dati (prime 3 righe):")
        try:
            with open(logger.filename, 'r') as f:
                for i, line in enumerate(f):
                    if i < 4:  # Header + 3 righe
                        print(f"  {line.rstrip()}")
                    else:
                        break
        except:
            pass
        
        print()
        print("✅ Dati pronti per il training del modello AI!")
        print()
        
    finally:
        logger.cleanup()


if __name__ == "__main__":
    main()
