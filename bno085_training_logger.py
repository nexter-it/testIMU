#!/usr/bin/env python3
"""
BNO085 Live Monitor - Tutti i Dati Disponibili
Mostra in tempo reale TUTTI i dati IMU utili per training
Include velocità stimata (con avviso sulla deriva!)
"""

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


class BNO085Monitor:
    """Monitor completo per BNO085"""
    
    def __init__(self, bus=3, address=0x4A):
        self.bus = bus
        self.address = address
        self.sensor = None
        self.i2c = None
        
        # Variabili per calcolo velocità (ATTENZIONE: deriva!)
        self.velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz] in m/s
        self.last_time = None
        self.start_time = None
        
        # Contatori per statistiche
        self.sample_count = 0
        
    def initialize(self):
        """Inizializza BNO085"""
        try:
            print(f"Connessione BNO085 su bus {self.bus}...\n")
            
            self.i2c = ExtendedI2C(self.bus)
            time.sleep(0.1)
            
            self.sensor = BNO08X_I2C(self.i2c, address=self.address)
            time.sleep(0.5)
            
            # Abilita TUTTI i sensori
            sensors = [
                (BNO_REPORT_LINEAR_ACCELERATION, "Accelerazione Lineare"),
                (BNO_REPORT_GYROSCOPE, "Giroscopio"),
                (BNO_REPORT_MAGNETOMETER, "Magnetometro"),
                (BNO_REPORT_ROTATION_VECTOR, "Quaternioni"),
            ]
            
            for sensor_type, name in sensors:
                try:
                    self.sensor.enable_feature(sensor_type)
                    print(f"  ✓ {name}")
                    time.sleep(0.3)
                except RuntimeError as e:
                    print(f"  ✗ {name}: {e}")
                    return False
            
            print("\n✓ BNO085 pronto!\n")
            self.start_time = time.time()
            self.last_time = self.start_time
            return True
            
        except Exception as e:
            print(f"✗ Errore: {e}")
            return False
    
    def quaternion_to_euler(self, i, j, k, r):
        """Converte quaternioni in Eulero (gradi)"""
        roll = math.atan2(2*(r*i + j*k), 1 - 2*(i**2 + j**2))
        
        sinp = 2*(r*j - k*i)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        
        yaw = math.atan2(2*(r*k + i*j), 1 - 2*(j**2 + k**2))
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def estimate_velocity(self, lin_accel):
        """
        Stima velocità integrando accelerazione
        ⚠️ ATTENZIONE: Deriva rapidamente! Solo per test!
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Integrazione semplice: v = v0 + a*dt
        for i in range(3):
            self.velocity[i] += lin_accel[i] * dt
        
        self.last_time = current_time
        
        # Calcola magnitudine velocità
        speed = math.sqrt(sum(v**2 for v in self.velocity))
        
        return self.velocity, speed
    
    def read_and_display(self):
        """Legge tutti i dati e li mostra"""
        try:
            # Leggi tutti i sensori
            lin_accel = self.sensor.linear_acceleration
            gyro = self.sensor.gyro
            mag = self.sensor.magnetic
            quat = self.sensor.quaternion
            
            # Calcola derivati
            roll, pitch, yaw = self.quaternion_to_euler(*quat)
            velocity, speed = self.estimate_velocity(lin_accel)
            
            # Calcola magnitudini
            accel_mag = math.sqrt(sum(a**2 for a in lin_accel))
            gyro_mag = math.sqrt(sum(g**2 for g in gyro))
            mag_mag = math.sqrt(sum(m**2 for m in mag))
            
            # Tempo trascorso
            elapsed = time.time() - self.start_time
            
            self.sample_count += 1
            
            # Clear screen e stampa
            print("\033[2J\033[H")  # Clear screen
            print("="*80)
            print("🏁 BNO085 LIVE MONITOR - TUTTI I DATI PER TRAINING")
            print("="*80)
            print(f"Tempo: {elapsed:.1f}s | Campioni: {self.sample_count} | Freq: {self.sample_count/elapsed:.1f} Hz")
            print("="*80)
            
            print("\n📊 DATI GREZZI (utili per training)")
            print("-"*80)
            
            # Accelerazione Lineare (senza gravità)
            print(f"Accelerazione Lineare (m/s²):")
            print(f"  X: {lin_accel[0]:8.4f}  Y: {lin_accel[1]:8.4f}  Z: {lin_accel[2]:8.4f}  |Mag|: {accel_mag:7.4f}")
            print(f"  → Rileva: Accelerazioni/Frenate (Y), Curve laterali (X)")
            
            # Giroscopio
            print(f"\nGiroscopio (rad/s):")
            print(f"  X: {gyro[0]:8.4f}  Y: {gyro[1]:8.4f}  Z: {gyro[2]:8.4f}  |Mag|: {gyro_mag:7.4f}")
            print(f"  → Rileva: Velocità di rotazione (Z importante per curve)")
            
            # Magnetometro
            print(f"\nMagnetometro (µT):")
            print(f"  X: {mag[0]:8.2f}  Y: {mag[1]:8.2f}  Z: {mag[2]:8.2f}  |Mag|: {mag_mag:7.2f}")
            print(f"  → Rileva: Orientamento assoluto rispetto al Nord")
            
            # Quaternioni
            print(f"\nQuaternioni (orientamento completo):")
            print(f"  i: {quat[0]:7.4f}  j: {quat[1]:7.4f}  k: {quat[2]:7.4f}  r: {quat[3]:7.4f}")
            
            # Angoli di Eulero
            print(f"\nAngoli di Eulero (gradi):")
            print(f"  Roll:  {roll:7.2f}°  (inclinazione laterale)")
            print(f"  Pitch: {pitch:7.2f}°  (inclinazione avanti/indietro)")
            print(f"  Yaw:   {yaw:7.2f}°  (direzione/heading) ← IMPORTANTE!")
            
            print("\n" + "="*80)
            print("⚠️  DATI DERIVATI (con limitazioni)")
            print("="*80)
            
            # Velocità stimata (CON AVVISO!)
            print(f"\nVelocità Stimata (integrazione accelerazione):")
            print(f"  Vx: {velocity[0]:7.3f} m/s  Vy: {velocity[1]:7.3f} m/s  Vz: {velocity[2]:7.3f} m/s")
            print(f"  Speed: {speed:7.3f} m/s ({speed*3.6:.1f} km/h)")
            
            if elapsed > 5:
                print(f"  ⚠️  ATTENZIONE: Deriva attiva da {elapsed:.0f}s! Errore stimato: ±{elapsed*0.01:.2f} m/s")
            if elapsed > 30:
                print(f"  🔴 DATI INAFFIDABILI dopo 30s! Usa GPS per velocità reale!")
            
            # Features derivate utili
            print(f"\n📈 Features Derivate (da calcolare post-processing):")
            print(f"  Delta Yaw:     (calcola variazione yaw tra campioni)")
            print(f"  Accel laterale: {lin_accel[0]:.4f} m/s² (per curve)")
            print(f"  Accel longitud: {lin_accel[1]:.4f} m/s² (per freno/accelerazione)")
            print(f"  Rotazione Z:    {gyro[2]:.4f} rad/s = {math.degrees(gyro[2]):.2f} °/s")
            
            print("\n" + "="*80)
            print("💡 RACCOMANDAZIONI PER TRAINING:")
            print("="*80)
            print("  ✅ USA: accel_x, accel_y, gyro_z, yaw, mag (campo magnetico)")
            print("  ✅ CALCOLA: delta_yaw, accel_magnitude, gyro_magnitude")
            print("  ❌ NON USARE: velocità stimata da IMU (deriva troppo!)")
            print("  ✅ PER VELOCITÀ: Aggiungi GPS durante training")
            print("="*80)
            
            return True
            
        except Exception as e:
            print(f"\n⚠️ Errore lettura: {e}")
            return False
    
    def cleanup(self):
        """Chiude connessione"""
        try:
            if self.i2c:
                self.i2c.deinit()
        except:
            pass


def main():
    """Main loop"""
    monitor = BNO085Monitor(bus=3, address=0x4A)
    
    if not monitor.initialize():
        print("✗ Impossibile inizializzare il sensore")
        sys.exit(1)
    
    print("Premi Ctrl+C per uscire\n")
    time.sleep(2)
    
    try:
        while True:
            monitor.read_and_display()
            time.sleep(0.2)  # 5 Hz refresh
            
    except KeyboardInterrupt:
        print("\n\n" + "="*80)
        print("✓ Monitor terminato")
        print("="*80)
        elapsed = time.time() - monitor.start_time
        print(f"Tempo totale: {elapsed:.1f}s")
        print(f"Campioni: {monitor.sample_count}")
        print(f"Frequenza media: {monitor.sample_count/elapsed:.1f} Hz")
        print("="*80 + "\n")
    finally:
        monitor.cleanup()


if __name__ == "__main__":
    main()
