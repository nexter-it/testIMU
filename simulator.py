#!/usr/bin/env python3
"""
Simulatore Telemetria Kart - Legge file JSONL e invia dati al server UDP
Utile per testare il server senza il sensore IMU fisico
"""

import json
import socket
import time
import sys
import os
from datetime import datetime

class TelemetriaSimulator:
    def __init__(self, data_file, server_ip="193.70.113.55", server_port=5005, speed=1.0):
        """
        Args:
            data_file: Path al file JSONL con i dati
            server_ip: IP del server
            server_port: Porta UDP del server
            speed: Velocità di riproduzione (1.0 = velocità originale, 2.0 = 2x più veloce)
        """
        self.data_file = data_file
        self.server_ip = server_ip
        self.server_port = server_port
        self.speed = speed
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pacchetti_inviati = 0
        
    def leggi_dati(self):
        """Legge il file JSONL e ritorna lista di pacchetti"""
        pacchetti = []
        try:
            with open(self.data_file, 'r') as f:
                for line in f:
                    if line.strip():
                        try:
                            pacchetto = json.loads(line)
                            pacchetti.append(pacchetto)
                        except json.JSONDecodeError as e:
                            print(f"⚠️  Errore parsing JSON: {e}")
                            continue
        except FileNotFoundError:
            print(f"❌ File non trovato: {self.data_file}")
            return None
        
        return pacchetti
    
    def simula(self):
        """Legge e invia i dati con timing originale"""
        pacchetti = self.leggi_dati()
        if not pacchetti:
            sys.exit(1)
        
        print("\n" + "═"*60)
        print("  🎬 SIMULATORE TELEMETRIA KART")
        print("═"*60)
        print(f"\n📁 File: {self.data_file}")
        print(f"📊 Pacchetti: {len(pacchetti)}")
        print(f"📡 Server: {self.server_ip}:{self.server_port}")
        print(f"⏱️  Velocità: {self.speed}x")
        print(f"⏳ Durata: ~{len(pacchetti)/10/self.speed:.1f}s (@ 10 Hz)\n")
        
        timestamps = []
        try:
            for i, pacchetto in enumerate(pacchetti):
                # Calcola il tempo da attendere
                if i == 0:
                    ultimo_ts = datetime.fromisoformat(pacchetto['timestamp'])
                    timestamps.append(ultimo_ts)
                else:
                    ts_attuale = datetime.fromisoformat(pacchetto['timestamp'])
                    delta = (ts_attuale - timestamps[-1]).total_seconds()
                    timestamps.append(ts_attuale)
                    
                    # Attendi con fattore di velocità
                    wait_time = delta / self.speed
                    time.sleep(wait_time)
                
                # Invia il pacchetto
                json_data = json.dumps(pacchetto)
                self.sock.sendto(json_data.encode('utf-8'), (self.server_ip, self.server_port))
                self.pacchetti_inviati += 1
                
                # Print status
                if self.pacchetti_inviati % 10 == 0:
                    gps = pacchetto.get('gps', {})
                    accel = pacchetto.get('accelerometer', {})
                    gps_str = ""
                    if gps.get('latitude'):
                        gps_str = f"GPS: {gps['latitude']:.5f},{gps['longitude']:.5f} | "
                    
                    print(f"📡 Inviati {self.pacchetti_inviati} pacchetti | "
                          f"G: Lat {accel.get('gx', 0):+.2f} Long {accel.get('gy', 0):+.2f} | "
                          f"{gps_str}"
                          f"V: {gps.get('speed_kmh', 0):.1f} km/h")
        
        except KeyboardInterrupt:
            print("\n\n⏹️  Simulazione interrotta")
        
        except Exception as e:
            print(f"❌ Errore durante la simulazione: {e}")
        
        finally:
            print("\n" + "═"*60)
            print("  ✅ SIMULAZIONE COMPLETATA")
            print("═"*60)
            print(f"\n  Pacchetti inviati: {self.pacchetti_inviati}")
            if len(pacchetti) > 0:
                durata = (timestamps[-1] - timestamps[0]).total_seconds()
                print(f"  Durata traccia: {int(durata//60)}m {int(durata%60)}s")
            print("\n" + "═"*60 + "\n")
            self.sock.close()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Simulatore Telemetria Kart")
    parser.add_argument("file", nargs="?", help="File JSONL con dati (default: ultimo file creato)")
    parser.add_argument("--server-ip", default="193.70.113.55", help="IP del server (default: 193.70.113.55)")
    parser.add_argument("--server-port", type=int, default=5005, help="Porta del server (default: 5005)")
    parser.add_argument("--speed", type=float, default=1.0, help="Velocità di riproduzione (default: 1.0)")
    
    args = parser.parse_args()
    
    # Se non specificato, trova l'ultimo file data_log
    data_file = args.file
    if not data_file:
        data_dir = "/home/pi/testIMU"
        data_files = [f for f in os.listdir(data_dir) if f.startswith("data_log_") and f.endswith(".jsonl")]
        if not data_files:
            print("❌ Nessun file data_log trovato in /home/pi/testIMU")
            sys.exit(1)
        data_files.sort()
        data_file = os.path.join(data_dir, data_files[-1])
        print(f"📁 Usando ultimo file: {data_file}")
    
    simulator = TelemetriaSimulator(
        data_file=data_file,
        server_ip=args.server_ip,
        server_port=args.server_port,
        speed=args.speed
    )
    
    simulator.simula()


if __name__ == "__main__":
    main()


