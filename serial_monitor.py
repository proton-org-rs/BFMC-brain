#!/usr/bin/env python3
"""
Serial Monitor Script
Prikazuje sve podatke koji dolaze sa serial porta (npr. sa NUCLEO-a)
"""

import serial
import serial.tools.list_ports
import sys
import time
from datetime import datetime

def list_available_ports():
    """Prikazuje sve dostupne serial portove"""
    ports = serial.tools.list_ports.comports()
    available_ports = []
    
    print("\n" + "="*70)
    print("DOSTUPNI SERIJSKI PORTOVI:")
    print("="*70)
    
    if not ports:
        print("❌ Nema dostupnih portova!")
        return available_ports
    
    for i, port in enumerate(ports, 1):
        print(f"{i}. {port.device} - {port.description}")
        available_ports.append(port.device)
    
    print("="*70 + "\n")
    return available_ports

def select_port(available_ports):
    """Dozvoljava korisniku da bira port"""
    while True:
        try:
            choice = input("Unesite redni broj porta (ili 'q' za izlaz): ").strip()
            
            if choice.lower() == 'q':
                print("Izlaz...")
                sys.exit(0)
            
            port_index = int(choice) - 1
            if 0 <= port_index < len(available_ports):
                return available_ports[port_index]
            else:
                print("❌ Neispravan izbor. Pokušajte ponovo.")
        except ValueError:
            print("❌ Molim unesite broj ili 'q'.")

def open_serial_connection(port, baudrate=115200):
    """Otvara serijski port sa zadatim brzinama"""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ Povezan na {port} sa brzinom {baudrate} baud")
        return ser
    except serial.SerialException as e:
        print(f"❌ Greška pri otvaranju porta {port}: {e}")
        return None

def print_header():
    """Prikazuje zaglavlje"""
    print("\n" + "="*70)
    print("SERIJSKI MONITOR - PRIKAZ SVIH PODATAKA")
    print("="*70)
    print("Pritisnite Ctrl+C za izlaz")
    print("="*70 + "\n")

def monitor_serial(port, baudrate=115200):
    """Čita i prikazuje sve sa serial porta"""
    ser = open_serial_connection(port, baudrate)
    
    if not ser:
        return
    
    print_header()
    
    try:
        while True:
            if ser.in_waiting > 0:
                try:
                    # Čitaj liniju
                    line = ser.readline().decode('utf-8', errors='replace').rstrip('\n\r')
                    
                    if line:
                        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        print(f"[{timestamp}] {line}")
                        sys.stdout.flush()
                
                except UnicodeDecodeError as e:
                    print(f"❌ Greška pri dekodiranju: {e}")
                except Exception as e:
                    print(f"❌ Greška pri čitanju: {e}")
            
            time.sleep(0.01)  # Mali delay da ne trošimo CPU
    
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("Prekid korisnika - zatvaranje...")
        print("="*70)
    
    except Exception as e:
        print(f"\n❌ Neočekivana greška: {e}")
    
    finally:
        if ser and ser.is_open:
            ser.close()
            print("✓ Serial port zatvoren")

def main():
    """Glavna funkcija"""
    print("\n" + "="*70)
    print("SERIAL MONITOR - ČITAČ SERIJSKIH PORTOVA")
    print("="*70)
    
    # Prikazuj dostupne portove
    available_ports = list_available_ports()
    
    if not available_ports:
        print("❌ Nema dostupnih portova.")
        print("Provjerite:")
        print("  - Je li NUCLEO povezan putem USB-a?")
        print("  - Je li USB driver instaliran?")
        return
    
    # Dozvoli korisniku da bira port
    selected_port = select_port(available_ports)
    
    # Optionalno prikaži izbor baudrate-a
    print(f"\nKoristiću standardnu brzinu: 115200 baud")
    print("(Promijenite u kodu ako trebate drugu brzinu)")
    
    # Počni monitoring
    monitor_serial(selected_port, baudrate=115200)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n❌ Kritična greška: {e}")
        sys.exit(1)
