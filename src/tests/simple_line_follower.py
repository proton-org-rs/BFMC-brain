# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
# Example: Simple robot control - drive straight until stop line detected

"""
PRIMER IMPLEMENTACIJE LOGIČKE KONTROLE ROBOTA
==============================================

Ovaj modul demonstrira jednostavan primer upravljanja robotom:
- Robot ide pravo brzinom 20 cm/s
- Kada detektuje "stop liniju" (simulirano), zaustavlja se

Može se pokrenuti samostalno (simulacija) ili integrisati u glavni sistem.
"""

import logging
import sys
import os
import time
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Callable
from multiprocessing import Queue, Event

# Ensure Brain package root is in sys.path when running from workspace root
if os.path.basename(os.getcwd()) != 'Brain':
    brain_root = os.path.join(os.getcwd(), 'src', 'Brain')
    if os.path.isdir(brain_root):
        os.chdir(brain_root)
sys.path.append('.')

# ========================= STANJA ROBOTA =========================

class RobotState(Enum):
    """Definicija mogućih stanja robota."""
    IDLE = "idle"               # Robot miruje
    DRIVING = "driving"         # Robot vozi
    STOP_LINE_DETECTED = "stop_line_detected"  # Detektovana stop linija
    STOPPED = "stopped"         # Robot zaustavljen


# ========================= KOMANDE ROBOTA =========================

@dataclass
class MotorCommand:
    """Struktura za motor komande."""
    speed: float = 0.0      # Brzina u cm/s (pozitivna = napred, negativna = nazad)
    steer: float = 0.0      # Ugao upravljača u stepenima (-21 do +21)
    
    def __str__(self):
        return f"Speed: {self.speed} cm/s, Steer: {self.steer}°"


# ========================= IR SENZOR DETEKTOR =========================

class RealStopLineDetector:
    """
    Detektor stop linije korišćenjem IR senzora na NUCLEO.
    Čita StopLine poruke iz pipeline-a.
    """
    
    def __init__(self, queueList):
        """
        Args:
            queueList: Dictionary queues iz BFMC sistema
        """
        from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
        from src.utils.messages.allMessages import StopLine
        
        self.stopLineSubscriber = messageHandlerSubscriber(
            queueList,
            StopLine,
            "lastOnly",
            True
        )
        
        self._detected = False
        self._detection_time = None
        
    def update(self, dt: float):
        """
        Ažurira stanje detektora čitanjem iz IR senzora.
        
        Args:
            dt: Delta time u sekundama (nekorišćeno ali zadržano za kompatibilnost)
        """
        # Čita Stop liniju iz pipeline-a (non-blocking)
        # NUCLEO šalje @stopLine:1;; ili @stopLine:0;;
        stop_line_data = self.stopLineSubscriber.receive()
        
        if stop_line_data is not None:
            # Parse vrednost - NUCLEO šalje string "1" ili "0"
            is_detected = str(stop_line_data).strip() == "1" or stop_line_data == True
            self._detected = is_detected
            if self._detected:
                self._detection_time = time.time()
            
    def is_stop_line_detected(self) -> bool:
        """Vraća True ako je detektovana stop linija od IR senzora."""
        return self._detected
    
    def get_detection_time(self) -> Optional[float]:
        """Vraća vremensku marku kada je linija detektovana."""
        return self._detection_time
    
    def reset(self):
        """Resetuje detektor za novu vožnju."""
        self._detected = False
        self._detection_time = None


# ========================= SIMULIRANI SENZORI =========================

class SimulatedStopLineDetector:
    """
    Simulirani detektor stop linije.
    Koristi se samo u test/simulaciji modu bez hardvera.
    """
    
    def __init__(self, detection_distance: float = 100.0):
        """
        Args:
            detection_distance: Simulirana distanca (cm) na kojoj će se detektovati stop linija
        """
        self.detection_distance = detection_distance
        self.traveled_distance = 0.0
        self._detected = False
        
    def update(self, speed: float, dt: float):
        """
        Ažurira simulaciju na osnovu brzine i proteklog vremena.
        
        Args:
            speed: Trenutna brzina robota (cm/s)
            dt: Delta time u sekundama
        """
        self.traveled_distance += abs(speed) * dt
        
        if self.traveled_distance >= self.detection_distance:
            self._detected = True
            
    def is_stop_line_detected(self) -> bool:
        """Vraća True ako je detektovana stop linija."""
        return self._detected
    
    def reset(self):
        """Resetuje detektor za novu vožnju."""
        self.traveled_distance = 0.0
        self._detected = False


# ========================= REALNI MOTOR KONTROLER =========================

class RealMotorController:
    """
    Realni motor kontroler koji šalje komande preko BFMC message sistema.
    Uključuje podršku za aktivaciju IR senzora za detekciju stop linije.
    """
    
    def __init__(self, queueList):
        """
        Args:
            queueList: Dictionary queues iz BFMC sistema
        """
        from src.utils.messages.messageHandlerSender import messageHandlerSender
        from src.utils.messages.allMessages import SpeedMotor, SteerMotor, Klem, Brake, ToggleStopLine
        
        self.speedSender = messageHandlerSender(queueList, SpeedMotor)
        self.steerSender = messageHandlerSender(queueList, SteerMotor)
        self.klemSender = messageHandlerSender(queueList, Klem)
        self.brakeSender = messageHandlerSender(queueList, Brake)
        self.toggleStopLineSender = messageHandlerSender(queueList, ToggleStopLine)
        
        self.current_speed = 0.0
        self.current_steer = 0.0
        self._enabled = False
        self._ir_sensor_enabled = False
        
    def enable(self):
        """Omogućava motor (KL30)."""
        self._enabled = True
        self.klemSender.send("30")
        print("[Motor] Engine ENABLED (KL30)")
        
    def enable_ir_sensor(self):
        """Aktivira IR senzor za detekciju stop linije na NUCLEO."""
        self.toggleStopLineSender.send(1)
        self._ir_sensor_enabled = True
        print("[Motor] IR Sensor ENABLED (stopLine:1)")
        
    def disable_ir_sensor(self):
        """Deaktivira IR senzor za detekciju stop linije."""
        self.toggleStopLineSender.send(0)
        self._ir_sensor_enabled = False
        print("[Motor] IR Sensor DISABLED (stopLine:0)")
        
    def disable(self):
        """Onemogućava motor (KL0) i IR senzor."""
        self._enabled = False
        self.current_speed = 0.0
        self.current_steer = 0.0
        
        if self._ir_sensor_enabled:
            self.disable_ir_sensor()
            
        self.klemSender.send("0")
        print("[Motor] Engine DISABLED (KL0)")
        
    def set_speed(self, speed: float):
        """
        Postavlja brzinu motora.
        
        Args:
            speed: Brzina u cm/s (-30 do +30 za ovaj auto)
        """
        if not self._enabled:
            print("[Motor] WARNING: Engine not enabled!")
            return
            
        # Ograniči brzinu na dozvoljeni opseg
        speed = max(-30.0, min(30.0, speed))
        self.current_speed = speed
        self.speedSender.send(int(speed))
        print(f"[Motor] Speed set to: {speed:.1f} cm/s")
        
    def set_steer(self, angle: float):
        """
        Postavlja ugao upravljača.
        
        Args:
            angle: Ugao u stepenima (-21 do +21)
        """
        if not self._enabled:
            print("[Motor] WARNING: Engine not enabled!")
            return
            
        # Ograniči ugao na dozvoljeni opseg
        angle = max(-21.0, min(21.0, angle))
        self.current_steer = angle
        self.steerSender.send(int(angle))
        print(f"[Motor] Steer set to: {angle:.1f}°")
        
    def brake(self):
        """Kočenje - zaustavlja motor koristeći Brake komandu."""
        self.current_speed = 0.0
        self.brakeSender.send("0")  # Emergency brake
        self.speedSender.send(0)
        print("[Motor] BRAKE applied!")
        
    def is_enabled(self) -> bool:
        return self._enabled


# ========================= SIMULIRANI MOTOR KONTROLER =========================

class SimulatedMotorController:
    """
    Simulirani motor kontroler.
    U stvarnom sistemu, ovo bi slalo komande preko serial handlera na NUCLEO.
    """
    
    def __init__(self):
        self.current_speed = 0.0
        self.current_steer = 0.0
        self._enabled = False
        
    def enable(self):
        """Omogućava motor (KL30)."""
        self._enabled = True
        print("[Motor] Engine ENABLED (KL30)")
        
    def disable(self):
        """Onemogućava motor (KL0)."""
        self._enabled = False
        self.current_speed = 0.0
        self.current_steer = 0.0
        print("[Motor] Engine DISABLED (KL0)")
        
    def set_speed(self, speed: float):
        """
        Postavlja brzinu motora.
        
        Args:
            speed: Brzina u cm/s (-30 do +30 za ovaj auto)
        """
        if not self._enabled:
            print("[Motor] WARNING: Engine not enabled!")
            return
            
        # Ograniči brzinu na dozvoljeni opseg
        speed = max(-30.0, min(30.0, speed))
        self.current_speed = speed
        print(f"[Motor] Speed set to: {speed:.1f} cm/s")
        
    def set_steer(self, angle: float):
        """
        Postavlja ugao upravljača.
        
        Args:
            angle: Ugao u stepenima (-21 do +21)
        """
        if not self._enabled:
            print("[Motor] WARNING: Engine not enabled!")
            return
            
        # Ograniči ugao na dozvoljeni opseg
        angle = max(-21.0, min(21.0, angle))
        self.current_steer = angle
        print(f"[Motor] Steer set to: {angle:.1f}°")
        
    def brake(self):
        """Kočenje - zaustavlja motor."""
        self.current_speed = 0.0
        print("[Motor] BRAKE applied!")
        
    def is_enabled(self) -> bool:
        return self._enabled


# ========================= GLAVNI KONTROLER ROBOTA =========================

class SimpleRobotController:
    """
    Jednostavan kontroler robota koji implementira logiku:
    - Vozi pravo zadatom brzinom
    - Zaustavi se kada detektuje stop liniju (IR senzor ili simulacija)
    """
    
    def __init__(self, 
                 motor_controller,
                 stop_line_detector,
                 target_speed: float = 20.0):
        """
        Args:
            motor_controller: Motor controller instance (Real ili Simulated)
            stop_line_detector: Stop line detector (Real IR senzor ili Simulated)
            target_speed: Željena brzina vožnje u cm/s
        """
        self.target_speed = target_speed
        
        # Komponente
        self.motor = motor_controller
        self.stop_line_detector = stop_line_detector
        
        # Stanje
        self.state = RobotState.IDLE
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        
        # Timing
        self.control_rate = 50  # Hz
        self.dt = 1.0 / self.control_rate
        
    def start(self):
        """Pokreće kontrolnu petlju robota."""
        if self._running:
            print("[Controller] Already running!")
            return
            
        print("\n" + "="*60)
        print("POKRETANJE ROBOTA")
        print(f"Ciljna brzina: {self.target_speed} cm/s")
        print("Čekam IR senzor da detektuje stop liniju...")
        print("="*60 + "\n")
        
        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        
    def stop(self):
        """Zaustavlja kontrolnu petlju robota."""
        self._running = False
        if self._control_thread:
            self._control_thread.join(timeout=1.0)
        self.motor.disable()
        print("[Controller] Stopped")
        
    def _control_loop(self):
        """
        Glavna kontrolna petlja robota.
        Implementira logiku upravljanja baziranu na stanjima.
        """
        
        # Početno stanje - IDLE
        self.state = RobotState.IDLE
        
        while self._running:
            start_time = time.time()
            
            # State machine logika
            self._process_state()
            
            # Kontrola timing-a
            elapsed = time.time() - start_time
            sleep_time = max(0, self.dt - elapsed)
            time.sleep(sleep_time)
            
    def _process_state(self):
        """
        Procesira trenutno stanje i prelazi u sledeće ako je potrebno.
        Ovo je srce logičke kontrole robota.
        """
        
        if self.state == RobotState.IDLE:
            # Prelaz: IDLE -> DRIVING
            print("\n[State] IDLE -> Inicijalizacija...")
            self.motor.enable()
            time.sleep(0.1)  # Mali delay za stabilizaciju
            
            # Aktiviraj IR senzor za detekciju stop linije (ako je realni motor)
            if hasattr(self.motor, 'enable_ir_sensor'):
                self.motor.enable_ir_sensor()
                time.sleep(0.2)  # Čekaj da se IR senzor aktivira
            
            # Postavi brzinu i kreni
            self.motor.set_speed(self.target_speed)
            self.motor.set_steer(0)  # Pravo
            
            self.state = RobotState.DRIVING
            print("[State] Prelaz na DRIVING")
            
        elif self.state == RobotState.DRIVING:
            # Ažuriraj senzore - čita IR senzor sa NUCLEO
            if isinstance(self.stop_line_detector, RealStopLineDetector):
                self.stop_line_detector.update(self.dt)
            else:
                # Simulacija - koristi brzinu
                self.stop_line_detector.update(self.motor.current_speed, self.dt)
            
            # Proveri uslove prelaza
            if self.stop_line_detector.is_stop_line_detected():
                print(f"\n[Sensor] 🛑 STOP LINIJA DETEKTOVANA OD IR SENZORA!")
                if isinstance(self.stop_line_detector, RealStopLineDetector):
                    detection_time = self.stop_line_detector.get_detection_time()
                    if detection_time:
                        print(f"[Sensor] Vreme detekcije: {detection_time}")
                self.state = RobotState.STOP_LINE_DETECTED
            else:
                # Održavaj vožnju (u stvarnom sistemu ovde bi bila PID kontrola)
                pass
                
        elif self.state == RobotState.STOP_LINE_DETECTED:
            # Prelaz: STOP_LINE_DETECTED -> STOPPED
            print("[State] Zaustavljanje vozila...")
            self.motor.brake()
            self.motor.set_speed(0)
            
            self.state = RobotState.STOPPED
            print("[State] Prelaz na STOPPED")
            
        elif self.state == RobotState.STOPPED:
            # Robot je zaustavljen - završi petlju
            print("\n[State] Robot zaustavljen. Završavam kontrolnu petlju.")
            self._running = False
            

# ========================= TESTIRANJE I MAIN =========================

def run_with_real_hardware():
    """
    Pokreće kontroler sa realnim hardverom preko BFMC message sistema.
    IR senzor je direktno povezan na NUCLEO i šalje StopLine poruke.
    """
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("line_follower")
    
    print("\n" + "="*60)
    print("BFMC ROBOT LINE FOLLOWER - IR SENZOR MODE")
    print("Scenario: Vožnja pravo dok IR senzor ne detektuje stop liniju")
    print("="*60)
    
    # Setup queues
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
        "Log": Queue(),
    }
    
    # Import BFMC processes
    from src.gateway.processGateway import processGateway
    from src.hardware.serialhandler.processSerialHandler import processSerialHandler
    
    # Ready events
    serial_ready = Event()
    dashboard_ready = Event()
    dashboard_ready.set()
    
    # Start processes
    gw = processGateway(queueList, logger)
    sh = processSerialHandler(queueList, logger, serial_ready, dashboard_ready, debugging=False)
    
    gw.daemon = True
    sh.daemon = True
    
    gw.start()
    sh.start()
    
    # Wait for serial handler
    serial_ready.wait(timeout=5)
    time.sleep(0.5)  # Additional stabilization
    
    # Create real motor controller
    motor = RealMotorController(queueList)
    
    # Create REAL IR sensor stop line detector
    stop_line_detector = RealStopLineDetector(queueList)
    
    # Get parameters
    try:
        speed = float(input("Unesite brzinu (cm/s) [default=20]: ") or "20")
    except (ValueError, EOFError):
        speed = 20.0
    
    # Create and start controller
    controller = SimpleRobotController(
        motor_controller=motor,
        stop_line_detector=stop_line_detector,
        target_speed=speed
    )
    
    controller.start()
    
    # Wait for completion
    while controller._running:
        time.sleep(0.1)
    
    time.sleep(0.5)
    controller.stop()
    
    print("\n" + "="*60)
    print("VOŽNJA ZAVRŠENA")
    print("Robot se zaustavio jer je IR senzor detektovao stop liniju")
    print("="*60)
    
    # Shutdown processes
    sh.stop()
    gw.stop()
    sh.join(timeout=2)
    gw.join(timeout=2)


def run_simulation():
    """
    Pokreće simulaciju robota sa simuliranom stop line detekcijom.
    Koristi se kada IR senzor nije dostupan (test/razvoj).
    """
    
    print("\n" + "="*60)
    print("BFMC ROBOT CONTROL SIMULATION (No hardware, simulated IR sensor)")
    print("Scenario: Vožnja pravo do simulirane stop linije")
    print("="*60)
    
    # Create simulated motor
    motor = SimulatedMotorController()
    
    # Create SIMULATED stop line detector
    stop_line_detector = SimulatedStopLineDetector(detection_distance=100.0)
    
    # Get parameters
    try:
        speed = float(input("Unesite brzinu (cm/s) [default=20]: ") or "20")
        distance = float(input("Unesite distancu do stop linije (cm) [default=100]: ") or "100")
    except (ValueError, EOFError):
        speed = 200.0
        distance = 100.0
    
    # Update detector distance
    stop_line_detector.detection_distance = distance
    
    # Create controller
    controller = SimpleRobotController(
        motor_controller=motor,
        stop_line_detector=stop_line_detector,
        target_speed=speed
    )
    
    # Start
    controller.start()
    
    # Wait for completion
    while controller._running:
        time.sleep(0.1)
        
    time.sleep(0.5)
    controller.stop()
    
    print("\n" + "="*60)
    print("SIMULACIJA ZAVRŠENA")
    print(f"Ukupna pređena distanca: {stop_line_detector.traveled_distance:.1f} cm")
    print("="*60)


# ========================= MAIN =========================

if __name__ == "__main__":
    print("\nOdaberite mod:")
    print("1. Realni hardver (sa NUCLEO komunikacijom)")
    print("2. Simulacija (bez hardvera)")
    
    choice = input("\nIzbor (1/2) [default=1]: ") or "1"
    
    if choice == "2":
        run_simulation()
    else:
        run_with_real_hardware()
