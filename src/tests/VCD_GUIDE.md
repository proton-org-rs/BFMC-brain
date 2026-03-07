# VCD (Velocity Control Duration) - Dashboard Integration Guide

## Problem Analysis

Originalna skripta je korišćala **pogrešan pristup** slanju komandi:
- ❌ Slala je `SpeedMotor` i `SteerMotor` kao **stringove**
- ❌ Nije slala `Time` parametar
- ❌ Vremenske vrednosti nisu bile pomnožene sa 10

## Kako Dashboard Funkcioniše

Dashboard komponenta (`time-speed-steer.component.ts`) šalje komande ovako:

```typescript
this.webSocketService.sendMessageToFlask(
  `{"Name": "Control", "Value": {"Time":"${this.time * 10}","Speed":"${this.speed * 10}","Steer":"${limitedSteer * 10}"}}`
);
```

**Ključne karakteristike:**
- Koristi **`Control` poruku** (`msgType = "dict"`)
- Šalje **sve vrednosti pomnožene sa 10**
- Struktura: `{"Time": "XX", "Speed": "YY", "Steer": "ZZ"}`

## Kako `threadWrite` Procesira Komande

Kad `Control` poruka stigne u `threadWrite` ([threadWrite.py](c:\Users\Korisnik\Documents\Projekti\BFMC\Proton-ETF1-BFMC2026\src\Brain\src\hardware\serialhandler\threads\threadWrite.py#L224-L236)):

```python
controlRecv = self.controlSubscriber.receive()
if controlRecv is not None:
    command = {
        "action": "vcd",
        "time": int(controlRecv["Time"]),
        "speed": int(controlRecv["Speed"]),
        "steer": int(controlRecv["Steer"]),
    }
    self.send_to_serial(command)
```

To se šalje firmware-u kao `vcd` komanda sa vrednostima koje su **već pomnožene sa 10**.

## Ispravljena Skripta

### Imports
```python
from src.utils.messages.allMessages import Klem, Control
```

### Senders
```python
klem_sender = messageHandlerSender(queueList, Klem)
control_sender = messageHandlerSender(queueList, Control)
```

### Slanje Komande
```python
# Enable engine
klem_sender.send("30")
time.sleep(1.0)

# Send VCD control (multiplied by 10, like dashboard does)
control_sender.send({
    "Time": str(5 * 10),      # 5 seconds
    "Speed": str(20 * 10),    # 20 cm/s
    "Steer": str(0 * 10)      # 0 degrees
})

# Wait for execution
time.sleep(6.0)  # 5s driving + 1s margin

# Disable engine
klem_sender.send("0")
```

## Parametri VCD Komande

| Parametar | Opseg | Opis | Faktor |
|-----------|-------|------|--------|
| **Time** | 0-300+ | Trajanje kretanja u sekundama | ×10 |
| **Speed** | -50 do 50 | Brzina u cm/s (negativno = unazad) | ×10 |
| **Steer** | -25 do 25 | Ugao skretanja u stepenima | ×10 |

## Primeri

### Primer 1: Idem napred 3s na 20 cm/s
```python
control_sender.send({
    "Time": str(3 * 10),      # "30"
    "Speed": str(20 * 10),    # "200"
    "Steer": str(0 * 10)      # "0"
})
time.sleep(4.0)  # 3s + margin
```

### Primer 2: Skretam udesno 2s na 15 cm/s
```python
control_sender.send({
    "Time": str(2 * 10),      # "20"
    "Speed": str(15 * 10),    # "150"
    "Steer": str(10 * 10)     # "100" (desno je pozitivno)
})
time.sleep(3.0)  # 2s + margin
```

### Primer 3: Idem unazad 1s na -10 cm/s
```python
control_sender.send({
    "Time": str(1 * 10),      # "10"
    "Speed": str(-10 * 10),   # "-100"
    "Steer": str(0 * 10)      # "0"
})
time.sleep(2.0)  # 1s + margin
```

## Razlika: SpeedMotor vs Control

### ❌ POGREŠNO (stari pristup):
```python
speed_sender = messageHandlerSender(queueList, SpeedMotor)
steer_sender = messageHandlerSender(queueList, SteerMotor)

speed_sender.send("20")     # Samo brzina
steer_sender.send("0")      # Samo smer
```

**Problem:** Firmware ne zna trajanje, nema vremenskog ograničenja, vrednosti nisu 100% precizne.

### ✅ ISPRAVNO (dashboard pristup):
```python
control_sender = messageHandlerSender(queueList, Control)

control_sender.send({
    "Time": str(5 * 10),    # Vremensko ograničenje
    "Speed": str(20 * 10),  # Precizna brzina
    "Steer": str(0 * 10)    # Precizna kieracija
})
```

**Prednosti:**
- Vremensko ograničenje (firmware zna kada da stane)
- Sve vrednosti su pomnožene sa 10 (preciznije)
- Isti format kao dashboard UI
- Automatski se osigurava da je vreme isteklo (ne zavisi od klijenta)

## Dostupne Skripte

1. **`run_forward_until_stop.py`** - Osnovna verzija, jednostavna
2. **`run_forward_until_stop_debug.py`** - Sa detaljnim debug outputom
3. **`run_control_test.py`** - Testira 5 različitih VCD scenarija

## Testiranje

```bash
cd src/Brain
python ../examples/run_forward_until_stop.py
```

Očekivani rezultat: Vozilo bi trebalo da ide napred 5 sekundi na 20 cm/s, pa da stane.
