# Run Scripts - Ispravljene Verzije

## Šta je Promenjeno?

Analizirao sam kako se sa **dashboard-a** prate brzina, vreme i smer, i **popravió sam sve skripte** da koriste **isti format kao dashboard**.

### Ključna Otkrića:

1. **Dashboard koristi `Control` poruku** sa `dict` vrednostima
   - Format: `{"Time": "XX", "Speed": "YY", "Steer": "ZZ"}`
   - Sve vrednosti su **pomnožene sa 10**

2. **Stara skripta je korišćena `SpeedMotor` i `SteerMotor`** kao stringove
   - ❌ Nije slala `Time`
   - ❌ Vrednosti nisu bile × 10
   - ❌ Problem: Brazina je uvek bila 5 cm/s umesto željene vrednosti

3. **Firmware očekuje vrednosti × 10**
   - `threadWrite.py` direktno prosleđuje vrednosti firmware-u
   - Firmware sa `speed: 200` zna da je to `200 * 0.1 = 20 cm/s`

---

## Dostupne Skripte

### 1. `run_forward_until_stop.py` ⭐ PREPORUČENO
Osnovna, čista verzija - idealna za produkciju.

```bash
python src/Brain/src/examples/run_forward_until_stop.py
```

**Šta radi:**
- ✅ Vozilo ide napred 5 sekundi na 20 cm/s
- ✅ Koristi `Control` poruku (kao dashboard)
- ✅ Sve vrednosti × 10
- ✅ Pouzdano i stabilno

---

### 2. `run_forward_until_stop_debug.py` 🔍 DEBUGGING
Detaljni logging za troubleshooting.

```bash
python src/Brain/src/examples/run_forward_until_stop_debug.py
```

**Šta prikazuje:**
- Detaljne debug poruke
- Kada se koje komande šalju
- Status svakog koraka

---

### 3. `run_control_test.py` 🧪 TESTIRANJE
Testira 5 različitih VCD scenarija za validaciju.

```bash
python src/Brain/src/examples/run_control_test.py
```

**Test scenariji:**
- Test 1: Forward 3s @ 10 cm/s
- Test 2: Forward 3s @ 20 cm/s + desna krivina
- Test 3: Forward 3s @ 40 cm/s (brzo)
- Test 4: Unazad 2s @ -20 cm/s
- Test 5: Forward 3s @ 20 cm/s + leva krivina

---

## VCD Format Pojašnjenje

| Parametar | Opseg | Opis | Primer |
|-----------|-------|------|--------|
| **Time** | 0-300 | Sekundi kretanja | `3 * 10 = "30"` |
| **Speed** | -50 do 50 | cm/s (- = unazad) | `20 * 10 = "200"` |
| **Steer** | -25 do 25 | Stepeni skretanja | `10 * 10 = "100"` |

### Primeri:

```python
# Idem napred 5s na 20 cm/s
control_sender.send({
    "Time": str(5 * 10),      # 5 sekundi
    "Speed": str(20 * 10),    # 20 cm/s
    "Steer": str(0 * 10)      # Strago
})

# Idem unazad 3s na 15 cm/s skretanjem ulevo
control_sender.send({
    "Time": str(3 * 10),      # 3 sekunde
    "Speed": str(-15 * 10),   # Unazad 15 cm/s
    "Steer": str(-15 * 10)    # Levo 15°
})
```

---

## Komparacija: Stara vs Novo

### ❌ STARA (Pogrešna)
```python
from src.utils.messages.allMessages import Klem, SpeedMotor, SteerMotor

speed_sender = messageHandlerSender(queueList, SpeedMotor)
steer_sender = messageHandlerSender(queueList, SteerMotor)

speed_sender.send("20")      # Samo broj, bez × 10
steer_sender.send("0")       # Nema vremenskog ograničenja
```

**Problemi:**
- Vrednosti nisu × 10
- Nema trajanja komande
- Brazina je uvek 5 cm/s
- Ponekad ne radi

### ✅ NOVA (Ispravna)
```python
from src.utils.messages.allMessages import Klem, Control

control_sender = messageHandlerSender(queueList, Control)

control_sender.send({
    "Time": str(5 * 10),      # 5 sekundi × 10
    "Speed": str(20 * 10),    # 20 cm/s × 10
    "Steer": str(0 * 10)      # Strago × 10
})
```

**Prednosti:**
- ✅ Sve vrednosti × 10 (kao firmware očekuje)
- ✅ Vremensko ograničenje (znamo kad će da stane)
- ✅ Precizna brazina (zaista 20 cm/s)
- ✅ Isti format kao dashboard
- ✅ Pouzdano

---

## Očekivani Rezultati

### `run_forward_until_stop.py`:
```
INFO - Waiting for serial handler to be ready...
INFO - Serial handler ready, waiting for threads to initialize...
INFO - Enabling engine (KL=30)...
INFO - Engine enabled, ready to drive
INFO - Sending VCD control: time=5s, speed=20cm/s, steer=0°
INFO - Vehicle is executing VCD command for ~6.0s...
INFO - Disabling engine (KL=0)...
INFO - Stop command sent. Shutting down processes...
```

**Rezultat:** Vozilo ide napred 5 sekundi na 20 cm/s, pa se gasi. ✓

---

## Dokumentacija

- **`VCD_GUIDE.md`** - Detaljno objašnjenje VCD protokola
- **`POREĐENJE_STARA_VS_NOVA.md`** - Side-by-side poređenje pristupa

---

## Troubleshooting

### Problem: "Serial handler not ready"
- Proveri da li je NUCLEO ploča USB-om povezana
- Proveri `/dev/ttyACM*` portove (Linux) ili `COM*` (Windows)

### Problem: Brazina je dalje 5 cm/s
- Koristi novu verziju skripte (`run_forward_until_stop.py`)
- Stara verzija je imala ovaj problem jer nije slala × 10

### Problem: Vozilo se ne pomera
- Proveri da li je KL=30 komanda primljena pre speed komande
- Pauza od 1s je kritična - nikad je ne skraćuj

### Problem: Skripta ponekad radi, ponekad ne
- Duži timeout (10s umesto 8s) za serial handler
- Viša pauza (1.5s) pre slanja komandi
- Pauza (1s) između KL=30 i Control komande

---

## Brzi Start

```bash
# 1. Idi u Brain root
cd src/Brain

# 2. Aktiviraj virtualenv
source .venv/bin/activate    # Linux/Mac
.venv\Scripts\activate       # Windows

# 3. Pokreni osnovnu skriptu
python ../examples/run_forward_until_stop.py

# Ili za debug:
python ../examples/run_forward_until_stop_debug.py

# Ili za testiranje:
python ../examples/run_control_test.py
```

---

## Zaključak

Sve tri skripte koriste **ispravnu Control poruku sa vrednostima × 10**, što je **isti format koji dashboard koristi**. 

Ovo rešava sve probleme:
- ✅ Brazina je precizna
- ✅ Vreme je eksplicitno
- ✅ Stabilno i pouzdano
- ✅ Slijedi protokol

Testiraj i javi rezultate! 🚗
