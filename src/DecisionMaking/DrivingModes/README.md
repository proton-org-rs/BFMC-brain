# DrivingModes — Sistem modova vožnje

## Pregled

`DrivingModes` paket definiše sve moguće **režime (modove) vožnje** autonomnog vozila u okviru BFMC takmičenja. Sistem je realizovan kao **state machine** (mašina stanja) koja omogućava čistu tranziciju između modova na osnovu detekcija sa kamere (YOLO model), senzorskih podataka i konteksta mape (PathPlanning).

Ovaj paket koristi `threadDecisionMaking` — centralna nit koja prima detekcije i prelazi između modova, čime upravlja ponašanjem vozila (brzina, skretanje, kočenje).

## Arhitektura

```
  YOLO Detekcija          PathPlanning (mapa)
       │                        │
       ▼                        ▼
┌─────────────────────────────────────────┐
│          threadDecisionMaking           │
│                                         │
│   ┌─────────────────────────────────┐   │
│   │       DrivingModeContext        │   │
│   │                                 │   │
│   │  current_mode: DrivingMode      │   │
│   │  previous_mode: DrivingMode     │   │
│   │  mode_data: dict                │   │
│   │  mode_start_time: float         │   │
│   └─────────────────────────────────┘   │
│                  │                      │
│                  ▼                      │
│         Motor Control                   │
│    (SpeedMotor, SteerMotor, Brake)      │
└─────────────────────────────────────────┘
```

## Struktura fajlova

| Fajl | Opis |
|------|------|
| `__init__.py` | Inicijalizacija paketa |
| `drivingMode.py` | `DrivingMode` enum + `DrivingModeContext` klasa |
| `README.md` | Ovaj dokument |

## DrivingMode Enum

Enum `DrivingMode` definiše **15 mogućih stanja** vozila:

### Normalna vožnja

| Mod | Opis |
|-----|------|
| `LANE_FOLLOWING` | Podrazumevani mod — praćenje linija trake pri normalnoj brzini (150). |

### Raskrsnice

| Mod | Opis |
|-----|------|
| `CROSSING` | Prilaz ili prolaz kroz raskrsnicu (prioritetnu / neprioritetnu). |
| `ROUNDABOUT` | Prilaz ili prolaz kroz kružni tok. |

### Pešaci i prepreke

| Mod | Opis |
|-----|------|
| `PEDESTRIAN_CROSSING` | Pešački prelaz — usporavanje, provera za pešake. |
| `OBJECT_ON_ROAD` | Objekat detektovan na putu — zaustavljanje ili obilaženje. |

### Specijalne deonice puta

| Mod | Opis |
|-----|------|
| `TUNNEL` | Tunel — uključivanje svetala, korekcija brzine. |
| `HIGHWAY` | Auto-put — povećana brzina (300), praćenje traka. |
| `ONE_WAY` | Jednosmerna ulica. |
| `PARKING` | Zona parkiranja — smanjena brzina, traženje slobodnog mesta. |

### Semafori

| Mod | Opis |
|-----|------|
| `TRAFFIC_LIGHT_RED` | Crveno svetlo — zaustavljanje vozila. |
| `TRAFFIC_LIGHT_YELLOW` | Žuto svetlo — priprema za zaustavljanje. |
| `TRAFFIC_LIGHT_GREEN` | Zeleno svetlo — nastavak vožnje. |

### Vanredni / specijalni modovi

| Mod | Opis |
|-----|------|
| `STOP` | Stop znak ili vanredno zaustavljanje. |
| `NO_ENTRY` | Zabrana ulaska — preusmeravanje. |
| `RAMP` | Rampa / nagib — korekcija brzine za uspon/silaz. |

## DrivingModeContext klasa

`DrivingModeContext` je wrapper koji čuva **trenutno stanje** vožnje zajedno sa meta-podacima.

### Atributi

| Atribut | Tip | Opis |
|---------|-----|------|
| `current_mode` | `DrivingMode` | Trenutno aktivan mod vožnje. |
| `previous_mode` | `DrivingMode \| None` | Prethodni mod (za revert). |
| `mode_data` | `dict` | Dodatni parametri specifični za mod. |

### Metode

| Metoda | Povratni tip | Opis |
|--------|-------------|------|
| `set_mode(new_mode, data=None)` | `bool` | Prelaz u novi mod. Vraća `True` ako je mod zaista promenjen. |
| `revert_to_previous()` | `bool` | Vraćanje na prethodni mod. |
| `get_mode_duration()` | `float` | Vreme (u sekundama) koliko je trenutni mod aktivan. |
| `is_mode(mode)` | `bool` | Provera da li je aktivan dati mod. |

### Primer `mode_data`

Svaki mod može imati pridružene podatke koji detaljnije opisuju kontekst:

```python
# Primer: raskrsnica sa pravcem skretanja
context.set_mode(DrivingMode.CROSSING, {"direction": "left", "priority": False})

# Primer: parking sa ID mesta
context.set_mode(DrivingMode.PARKING, {"spot_id": 2, "side": "right"})

# Primer: stop znak — izvor detekcije
context.set_mode(DrivingMode.STOP, {"source": "stop_line"})
```

## Mapiranje YOLO detekcija → DrivingMode

`threadDecisionMaking` koristi YOLO detekcije iz `LaneDetection.get_drive_params()` i mapira ih na odgovarajuće modove:

| YOLO Label | DrivingMode tranzicija | Ponašanje |
|------------|----------------------|-----------|
| `"priority"` | — (ostaje u trenutnom) | Nastavlja normalno. |
| `"crosswalk_blue"` | → `PEDESTRIAN_CROSSING` | Usporava, proverava pešake. |
| `"stop"` | Armira `toStop` flag | Kada se detektuje stop linija → `STOP` (2s pauza). |
| `"parking"` | → `PARKING` | Aktivira logiku parkiranja. |
| `"highway_enter"` | → `HIGHWAY` | Brzina: 150 → 300. |
| `"highway_exit"` | → `LANE_FOLLOWING` | Brzina: 300 → 150. |
| `"no_entry"` | → `NO_ENTRY` | Zabrana prolaza. |
| `"roundabout"` | → `ROUNDABOUT` | Prilaz kružnom toku. |
| `"one_way"` | → `ONE_WAY` | Jednosmerna ulica. |
| `"red"` | → `TRAFFIC_LIGHT_RED` | Zaustavljanje vozila. |
| `"yellow"` | → `TRAFFIC_LIGHT_YELLOW` | Zaustavljanje vozila. |
| `"green"` | → `TRAFFIC_LIGHT_GREEN` | Nastavak vožnje. |
| `"off"` | — | Bez akcije. |

## Dijagram tranzicija

```
                    ┌──────────────────┐
                    │  LANE_FOLLOWING   │◄──────────────────┐
                    └────────┬─────────┘                    │
                             │                              │
            ┌────────────────┼────────────────┐             │
            ▼                ▼                ▼             │
   ┌──────────────┐  ┌─────────────┐  ┌────────────┐       │
   │   CROSSING   │  │  ROUNDABOUT │  │  ONE_WAY   │───────┤
   └──────────────┘  └─────────────┘  └────────────┘       │
                                                            │
            ┌───────────────────────────────┐               │
            ▼                               ▼               │
   ┌──────────────────┐            ┌──────────────┐         │
   │ PEDESTRIAN_CROSS. │            │   HIGHWAY    │─────────┤
   └──────────────────┘            └──────────────┘         │
                                                            │
            ┌───────────────────────────────┐               │
            ▼                               ▼               │
   ┌──────────────┐                ┌──────────────┐         │
   │    PARKING   │                │     STOP     │─────────┤
   └──────────────┘                └──────┬───────┘         │
                                          │ (2s)            │
                                          └─────────────────┘
                                                            
   ┌──────────────────────────────────────────┐             
   │           TRAFFIC LIGHTS                  │             
   │                                           │             
   │  RED ──→ YELLOW ──→ GREEN ──→ LANE_FOLLOW │             
   │   ▲                              │        │             
   │   └──────────────────────────────┘        │             
   └──────────────────────────────────────────┘             
```

## Brzine po modovima

| Mod | Brzina (PWM) | Konstanta |
|-----|-------------|-----------|
| `LANE_FOLLOWING` | 150 | `SPEED_NORMAL` |
| `HIGHWAY` | 300 | `SPEED_HIGHWAY` |
| `PEDESTRIAN_CROSSING` | 100 | `SPEED_SLOW` |
| `STOP` / `TRAFFIC_LIGHT_RED` / `TRAFFIC_LIGHT_YELLOW` | 0 | `SPEED_STOP` |

## Korišćenje

```python
from src.DecisionMaking.DrivingModes.drivingMode import DrivingMode, DrivingModeContext

# Kreiranje konteksta (početni mod: LANE_FOLLOWING)
ctx = DrivingModeContext()
print(ctx.current_mode)  # DrivingMode.LANE_FOLLOWING

# Tranzicija u HIGHWAY mod
changed = ctx.set_mode(DrivingMode.HIGHWAY)
print(changed)           # True
print(ctx.current_mode)  # DrivingMode.HIGHWAY
print(ctx.previous_mode) # DrivingMode.LANE_FOLLOWING

# Provera trajanja moda
duration = ctx.get_mode_duration()
print(f"U HIGHWAY modu već {duration:.1f}s")

# Povratak na prethodni mod
ctx.revert_to_previous()
print(ctx.current_mode)  # DrivingMode.LANE_FOLLOWING

# Provera trenutnog moda
if ctx.is_mode(DrivingMode.LANE_FOLLOWING):
    print("Normalna vožnja")
```

## Proširivanje sistema

Za dodavanje novog moda vožnje:

1. Dodati novu vrednost u `DrivingMode` enum u [drivingMode.py](drivingMode.py)
2. Dodati handler u `_handle_detection()` metodu u `threadDecisionMaking`
3. Definisati ponašanje (brzina, skretanje, kočenje) za novi mod
4. Ažurirati YOLO model ako je potrebna nova detekcija znaka
