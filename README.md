# 3D Adaptive Cruise Control Simulator (pygame)

![tests](https://github.com/IngQadir/acc-3d-pygame/actions/workflows/tests.yml/badge.svg)

![ACC demo](demo.gif)

A 3D highway driving simulator that demonstrates **Cruise Control / Adaptive Cruise Control (ACC)**.
The 3D view (chase camera, perspective road, 3D car models) is rendered with a small
custom projection engine on top of **pygame only**: no OpenGL, no CARLA, no GPU needed.

## Features
- **Normal driving** when cruise is OFF: you control the speed with **+ SPEED / - SPEED / BRAKE**.
- **CRUISE button**: switches cruise ON and holds the current speed (minimum 30 km/h). Press again for OFF.
- **Driver override**: pressing BRAKE, + SPEED or - SPEED switches cruise OFF, like in a real car.
- **Radar / ACC**: a slower car ahead is detected (150 m range). Your car slows down automatically
  and keeps a **1.8 s time gap**, and the amber **FOLLOW** lamp blinks.
- **Lane change of the car ahead**: it switches on its **indicator (blinker)** and moves to another lane.
  When the lane is free, your car **automatically accelerates back to the set speed**.
- **Forward collision warning**: a red BRAKE! warning when the time-to-collision is too short.
- **Dashboard**: speedometer, big **CRUISE ON / OFF** state, set speed, mode, radar distance, lamps.

## How it works

### Program flow (one frame, ~60 times per second)
```mermaid
flowchart TD
    A([Start]) --> B[Read keyboard and mouse]
    B --> C{CRUISE pressed?}
    C -- yes --> D{Cruise ON?}
    D -- yes --> E[Cruise OFF]
    D -- no --> F{Speed >= 30 km/h?}
    F -- yes --> G[Cruise ON, set speed = current speed]
    F -- no --> H[Show: needs 30 km/h]
    C -- no --> I
    E --> I
    G --> I
    H --> I
    I{Cruise ON and BRAKE / +SPEED / -SPEED held?}
    I -- yes --> J[Cruise OFF: driver override]
    I -- no --> K
    J --> K[Radar: nearest car ahead in my lane within 150 m]
    K --> L{Cruise ON?}
    L -- no --> M[Normal driving: brake -7.0 / accel +2.5 / decel -2.0 / coast -0.3 m/s²]
    L -- yes --> N[ACC controller]
    M --> O[Actuator lag 0.3 s, update speed and position]
    N --> O
    O --> P[Move other cars, lead car indicator and lane change]
    P --> Q[Spawn traffic, collision check, brake warning]
    Q --> R[Draw 3D road, cars, radar beam and dashboard]
    R --> B
```

### ACC controller decision
```mermaid
flowchart TD
    S([Cruise ON]) --> A["Speed PI: a_speed = 0.6·(v_set − v) + 0.08·∫error"]
    A --> T{Car ahead detected by radar?}
    T -- no --> U["a = a_speed  →  mode CRUISE (hold / resume set speed)"]
    T -- yes --> V["Gap control: d_des = 6 m + 1.8 s·v<br/>a_gap = 0.15·(gap − d_des) + 0.7·(v_lead − v)"]
    V --> W{a_gap < a_speed?}
    W -- yes --> X["a = a_gap  →  mode FOLLOWING (FOLLOW lamp blinks)"]
    W -- no --> U
    U --> Y["Limit a to −3.5 … +1.8 m/s²"]
    X --> Y
    Y --> Z([Acceleration command to vehicle])
```

### Cruise control states
```mermaid
stateDiagram-v2
    [*] --> OFF
    OFF --> CRUISE: CRUISE pressed (v ≥ 30 km/h)
    CRUISE --> FOLLOWING: slower car ahead
    FOLLOWING --> CRUISE: car ahead changes lane, lane free
    CRUISE --> OFF: BRAKE / +SPEED / −SPEED / CRUISE
    FOLLOWING --> OFF: BRAKE / +SPEED / −SPEED / collision
```

### Lead car behaviour
```mermaid
flowchart LR
    A[Spawn 200 m ahead, slower] --> B[Drive in my lane]
    B --> C[Followed for 3–6 s]
    C --> D[Indicator blinks 2.5 s]
    D --> E[Lane change 2.5 s]
    E --> F[Radar loses target]
    F --> G[My car resumes set speed]
```

## Controls
| Action            | Keyboard   | On-screen button |
|-------------------|------------|------------------|
| Increase speed    | Up arrow   | + SPEED          |
| Decrease speed    | Down arrow | - SPEED          |
| Brake             | Space      | BRAKE            |
| Cruise ON / OFF   | C          | CRUISE           |
| Reset / Quit      | R / Esc    |                  |

## Run
```bash
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python acc_3d.py
```
Keep this project in its own folder and virtual environment so it never touches a CARLA installation.

## Tests
```bash
python -m pytest -q
```
The tests cover default normal driving, engage/disengage, cancel on brake/accel/decel, speed hold,
following at the time gap, the lead car's indicator and lane change with automatic resume,
and a 5-minute random traffic run with zero collisions. They run on every push via GitHub Actions.

## Architecture
```
acc_model.py       vehicle dynamics, traffic, CruiseControl (no pygame, unit-testable)
acc_3d.py          3D renderer (camera projection, road, car models), dashboard, input
test_acc_model.py  pytest unit tests
make_demo_gif.py   renders the scripted demo scenario to demo.gif
```

**Controller**: a speed PI controller (with anti-windup) and a gap controller
`a = K_gap*(gap - (d0 + T_gap*v)) + K_rel*(v_lead - v)` are combined with `min()` arbitration,
so the more cautious command always wins. The command is limited to -3.5 to +1.8 m/s^2 and passed
through a first-order actuator lag (tau = 0.3 s).

## Possible extensions
- Set speed +/- buttons while cruise is ON (like SET+/SET- on a steering wheel)
- Selectable time gap (1.0 / 1.5 / 2.0 s)
- Stop & Go to standstill, cut-in scenarios
- Port the controller to C and call it through ctypes
