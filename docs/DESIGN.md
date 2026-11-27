# ECU Simulator — Design Specification

**Status:** Draft v0.1 · 2026-11-26
**Scope:** Cross-platform desktop application that simulates the engine control module (ECM) of a ~2015 1.6 L DOHC port-injected gasoline engine in a car with a 5-speed manual gearbox. It talks to external diagnostic equipment over a USB-CAN adapter using **generic OBD-II on CAN (SAE J1979 / ISO 15765-4)**.

---

## 1. Goals and non-goals

### Goals
1. Engine and vehicle behaviour that hangs together physically. Every value a scan tool reads must agree with every other value: MAF with MAP and RPM, fuel trims with lambda, timing with load, coolant temperature with warm-up time.
2. Realistic ECU strategy:
   - Cranking, start flare, open loop when cold, catalyst heating, closed loop.
   - Adaptive fuel trims, deceleration fuel cut, power enrichment, idle-speed control.
   - Limp-home modes.
3. Driver-style controls: ignition key, accelerator, brake, clutch, gear lever and handbrake, plus a gauge cluster with warning lamps.
4. Fault injection on demand. Each fault drives the physical model or a sensor, not the DTC directly. The ECU then *detects* the fault through its own monitors, the way a real ECM does.
5. Full generic OBD-II service set: Modes 01, 02, 03, 04, 06, 07, 09 and 0A. Readiness monitors either follow drive-cycle logic or are forced from the UI.
6. Runs on Linux, Windows and macOS, and works with common USB-CAN adapters.

### Non-goals (v1)
- Manufacturer-specific diagnostics (UDS/KWP security access, coding, flashing). The architecture leaves room to add UDS later.
- K-line, J1850 and other non-CAN OBD protocols.
- Multiple ECUs on the bus (TCM, ABS, BCM). There is a single ECM at `0x7E0`/`0x7E8`.
- Crank-angle-resolved combustion simulation. The model runs per event and per cycle.
- Matching any specific production vehicle's calibration.

---

## 2. Reference engine and vehicle

### 2.1 Engine
| Parameter | Value |
|---|---|
| Layout | Inline 4, DOHC 16 V, single bank |
| Displacement | 1598 cc (bore 79.0 × stroke 81.5 mm) |
| Compression ratio | 10.5 : 1 |
| Valve timing | Intake cam phaser (VVT), 0–40° crank advance range |
| Fuel system | Sequential port injection, returnless, 3.0 bar (300 kPa) rail |
| Injectors | 220 cc/min @ 3.0 bar (≈ 2.71 mg/ms), dead time ≈ 0.65 ms @ 14 V |
| Ignition | Coil-on-plug, 4 coils |
| Throttle | Electronic (drive-by-wire), dual TPS tracks |
| Peak power | ≈ 92 kW (125 PS) @ 6000 rpm |
| Peak torque | ≈ 155 Nm @ 4000 rpm |
| Redline / limiter | 6500 rpm / fuel-cut limiter at 6700 rpm |
| Warm idle | 750 rpm (850 rpm with A/C on) |
| Fuel | RON 95 gasoline, stoich AFR 14.7 |
| Engine + flywheel inertia | ≈ 0.15 kg·m² |

### 2.2 Sensors and actuators
| Item | Type | ECU uses it for |
|---|---|---|
| **MAF** | Hot-film with integrated IAT | Main load signal |
| **MAP** | 1-bar manifold pressure | MAF rationality check, backup load if the MAF fails, baro learn at key-on |
| **IAT** | NTC (inside the MAF housing) | Charge density, spark correction |
| **ECT (CTS)** | NTC | Warm-up strategy, fan control, enrichment |
| **TPS A / TPS B** | Dual-track throttle position | Throttle feedback and correlation check |
| **APP D / APP E** | Dual-track pedal position | Driver demand and correlation check |
| **CKP** | 60-2 tooth wheel | RPM, sync, misfire detection |
| **CMP** | Intake cam, 4 teeth | Phase sync, VVT angle measurement |
| **Knock sensor** | Piezo, one per engine | Per-cylinder knock retard |
| **Upstream O2 (B1S1)** | Wideband (5-wire planar) | Closed-loop fuelling |
| **Downstream O2 (B1S2)** | Narrowband zirconia, heated | Catalyst monitor and downstream trim |
| **Fuel tank pressure** | Differential | EVAP leak monitor |
| **Fuel level** | Float potentiometer | Gauge and EVAP enable conditions |
| **VSS** | From ABS wheel speed | Vehicle speed, decel fuel cut, idle mode |
| **Clutch switches** | Top/bottom of travel | Starter interlock, decel strategy |
| **Brake switch** | Two contacts | Brake-overrides-throttle, idle |
| **Battery voltage** | ECU-internal | Injector dead time, coil dwell, P0562/P0563 |
| Actuators | Injectors ×4, coils ×4, throttle motor, VVT oil-control valve, EVAP purge valve, canister vent valve, cooling fan (2 speeds), fuel pump relay, O2 heaters ×2, A/C clutch, alternator field (smart charging off) | |

### 2.3 Vehicle and driveline
| Parameter | Value |
|---|---|
| Kerb mass + driver | 1250 kg |
| Drag | Cd 0.32, frontal area 2.10 m² |
| Rolling resistance | Crr 0.012 |
| Tyres | 195/55 R16, rolling radius 0.305 m |
| Gear ratios | 1st 3.55 · 2nd 1.90 · 3rd 1.28 · 4th 0.95 · 5th 0.76 · R 3.50 |
| Final drive | 4.06 |
| Clutch | Friction model with bite point at ~35–55% pedal travel. The engine can stall. |
| Brakes | Max deceleration ≈ 9 m/s², plus a handbrake |
| Road | Adjustable grade (−10% to +10%) |

Sanity check: 5th gear at 100 km/h is about 2700 rpm. The drag-limited top speed is about 195–200 km/h, reached in 5th gear at ~5400 rpm.

---

## 3. Architecture

```
┌────────────────────────────── UI (PySide6) ─────────────────────────────┐
│ Driver panel · Gauge cluster · Live data · Charts · Faults · Diagnostics │
│ CAN setup · Bus trace · Environment · Time control                       │
└───────────────▲──────────────────────────────────────────┬───────────────┘
                │ snapshots (≈30 Hz)                       │ driver inputs, fault commands
┌───────────────┴──────────────────────────────────────────▼───────────────┐
│                     Simulation core (fixed step, 100 Hz)                 │
│                                                                          │
│  ┌──────────────┐   true state   ┌──────────────┐  sensor values  ┌─────┐ │
│  │ PLANT        │ ─────────────► │ SENSORS +    │ ──────────────► │ ECU │ │
│  │ engine,      │                │ WIRING/FAULT │                 │logic│ │
│  │ vehicle,     │ ◄───────────── │ LAYER        │ ◄────────────── │     │ │
│  │ thermal,     │  actuator      └──────────────┘  actuator cmds  └──┬──┘ │
│  │ exhaust,     │  effects                                            │    │
│  │ EVAP, elec.  │                                     diag data store │    │
│  └──────────────┘                                                     ▼    │
│                                          DTC mgr · monitors · freeze frame │
└──────────────────────────────────────────────────────────▲────────────────┘
                                                           │ read-only snapshot
┌──────────────────────────────────────────────────────────┴───────────────┐
│ Diagnostic server: OBD service handlers → ISO-TP → CAN transport          │
│ (python-can: socketcan / slcan / pcan / kvaser / gs_usb / virtual)        │
└──────────────────────────────────────────────────────────────────────────┘
```

### 3.1 Key principle: plant, sensors and ECU are separate
- The **plant** holds the *true* physical state: real airflow, real lambda, real temperatures.
- The **sensor layer** converts true state into what each sensor reports. It adds transfer functions, time constants, noise and any active faults (open, short, offset, stuck, slow, noisy, intermittent).
- The **ECU** sees *only* sensor values. It never reads plant state, and it decides strategy, actuator commands and diagnoses from what it sees.

This separation is what makes faults behave correctly. Two examples:
- **Vacuum leak.** Air enters after the MAF, so the MAF under-reads. The ECU fuels for less air than the engine actually gets, so true lambda goes lean. The wideband reports lean, STFT and then LTFT climb, and P0171 sets.
- **Exhaust leak ahead of B1S1.** Outside air dilutes the gas at the sensor, so it reads lean while the engine is actually lean-correct. The ECU adds fuel, the engine really runs rich, and the downstream sensor sees rich.

### 3.2 Threads and timing
| Thread | Rate | Job |
|---|---|---|
| Sim | 100 Hz sim-time (10 ms step), sub-stepped at 1 ms for fast dynamics such as manifold filling and engine speed | Plant, sensors, ECU, diagnostics |
| CAN RX/TX | Event-driven | ISO-TP, OBD responses built from the latest snapshot |
| UI | ~30 Hz | Render gauges, charts, tables |

- The sim publishes an immutable snapshot each step. The diagnostic server and UI read only snapshots, so they never block the sim.
- OBD responses are sent within ~5–10 ms of a request, well inside the 50 ms P2 limit. Response timing is wall-clock time and is not affected by time acceleration.
- **Time control:** pause, single-step, and ×1 / ×2 / ×5 / ×10 / ×20. Everything the sim counts (run time, drive cycles, monitor timers) uses sim time.

### 3.3 Tech stack
| Concern | Choice | Reason |
|---|---|---|
| Language | Python 3.12+ | Fast iteration, good CAN ecosystem |
| GUI | PySide6 (Qt 6) | Native look on all 3 OSes, custom painting for gauges |
| Charts | pyqtgraph | Fast real-time strip charts |
| CAN | python-can | One API over every adapter backend |
| ISO-TP / OBD | Written in-house (small) | Full control of padding, flow control, timing and multi-PID requests |
| Persistence | JSON files (keep-alive memory, config) | Readable, easy to diff |
| Tests | pytest | |
| Packaging | PyInstaller (one-folder bundles per OS) | |
| Gamepad / wheel input (later) | pygame / SDL2 | Qt 6 has no gamepad module |

If the 100 Hz Python loop turns out too slow (not expected), the plant can move to a Rust or C extension with no change to the architecture.

### 3.4 CAN adapter support
| Adapter | Linux | Windows | macOS |
|---|---|---|---|
| CANable / candleLight (gs_usb) | socketcan (kernel) | gs_usb via libusb | — |
| CANable (slcan firmware) | slcan | slcan (COM port) | slcan (tty) |
| PEAK PCAN-USB | socketcan or PCAN | PCAN-Basic | PCBUSB (MacCAN) |
| Kvaser | socketcan or CANlib | CANlib | — |
| Virtual / vcan0 | yes | python-can virtual | python-can virtual |

**Recommended cheap adapter:** CANable 2.0. Its slcan firmware works the same on all three OSes.

**Bench wiring:** use a female OBD-II (J1962) connector.
- Pin 6 = CAN-H, pin 14 = CAN-L, with 120 Ω termination at the simulator end.
- Pins 4 and 5 = ground.
- **Pin 16 = +12 V from a bench supply.** Handheld scan tools are powered from this pin.

---

## 4. Plant model

All values below are targets for the default calibration. Calibration tables live in data files so they can be tuned.

### 4.1 Air path
- **Air filter:** a small pressure drop proportional to flow². The "restricted filter" fault scales it up.
- **MAF location:** upstream of the throttle. It measures true airflow *through the throttle* only, so it does not include vacuum-leak air or EVAP purge flow.
- **Throttle:** compressible orifice flow. Effective area is a function of throttle angle, including a 1.5–2% mechanical rest stop and a carbon-deposit factor used by the dirty-throttle fault. The throttle motor is modelled as a second-order system (τ ≈ 30 ms) tracking the ECU's target angle.
- **Intake manifold:** filling and emptying dynamics.
  - dP/dt = (R·T/V)·(ṁ_in − ṁ_cyl), with manifold volume ≈ 2.5 L.
  - ṁ_in includes throttle flow, vacuum leak, purge flow and PCV.
- **Cylinder flow:** ṁ_cyl = VE(rpm, MAP, VVT) · V_d · (rpm/120) · ρ(MAP, T_charge).
  - Charge temperature T_charge blends IAT with ECT at low flow, which gives heat-soak behaviour.
- Target values:

| Condition | MAP | MAF | Calculated load (PID 04) |
|---|---|---|---|
| Warm idle, 750 rpm | 30–34 kPa | 2.5–3.0 g/s | ~20–25% |
| Cold fast idle, 1150 rpm, 15 °C | ~38 kPa | ~4.5 g/s | |
| Cruise 100 km/h, 5th gear | ~55 kPa | ~12–15 g/s | |
| WOT, 6000 rpm | baro − 3 kPa | ~85–95 g/s | ~95–100% |

- **Baro:** user-set altitude gives baro (sea level = 101 kPa). The ECU learns baro from MAP at key-on and again at WOT.

### 4.2 Fuel path
- Injected mass per cylinder = ECU command (pulse width − dead time(V_batt)) × flow rate × per-injector health factor.
- **Wall wetting (x-τ model):** a fraction x of injected fuel deposits on the port wall and evaporates with time constant τ. Both are larger when cold. This produces realistic lean tip-in and rich tip-out spikes, which the ECU's transient compensation partly cancels.
- **Purge vapour:** adds fuel mass when the canister holds vapour (see 4.7).
- **Leaking injector fault:** adds fuel, including after shutdown, which gives a rich hot restart.
- **True lambda** per cylinder = air ÷ (fuel × 14.7). The exhaust sees the mixed value after a transport delay.
- Target pulse widths:
  - Warm idle ≈ 3.3 ms.
  - Cold idle at 15 °C ≈ 4.0–4.5 ms.
  - Cranking ≈ 10–15 ms.
  - WOT at 6000 rpm ≈ 13.5 ms, which is ~68% duty.

### 4.3 Combustion and torque
- Indicated torque = air mass per cycle × fuel conversion efficiency × η_spark(Δ from MBT) × η_lambda(λ) × misfire factor.
  - η_spark is a parabolic loss for spark away from MBT. About −10° from MBT gives ~10% torque loss.
  - η_lambda peaks at λ ≈ 0.88 (best torque) and falls off when lean.
- Losses:
  - **Friction (FMEP):** a function of rpm and oil temperature. Cold friction is ~40% higher at 15 °C.
  - **Pumping:** from MAP versus exhaust back-pressure.
  - **Accessories:** alternator (from electrical load), A/C compressor, power steering.
- **Engine speed:** J·dω/dt = T_engine − T_clutch − T_losses. Stalls below ~300 rpm when the clutch is engaged.
- **Misfire:** each cylinder has a misfire probability per event, set by faults (coil, plug, injector, lean limit, compression). A misfire produces:
  - no torque from that event, so the crank decelerates and the CKP signal shows it;
  - unburnt mixture to the exhaust, so the cat heats up and the upstream O2 reads lean from the extra oxygen.
- **Knock:** knock intensity rises when spark advance exceeds the knock-limited spark advance (KLSA). KLSA depends on load, rpm, IAT, ECT and fuel octane. "Low-octane fuel" and "carbon build-up" faults lower KLSA.

### 4.4 Rotation and crank/cam signals
- The CKP and CMP signals are not modelled as waveforms. The plant gives per-tooth-event timing so the ECU can work out sync status, per-cylinder segment speed (for misfire detection) and cam phase.
- Cranking: starter drives the engine at ~200–250 rpm (lower with a weak battery or when cold).
  - Sync needs the CKP gap (~1 revolution) and then a CMP edge.
  - First firing comes about 1.0–1.5 s after the starter engages.
- Faults:
  - CKP loss: the engine dies or won't start, and the tachometer drops to 0.
  - CMP loss: the engine still starts but takes longer, using batch fire and waste spark.
  - Timing-chain stretch: the cam/crank offset error grows.

### 4.5 Thermal model
- **Coolant (engine loop before the thermostat):** effective thermal mass ≈ 25 kJ/K.
  - Heat in = ~30% of fuel energy, plus friction.
  - Heat out = block-to-ambient, plus the cabin heater core (switchable), plus the radiator (once the thermostat opens).
- **Thermostat:** starts opening at 88 °C and is fully open at 100 °C. Faults: stuck open, stuck closed.
- **Radiator:** heat transfer (UA) depends on ram airflow from vehicle speed plus the fan (2 speeds).
- **Fan:** low speed on at 98 °C and off at 94 °C. High speed on at 105 °C. Also on with A/C.
- **Oil:** lags coolant and settles 5–15 °C above it under load.
- **IAT:** ambient plus under-hood heat-soak at low vehicle speed and idle. Pulls back toward ambient with airflow. Hot restarts show high IAT.
- **Catalyst:** a lumped thermal mass heated by exhaust gas temperature. EGT depends on load, spark retard and lambda. The catalyst also gains exothermic heat from unburnt HC and O2, which is how misfire overheats it.
  - Light-off ≈ 280 °C. Normal operation 400–750 °C. Damage above ~950 °C.
- **Soak:** at key-off, temperatures decay toward ambient with an engine-off time constant (~2 h to fall within 5 °C of ambient). The UI can skip ahead in soak time.
- Target behaviour: from 15 °C, about 12–15 min at idle or 5–8 min of mixed driving to reach 90 °C.

### 4.6 Exhaust, catalyst and O2 sensors
- **Transport delay** from exhaust port to B1S1 ≈ 0.25 s at idle, falling to ~40 ms at high flow (scales with 1/ṁ).
- **Wideband B1S1:**
  - Lambda range 0.65–3.0 (reads "air" during fuel cut). First-order response, τ ≈ 80–120 ms.
  - Heater warm-up ≈ 15 s from cold.
  - The ECU reports equivalence ratio and pump current (PID 34) and an equivalent voltage (PID 24).
  - Faults: slow response, offset/bias, heater open, signal stuck.
- **Catalyst oxygen storage:**
  - Storage state s ∈ [0, 1]; capacity C (mg O2) ≈ health × temperature factor × nominal.
  - Lean gas fills s and rich gas drains it. When the cat is full or empty, the excursion passes through to the downstream gas.
- **Narrowband B1S2:**
  - Output 0.05–0.95 V with a steep switch near λ = 1. Response τ ≈ 150 ms.
  - Dew-point protection: the heater stays at low power until the cat outlet passes ~100 °C, so the signal sits around 0.45 V until then.
  - With a healthy, lit-off cat it holds steady at ~0.6–0.75 V. A worn cat makes it follow the upstream oscillation.
  - Faults: slow, stuck low or high, heater open.

### 4.7 EVAP system
- **Tank:** vapour generation depends on fuel temperature and fill level. The canister load state is 0–1.
- **Purge valve:** PWM. Flow depends on duty and manifold vacuum. Vapour content depends on canister load.
- **Canister vent valve:** normally open. The ECU closes it for leak tests.
- **Tank pressure sensor:** PID 32.
- **Leak:** an orifice size of 0, 0.5, 1.0 mm or a large leak (missing fuel cap), as faults.
- **Test (vacuum decay):**
  1. Close the vent and pull −2 kPa with purge.
  2. Seal the system and watch the pressure rise over ~20 s.
  3. Compare the rise rate with thresholds for 1.0 mm (P0442) and large leak (P0455). A separate engine-off natural-vacuum test checks for 0.5 mm (P0456, US region only).

### 4.8 Electrical
- **Battery:** state of charge, internal resistance and open-circuit voltage ≈ 12.6 V. A "weak battery" fault raises resistance and lowers capacity.
- **Starter:** ~150 A, so voltage dips to ~10–10.5 V while cranking. Below ~8 V during crank the ECU resets and the engine won't start.
- **Alternator:** regulates to 14.2–14.4 V when running, with a current limit. Fault: no output (P0562 after a delay, battery lamp on), or over-voltage (P0563).
- **Loads:** headlights, blower, rear demister, fan and O2 heaters. They change alternator torque, which the idle controller must catch.

### 4.9 Vehicle dynamics
- Longitudinal model only: tractive force from clutch torque through gear and final drive, minus aero drag, rolling resistance, grade and brakes.
- **Clutch:** pedal position gives clamp force. Friction torque is limited by slip, with a stick-slip transition. Releasing too fast at low rpm stalls the engine.
- **Gear changes:** only in neutral or with the clutch pressed past ~80% travel. Otherwise the UI refuses the change with a "grind" indication.
- **Starter interlock:** the engine cranks only with the clutch pressed or the gearbox in neutral (configurable).

---

## 5. ECU logic

### 5.1 Operating modes
| Mode | Entry | Behaviour |
|---|---|---|
| Key off | — | No CAN traffic. Keep-alive memory retained. |
| KOEO (key on, engine off) | Key to ON | Fuel pump primes 2 s, throttle self-test, MIL bulb check, baro learn from MAP. Answers OBD. |
| Crank | Starter running, RPM below 400 | Sync, cranking fuel from ECT table, fixed spark ≈ 5–10° BTDC. Pressing the pedal fully gives clear-flood (no fuel). |
| Start flare / post-start | RPM above 400 | Post-start enrichment decaying over a time set by start ECT (≈ 40 s at 15 °C), idle target from ECT table |
| Catalyst heating | Start ECT 5–60 °C, first ~30 s | Retarded spark (−2…+3°), raised idle airflow, slightly lean (λ 1.00–1.02) once closed loop |
| Open loop – cold | Wideband not ready, or post-start enrichment active | Commanded λ from ECT table (0.85–0.95). Fuel status 1. |
| Closed loop | Wideband ready, enrichment done, no fault | λ target 1.00, wideband PI loop (STFT), LTFT learning, downstream trim. Fuel status 2. |
| Open loop – drive | Power enrichment (high load above threshold) or decel fuel cut | Fuel status 4 |
| Open loop – fault | Wideband fault, MAF fault, etc. | Fuel status 8 (or 16 if closed loop with a degraded sensor) |
| Limp home | APP/TPS/throttle faults | Throttle at limp position (~7% via spring), idle ~1200 rpm, power limited, MIL on |
| Rev limit | RPM ≥ 6700 | Fuel cut, resumes at 6600 |

Typical cold start at 15 °C ambient: closed loop at ~25–45 s, catalyst lit off by ~30–40 s, downstream heater at full power by ~60–90 s, warm idle of 750 rpm reached at ECT ≈ 75–80 °C.

### 5.2 Torque and throttle control
- APP gives driver torque demand (pedal map by rpm). Arbitration with the idle controller, rev limiter, limp mode and brake override (brake plus pedal → reduced demand) gives the throttle target angle.
- Dual-track plausibility checks:
  - APP D vs APP E: D = 2 × E scaling, tolerance 5%.
  - TPS A vs TPS B: complementary tracks.
  - Commanded vs actual throttle (P2101).

### 5.3 Idle-speed control
- Target rpm vs ECT: 15 °C → 1150, 40 °C → 1000, 70 °C → 850, ≥ 85 °C → 750. Add +100 rpm with A/C on and +50 rpm at low battery voltage.
- Airflow path: PI on throttle position, plus an adaptive learned offset (idle-air learning, reset by battery disconnect).
- Fast path: spark reserve held ~4–6° retarded from MBT at idle so torque can be added instantly. This makes timing advance wander visibly at idle, as on real cars.
- Feed-forward for A/C, fan, electrical loads and clutch disengage.
- Dashpot on throttle closing to prevent stall.

### 5.4 Fuelling
1. Air per cylinder from the MAF, with a manifold-filling correction and predicted to the intake valve closing point. In MAF-fault backup it comes from MAP with speed-density instead.
2. Base fuel = air ÷ (14.7 × λ_target).
3. × (1 + STFT + LTFT(cell) + purge correction).
4. + transient compensation (inverse x-τ wall-wetting model).
5. Add injector dead time by battery voltage → pulse width.
6. Injection end angle ≈ 400° BTDC firing (closed-valve injection). Open-valve injection is used during cranking and high load.
- **STFT:** PI on wideband error, range ±25%, about 3 Hz bandwidth. It hovers around 0 ±2–3% because it is wideband-controlled, with no narrowband sawtooth.
- **LTFT:** 4 cells (idle, low, mid, high load). Each learns 0.5%/s toward mean STFT once its cell has been stable for 5 s. Range ±25%. Stored in keep-alive memory.
- **Downstream trim:** slow adjustment of the λ target (±0.02) to keep B1S2 at ~0.65 V. Reported in PID 15 byte B.
- **Purge learning:** when purge is active, the vapour estimate absorbs the fuel shift so LTFT stays clean. Before the estimate converges, STFT goes negative.
- **Decel fuel cut:** enters when warm, throttle closed, rpm > 1300 and vehicle moving. Resumes below 1100 rpm or on pedal tip-in. Fuel status 4. Wideband reads air.
- **Power enrichment:** load > 85% for > 0.5 s, or cat-protection temperature model > 850 °C → λ 0.85–0.88.

### 5.5 Spark
- Base map (rpm × load) at MBT or KLSA:
  - Warm idle 10–14° BTDC (including reserve).
  - Light-load cruise 32–40°.
  - WOT 4000 rpm ≈ 18–22°.
  - WOT 6000 rpm ≈ 26–30°.
- Corrections:
  - ECT: cold adds advance at part load.
  - IAT: hot intake air retards.
  - Catalyst heating, idle-speed control, and transient (tip-in retard).
  - Knock retard per cylinder: 2° steps up to 8° max, recovering 0.5° per 2 s.
- PID 0E reports cylinder-1 spark (or the average).

### 5.6 VVT (intake cam)
- Target by rpm × load: 0° at idle, up to ~30° advance at mid-load cruise, ~10° at high rpm WOT.
- PI control of the oil-control valve. Response τ ≈ 150 ms when warm, slower with cold oil.
- Below ~40 °C oil the cam is held at its park position.
- Faults: solenoid stuck (P0011 / P0012 if over- or under-advanced versus target), chain stretch (P0016).

### 5.7 Cooling fan, A/C, charging
- Fan on/off by ECT, A/C pressure and ECT-sensor fault (fan forced on).
- A/C clutch is cut at WOT and ECT > 115 °C.
- Alternator monitor: voltage < 11.8 V for 30 s while running → P0562.

---

## 6. Diagnostics (OBD on-board side)

### 6.1 DTC lifecycle (J1979 / CARB-style)
| Event | Result |
|---|---|
| Monitor fails, 1st trip | **Pending** DTC (Mode 07). Freeze frame stored for misfire and fuel-system faults. |
| Same monitor fails, 2nd consecutive trip | **Confirmed** DTC (Mode 03), **MIL on**, freeze frame stored (Mode 02), **permanent** DTC (Mode 0A) |
| Catalyst-damaging misfire | MIL **flashes** while it occurs. Confirmed after 1 trip if it occurs again. |
| Monitor passes on 3 consecutive trips | MIL off. DTC kept as history. |
| 40 warm-up cycles with no recurrence | DTC erased |
| Pending, then passes the next trip | Pending cleared |
| Mode 04 (clear) | Clears confirmed and pending DTCs, freeze frame, Mode 06 results and monitor status (all incomplete). Resets distance/time since clear and warm-up count. **Permanent DTCs stay** until the monitor passes. LTFT reset is configurable (default: keep). |
| Battery disconnect (UI button) | Clears keep-alive memory: adaptives, idle learn, DTCs, monitors. Permanent DTCs stay (non-volatile). |

- **Warm-up cycle:** ECT rises ≥ 22 °C since start and reaches ≥ 70 °C.
- **Trip / drive cycle:** key-on, engine start, and the monitor's enabling conditions met, then key-off.

### 6.2 Readiness monitors (spark ignition)
| Monitor | Type | Enable conditions (simplified realistic) | Completes when |
|---|---|---|---|
| Misfire | Continuous | ECT > 0 °C, rpm 450–6500, not in fuel cut, no rough-road flag | 1000 revs evaluated |
| Fuel system | Continuous | Closed loop, ECT > 70 °C | 60 s of closed loop |
| Comprehensive components | Continuous | Key-on | Rationality tests done (MAF vs MAP, ECT warm-up rate, TPS/APP, IAT vs ECT at cold start) |
| Catalyst | Non-continuous | Warm, closed loop, cat > 450 °C, steady cruise 40–100 km/h, ≥ 60 s | Oxygen-storage index computed (downstream/upstream switch ratio, or rich/lean step test) |
| Heated catalyst | Not supported | | |
| EVAP | Non-continuous | Cold start (ECT − IAT < 7 °C), ambient 4–35 °C, fuel 15–85%, steady cruise or idle after > 10 min run | Vacuum-decay test done |
| Secondary air | Not supported | | |
| A/C refrigerant | Not supported | | |
| O2 sensor | Non-continuous | Closed loop, warm. Response test during a decel fuel-cut event and on rich/lean steps | B1S1 response + B1S2 switch tests pass |
| O2 sensor heater | Non-continuous | Cold start | Heater warm-up time measured |
| EGR / VVT | Non-continuous | Oil > 60 °C, cruise | VVT tracking test done |

Every monitor has a **"Force complete / Force incomplete / Auto"** selector in the UI. A **drive-cycle helper** shows which enable conditions are still missing, for example "Catalyst: need steady 40–100 km/h for 47 more seconds".

### 6.3 DTC catalogue (v1)
| Code | Description | Detected by |
|---|---|---|
| P0010 / P0011 / P0012 | Intake cam actuator circuit / over-advanced / over-retarded | VVT monitor |
| P0016 | Crank/cam correlation | CKP/CMP phase |
| P0030 / P0036 | HO2S heater control circuit B1S1 / B1S2 | Circuit check |
| P0101 / P0102 / P0103 | MAF range/performance, low, high | MAF vs MAP-model rationality, voltage limits |
| P0106 / P0107 / P0108 | MAP range/performance, low, high | MAP vs baro at key-on, vs MAF model |
| P0111 / P0112 / P0113 | IAT range/performance, low, high | Cold-start comparison with ECT, limits |
| P0116 / P0117 / P0118 | ECT range/performance, low, high | Warm-up rate model, limits |
| P0121 / P0122 / P0123 | TPS A range/performance, low, high | |
| P0221 / P0222 / P0223 | TPS B range/performance, low, high | |
| P0128 | Coolant below thermostat regulating temperature | Warm-up model vs actual |
| P0130 / P0131 / P0132 / P0133 / P0134 / P0135 | B1S1 circuit / low / high / slow / no activity / heater | |
| P0136 / P0137 / P0138 / P0139 / P0140 / P0141 | B1S2 circuit / low / high / slow / no activity / heater | |
| P2195 / P2196 | B1S1 signal stuck lean / rich | Downstream disagrees with upstream |
| P0171 / P0172 | System too lean / rich | Total trim > +25% / < −25% for 30 s |
| P0201–P0204 | Injector 1–4 circuit | Driver feedback |
| P0300 / P0301–P0304 | Random / cylinder-specific misfire | Crank-speed variance |
| P0325 / P0327 / P0328 | Knock sensor circuit / low / high | |
| P0335 / P0336 | CKP circuit / range | |
| P0340 / P0341 | CMP circuit / range | |
| P0351–P0354 | Ignition coil 1–4 primary | Dwell feedback |
| P0420 | Catalyst efficiency below threshold B1 | Catalyst monitor |
| P0441 | EVAP incorrect purge flow | |
| P0442 / P0455 / P0456 | EVAP leak small / large / very small (US) | EVAP monitor |
| P0443 | Purge valve circuit | |
| P0446 | Vent valve performance | |
| P0496 | EVAP high purge flow | Tank vacuum with purge commanded off |
| P0451 / P0452 / P0453 | Tank pressure sensor performance / low / high | |
| P0461 | Fuel level sensor range/performance | |
| P0480 | Fan 1 control circuit | |
| P0500 | Vehicle speed sensor | VSS vs load and rpm |
| P0506 / P0507 | Idle rpm lower / higher than expected | |
| P0562 / P0563 | System voltage low / high | |
| P2101 | Throttle actuator control motor performance | |
| P2122 / P2123 / P2127 / P2128 | APP D / E low / high | |
| P2135 | TPS A/B correlation | |
| P2138 | APP D/E correlation | |

### 6.4 Mode 06 (on-board monitor test results)
| OBDMID | Test | TIDs |
|---|---|---|
| 0x01 | O2 B1S1 | Response rate (lean→rich, rich→lean) |
| 0x02 | O2 B1S2 | Max/min voltage, switch time |
| 0x21 | Catalyst B1 | Oxygen-storage index |
| 0x35 | VVT B1 | Tracking error |
| 0x39 / 0x3A / 0x3B / 0x3C | EVAP 0.150" / 0.090" / 0.040" / 0.020" | Decay rate |
| 0x3D | Purge flow | |
| 0x41 / 0x42 | O2 heater B1S1 / B1S2 | Warm-up time / resistance |
| 0x81 | Fuel system B1 | Trim values |
| 0xA1 | Misfire general | Total count |
| 0xA2–0xA5 | Misfire cylinder 1–4 | TID 0x0B (exponentially weighted average, last 10 cycles), 0x0C (current/last cycle) |

Each result is reported as test value, minimum and maximum, with standardised unit/scaling IDs (UASIDs).

### 6.5 Freeze frame (Mode 02, frame 0)
Stores the DTC that triggered it plus PIDs 03, 04, 05, 06, 07, 0B, 0C, 0D, 0E, 0F, 10, 11, 1F, 2F, 33, 42, 44, 46. Misfire and fuel-system faults replace an older freeze frame. Other faults never do.

---

## 7. Diagnostic server (external CAN side)

### 7.1 Addressing and bus
| Setting | Default | Options |
|---|---|---|
| Bitrate | 500 kbit/s | 250 kbit/s |
| ID format | 11-bit | 29-bit |
| Functional request | `0x7DF` | `0x18DB33F1` |
| Physical request / response | `0x7E0` / `0x7E8` | `0x18DA10F1` / `0x18DAF110` |
| Frame padding | `0xAA` | `0x00`, `0x55`, `0xCC`, off (DLC-trimmed) |
| Silent when key off | yes | |

### 7.2 ISO-TP (ISO 15765-2)
- Single frame, first frame, consecutive frame and flow control.
- Honour the tester's block size (BS) and minimum separation time (STmin), including STmin values in the 0xF1–0xF9 range, which mean 100–900 µs.
- Timeouts: N_Bs = 1000 ms waiting for flow control. Abort when the time is exceeded.
- Receiving multi-frame requests is also supported, for future UDS.

### 7.3 OBD request rules (J1979)
- Mode 01 / 02 / 09 requests may carry **up to 6 PIDs**. The ECU answers with only the supported ones, in one response (multi-frame if needed).
- Unsupported PID or service on a **functional** request → **no response**.
- On a **physical** request → negative response `7F <SID> 12` (sub-function not supported) or `7F <SID> 11` (service not supported).
- Mode 04 with the engine running → `7F 04 22` (conditions not correct). Configurable, since some ECUs allow it.
- Support bitmaps for PIDs 00, 20, 40, 60, 80, A0 are generated from the PID registry.

### 7.4 Mode 01 PID list (scaling per SAE J1979)
| PID | Name | Bytes | Scaling |
|---|---|---|---|
| 00/20/40/60 | Supported PIDs | 4 | Bitmap |
| 01 | Monitor status since DTCs cleared | 4 | A7 = MIL, A0–6 = DTC count, B = continuous monitors, C/D = non-continuous supported / incomplete |
| 03 | Fuel system status | 2 | A: 1 = OL temp, 2 = CL, 4 = OL drive, 8 = OL fault, 16 = CL fault. B = 0. |
| 04 | Calculated engine load | 1 | A × 100/255 % |
| 05 | Coolant temperature | 1 | A − 40 °C |
| 06 / 07 | STFT / LTFT bank 1 | 1 | (A − 128) × 100/128 % |
| 0B | Intake manifold absolute pressure | 1 | A kPa |
| 0C | Engine RPM | 2 | (256A + B)/4 |
| 0D | Vehicle speed | 1 | A km/h |
| 0E | Timing advance | 1 | A/2 − 64 ° |
| 0F | Intake air temperature | 1 | A − 40 °C |
| 10 | MAF air flow | 2 | (256A + B)/100 g/s |
| 11 | Absolute throttle position | 1 | A × 100/255 % |
| 13 | O2 sensors present | 1 | 0x03 (B1S1, B1S2) |
| 15 | O2 B1S2 voltage + trim | 2 | A/200 V, (B − 128) × 100/128 % |
| 1C | OBD standard | 1 | Region: 0x01 OBD-II (CARB) / 0x06 EOBD / 0x09 EOBD + OBD + OBD-II |
| 1F | Run time since start | 2 | s |
| 21 | Distance with MIL on | 2 | km |
| 24 | O2 S1 wideband lambda + voltage | 4 | 2/65536 × (256A + B), 8/65536 × (256C + D) V |
| 2E | Commanded EVAP purge | 1 | A × 100/255 % |
| 2F | Fuel tank level | 1 | A × 100/255 % |
| 30 | Warm-ups since codes cleared | 1 | count |
| 31 | Distance since codes cleared | 2 | km |
| 32 | EVAP vapour pressure | 2 | signed (256A + B)/4 Pa |
| 33 | Barometric pressure | 1 | A kPa |
| 34 | O2 S1 wideband lambda + current | 4 | 2/65536 × (256A + B), (256C + D)/256 − 128 mA |
| 3C | Catalyst temperature B1S1 | 2 | (256A + B)/10 − 40 °C |
| 41 | Monitor status this drive cycle | 4 | Same layout as PID 01 |
| 42 | Control module voltage | 2 | (256A + B)/1000 V |
| 43 | Absolute load | 2 | (256A + B) × 100/255 % |
| 44 | Commanded equivalence ratio | 2 | 2/65536 × (256A + B) |
| 45 | Relative throttle position | 1 | A × 100/255 % |
| 46 | Ambient air temperature | 1 | A − 40 °C |
| 47 | Absolute throttle position B | 1 | A × 100/255 % |
| 49 / 4A | Accelerator pedal position D / E | 1 | A × 100/255 % |
| 4C | Commanded throttle actuator | 1 | A × 100/255 % |
| 4D | Time run with MIL on | 2 | min |
| 4E | Time since codes cleared | 2 | min |
| 51 | Fuel type | 1 | 0x01 gasoline |
| 5C | Engine oil temperature | 1 | A − 40 °C |
| 5E | Engine fuel rate | 2 | (256A + B)/20 L/h |

Warm idle fuel rate ≈ 0.85 L/h.

### 7.5 Mode 09 (vehicle information)
| PID | Item | Notes |
|---|---|---|
| 00 | Supported | |
| 02 | VIN | 17 ASCII characters plus a leading count byte 0x01 = 20-byte multi-frame response. The VIN is user-editable and the check digit is validated. |
| 04 | Calibration ID | Up to 16 ASCII characters, e.g. `SIM16DOHC-E0101` |
| 06 | CVN | 4 bytes, computed over the "calibration" (hash of the config tables) |
| 08 | In-use performance tracking (spark) | Counters for catalyst, O2, EVAP, EGR/VVT, secondary O2, plus ignition and general denominators |
| 0A | ECU name | 20 bytes, `ECM\0-EngineControl` padded |

### 7.6 Other modes
- **03 / 07 / 0A:** a count byte, then 2 bytes per DTC.
- **04:** clear, with the conditions in 6.1 and 7.3.
- **02:** frame 0 only. PID 02 returns the DTC that caused the freeze frame.
- **06:** see 6.4.

### 7.7 ELM327 adapter emulation (M7)

**Goal:** diagnostic software that expects an ELM327 adapter (USB, Bluetooth or Wi-Fi) can connect to ecusim
with **no CAN hardware**. ecusim pretends to be the adapter *and* the car:

```
diagnostic app ──ASCII──▶ transport (TCP / pty / COM / Bluetooth) ──▶ ELM327 emulator ──▶ ObdServer.handle()
               ◀─ASCII──                                           ◀── (formats replies) ◀──
```

The emulator runs **inside ecusim** and calls the same `ObdServer` as the CAN side, so every mode, PID, DTC,
freeze frame and Mode 06 result is identical. CAN and ELM327 can be on at the same time.

#### 7.7.1 Transports: each one switched on or off separately
Every transport is **off by default**. Each one is a separate emulated adapter with its own AT settings (echo,
headers, …), like plugging in several real adapters. Each one accepts **one client at a time**, as a real adapter
does. A second client is refused, with the reason shown in the UI.

| Transport | OS | How the app connects | Settings |
|---|---|---|---|
| **TCP ("Wi-Fi ELM327")** | all | Apps with a Wi-Fi adapter option (Torque, Car Scanner, OBD Auto Doctor), python-OBD with `socket://host:port` | Port (default **35000**). **Listen on:** this computer only (`127.0.0.1`, default) or the network (`0.0.0.0`, for phones on the classroom Wi-Fi). When on the network, the UI shows the IP address to type into the phone. |
| **Virtual serial port (pty)** | Linux, macOS | Any program that opens a serial port, including Windows tools under Wine (`~/.wine/dosdevices/comN` symlink) | Link name (default `/dev/ttyELM327` if writable, otherwise `~/.local/share/ecusim/ttyELM327`). The link is removed when the transport is switched off. |
| **Existing serial port** | all | **Windows:** a com0com virtual cable pair (ecusim opens COM11, the app opens COM10). Also a real USB-serial null-modem cable to a second PC running the scan software. | Port, baud rate (default 38400) |
| **Bluetooth (SPP)** | Linux; Windows via a COM port | Android apps that pair with "OBDII" Bluetooth adapters | **Linux:** ecusim registers a Serial Port Profile with BlueZ over D-Bus and appears as an adapter named (default) `OBDII`. Pairing is done by the user in the OS Bluetooth settings. Needs an optional D-Bus package; the option is greyed out with the reason if it's missing. **Windows:** create an *incoming* COM port in Windows Bluetooth settings, then use **Existing serial port**. **macOS:** not supported. |

Notes:
- **iPhone/iPad** apps don't use classic Bluetooth serial; they use Wi-Fi (TCP transport) or BLE. BLE adapter
  emulation is out of scope (see §14).
- Ports that can't be opened (in use, no permission, missing driver) are reported in the UI and leave the other
  transports running.

#### 7.7.2 User controls
- **UI:** a new **ELM327** tab. One row per transport with an **On/Off switch**, its settings, a status
  (`off` / `waiting for app` / `connected: 192.168.1.23` / error text) and request/response counters.
  Emulator options are shared by all transports: identity string, default protocol, response latency.
- **Trace:** ELM327 traffic appears in the **CAN** tab trace, tagged `ELM`, both as the ASCII exchange and as
  the CAN frames a real adapter would have put on the bus. Students still see ISO-TP, without a CAN adapter.
- **Command line:** `--elm-tcp [HOST:]PORT`, `--elm-pty [PATH]`, `--elm-serial PORT[@BAUD]`, `--elm-bt [NAME]`.
  Any of them switches that transport on for that session. The same flags work in `headless` mode.
- **Persistence:** which transports are on and their settings are saved in `config.json`, so a classroom PC
  starts with them ready. A `--no-elm` flag starts with all of them off.

#### 7.7.3 What the emulator implements
Copy an ELM327 **v1.5** (the version clones report and most apps expect). The identity string can be changed,
because some apps look for v2.1.

- **Reset/identity:** `ATZ`, `ATWS`, `ATD`, `ATI`, `AT@1`, `ATRV` (battery voltage from the simulation, so it
  rises to ~14.2 V when the engine runs).
- **Formatting:** `ATE0/1` echo, `ATL0/1` line feeds, `ATS0/1` spaces, `ATH0/1` headers, `ATCAF0/1` automatic
  formatting, `ATD0/1` show DLC, `ATR0/1` responses.
- **Protocol:** `ATSP n`, `ATTP n`, `ATDP`, `ATDPN`. Protocol 0 (auto) "searches" on the first request
  (`SEARCHING...`), then settles on **6** (11-bit, 500k) or **7** (29-bit, 500k), whichever ecusim's CAN
  settings say. Protocols 8/9 (250k) are answered when ecusim is set to 250 kbit/s. A CAN protocol with the
  wrong ID format → `NO DATA`, the wrong bitrate → `CAN ERROR`. ISO 9141 / KWP → `BUS INIT: ...ERROR`, J1850
  and the rest → `NO DATA`. With the key off an automatic search ends in `UNABLE TO CONNECT`.
- **Addressing:** `ATSH` (functional `7DF` or physical `7E0`; the server's functional/physical rules apply),
  `ATCRA`, `ATCF`/`ATCM`. An unknown header gets `NO DATA`.
- **Timing:** `ATST`, `ATAT0/1/2`; a request ending with a response count (`010C1`) returns after that many
  replies, which fast apps rely on.
- **Replies:** single and multi-frame (headers off: `014` / `0: 49 02 01 …` / `1: …`; headers on: one line per
  CAN frame), negative responses (`7F 01 12`), `NO DATA` (unsupported PID on a functional request, or key off),
  `?` for unknown commands, `STOPPED` when the app sends a character while a request is running, the `>`
  prompt after every reply.
- **Not implemented:** `ATMA`/`ATMR`/`ATMT` bus monitoring (answered `?`), STN/OBDLink `ST…` commands,
  programmable parameters (`ATPP`), J1939. Apps that require them (FORScan, OBDLink app) are out of scope.

#### 7.7.4 Realism options
- **Response latency:** per-request delay, default 40 ms (a real ELM327 over USB), 0 for fast testing.
- **Serial speed:** on serial transports, replies are paced at the configured baud rate, so long multi-frame
  answers take as long as on a real adapter.
- **Key off:** the "car" stops answering (`NO DATA` / `UNABLE TO CONNECT`) but the adapter still does: `ATRV`
  still reads the battery voltage, like an adapter powered from the OBD socket.

#### 7.7.5 Code structure
- `ecusim/elm/emulator.py`: pure state machine, **no I/O**: bytes in, text out. It reports how long to
  "wait for the car" and the CAN frames a real adapter would exchange (for the trace).
- `ecusim/elm/transports.py`: TCP, pty, serial and Bluetooth transports, one thread each, each feeding its own
  emulator instance. Bluetooth uses `jeepney` (pure-Python D-Bus) to register the profile with BlueZ.
- `ecusim/elm/manager.py`: settings, on/off, `config.json`, traces. `ecusim/elm/cli_args.py`: the flags.
- `ecusim/ui/elm_panel.py`: the ELM327 tab.

#### 7.7.6 Tests
- Unit tests of the state machine against recorded real-ELM327 transcripts (init sequences of python-OBD,
  Torque and ScanMaster; every formatting combination; multi-frame VIN; negative responses; `STOPPED`).
- python-OBD end-to-end over TCP and pty (`obd.OBD("socket://127.0.0.1:35000")`, `obd.OBD("/dev/ttyELM327")`).
- Manual: Torque / Car Scanner on Android over Wi-Fi and Bluetooth; ScanMaster-ELM on Windows through com0com.

---

## 8. Fault injection

Faults are applied **to the plant or the sensor/wiring layer only**. The ECU must discover them. Each fault has:
- an **on/off** toggle;
- a **severity** 0–100% where that makes sense;
- a **mode** for sensor faults: open, short to ground, short to 5 V/12 V, offset, gain error, stuck at current value, slow response, noisy, intermittent;
- optionally an **intermittency** setting (duty % and period).

### 8.1 Fault catalogue (v1)
| Group | Fault | Physical effect | Expected outcome |
|---|---|---|---|
| Air | Vacuum leak (size) | Unmetered air after MAF | Lean, LTFT up (mostly at idle), rough or high idle → P0171, possibly P0507 |
| | Dirty MAF (gain −5…−25%) | MAF under-reads, more so at high flow | Lean at load, MAF/MAP rationality → P0171, P0101 |
| | Restricted air filter | ΔP before throttle | Lower WOT MAP, reduced power. Usually no DTC. |
| | Dirty throttle body | Reduced effective area near closed | Idle drop or stall on clutch-in, idle learn compensates → P0506 |
| | PCV hose disconnected | Large unmetered air | P0171, high idle |
| Fuel | Low fuel pressure (severity) | Less fuel per ms | Lean at load, then all ranges → P0171, possible misfire at WOT |
| | Clogged injector n | −x% fuel on cylinder n | Lean cylinder, misfire at high severity → P030n, P0171 |
| | Leaking injector n | Extra fuel, also after key-off | Rich, hard hot start, P0172 |
| | Injector n open circuit | No fuel on cylinder n | P020n + P030n, MIL flashing |
| | Purge valve stuck open | Constant purge flow | Rich at idle after refuel, rough hot idle → P0496, P0172 |
| | Out of fuel / low fuel | Fuel pressure collapses | Lean misfire, then stall |
| | Bad fuel (low octane) | Lower KLSA | Knock retard visible, reduced power, no DTC |
| Ignition | Coil n failed (primary) | No spark on cylinder n | P035n + P030n |
| | Worn plug n (severity) | Misfire probability rises with load | P030n at load only |
| | Random misfire | Probability spread across cylinders | P0300 |
| Mechanical | Low compression cylinder n | Less torque, misfire at idle | P030n at idle |
| | Timing chain stretched (°) | Cam/crank offset | P0016, VVT learning offset |
| | VVT solenoid stuck | Cam stuck at an angle | P0011 / P0012, power loss |
| Cooling | Thermostat stuck open | Bypass to radiator | Slow warm-up, P0128, longer open loop |
| | Thermostat stuck closed | No radiator flow | Overheat, fan high, temp lamp, A/C cut |
| | Fan failure | No fan | Overheats at idle or in traffic → P0480 |
| Exhaust | Catalyst degraded (oxygen-storage health %) | Less oxygen storage | B1S2 follows B1S1 → P0420 |
| | Exhaust leak before B1S1 | O2 dilution at sensor | False lean, ECU runs rich, B1S2 rich → P2195 / P0172 |
| EVAP | Leak 0.5 / 1.0 mm / gross (cap off) | Orifice in EVAP system | P0456 / P0442 / P0455 after the EVAP monitor runs |
| | Vent valve stuck open | Cannot seal | P0446 |
| Electrical | Weak battery | Higher internal resistance | Slow crank, low voltage dip, possible no-start |
| | Alternator failure | No charging | Battery lamp, voltage falls, P0562 |
| | Over-voltage regulator | 15.5–16 V | P0563 |
| Sensors | ECT, IAT, MAF, MAP, TPS A/B, APP D/E, B1S1, B1S2, knock, CKP, CMP, tank pressure, fuel level, VSS | Per the sensor fault modes above | Each sensor's circuit DTCs, rationality DTCs and fail-safe behaviour (e.g. ECT open → default 80 °C, fan on, P0118) |
| Heaters | B1S1 / B1S2 heater open | Slower or no light-off | Longer open loop, P0135 / P0141 / P0030 / P0036 |

### 8.2 Scenario presets
One-click setups combining environment, faults and actions:
- "Cold morning, healthy car": 15 °C soak, full readiness.
- "Freshly cleared codes": all monitors incomplete. Shows the drive-cycle helper.
- "Vacuum leak at idle", "Tired catalyst", "Misfire cylinder 3 under load", "Stuck-open thermostat in winter (0 °C)", "Gas cap left off".
- User-defined presets saved to JSON.

---

## 9. User interface

### 9.1 Layout (main window)
```
┌──────────────────────────────────────────────────────────────────────────┐
│ [CAN: slcan /dev/ttyACM0 500k 11-bit ● online]  [Time ×1 ▸ ❚❚]  [Preset ▾] │
├──────────────────────────────────────────────────────────────────────────┤
│                  GAUGE CLUSTER                                            │
│   ( Tachometer 0–8000 )   [MIL][BATT][OIL][TEMP][EPC]   ( Speedometer )   │
│        coolant temp ▮▮▮▯▯   fuel ▮▮▮▮▯   gear: 3   odo / trip            │
├───────────────────────────┬──────────────────────────────────────────────┤
│ DRIVER CONTROLS           │  Tabs: Live data │ Charts │ Faults │          │
│  Key: OFF ACC ON (START)  │        Diagnostics │ Monitors │ Bus trace │    │
│  Clutch ▮  Brake ▮  Gas ▮ │        Environment │ Vehicle info │ Settings │ │
│  H-pattern shifter 1–5 R  │                                              │
│  Handbrake · A/C · Lights │                                              │
│  Heater · Demister        │                                              │
└───────────────────────────┴──────────────────────────────────────────────┘
```

### 9.2 Driver controls
- **Ignition key:** OFF / ACC / ON / START. START is momentary and spring-returns to ON.
- **Pedals:** vertical sliders that can be dragged with the mouse or touch. They return to zero on release by default, with a "hold" lock option.
- **Keyboard:**

| Key | Action |
|---|---|
| ↑ / W | Accelerator, progressive ramp |
| ↓ / S | Brake |
| Space / C | Clutch |
| 1–5, R, N | Gears (needs clutch) |
| K | Key: hold = start |
| H | Handbrake |
| A | A/C |
| L | Lights |

- **Shifter:** clickable H-pattern widget.
- **Later:** gamepad, or a USB wheel and pedals through SDL.

### 9.3 Gauges
- Analog tachometer (redline 6500) and speedometer (0–240 km/h), drawn with QPainter so they scale with DPI.
- Coolant temperature and fuel level gauges, damped like a real cluster (the coolant needle sits mid-scale from ~75 to 105 °C).
- **Warning lamps:**
  - MIL: solid or flashing.
  - Battery, oil pressure, coolant overheat.
  - EPC / throttle (limp mode).
  - Glow of the key-on bulb check.
- **Extra gauge page:** MAP, lambda, STFT/LTFT bars, timing advance, injector pulse width and duty, VVT angle, knock retard per cylinder.

### 9.4 Panels
- **Live data:** every Mode 01 PID with engineering value, unit and raw hex bytes, plus ECU-internal values that OBD can't see (pulse width, VVT, knock retard, idle-speed state, LTFT cells, catalyst oxygen storage). **"Show truth" toggle** puts true plant values next to sensor and ECU values.
- **Charts:** up to 8 strip charts (pyqtgraph) with selectable signals, freeze and cursor readout, CSV export.
- **Faults:** grouped tree with toggle, severity slider, mode, intermittency, and a "clear all faults" button.
- **Diagnostics:**
  - Confirmed, pending and permanent DTCs with status, freeze frame and MIL state.
  - Mode 06 table.
  - Counters: warm-ups, distance/time since clear, distance with MIL.
  - Buttons: **Clear codes (internal)**, **Battery disconnect**, **Soak for N hours**.
- **Monitors:** each readiness monitor with status (supported / complete / incomplete / this cycle), Auto / Force complete / Force incomplete, and the drive-cycle helper hint.
- **Bus trace:** live CAN frames (RX/TX, ID, DLC, data, ISO-TP decode, OBD decode), filters, pause, candump-format export.
- **Environment:** ambient temperature (−30 to +45 °C), altitude/baro, fuel level, fuel octane, road grade, electrical load, cabin heater.
- **Vehicle info:** VIN (with check-digit validator), Calibration ID, CVN, ECU name, OBD region (US / EU), odometer.
- **Settings:** CAN interface, channel, bitrate, ID format, padding, response delay (for testing tester timeouts), Mode 04 policy, LTFT-reset-on-clear policy, units (metric / imperial in the UI only; OBD is always metric).

---

## 10. Persistence
| File | Contents | Survives |
|---|---|---|
| `config.json` | CAN settings, ELM327 transports and their settings, UI layout, vehicle info, region | Everything |
| `kam.json` (keep-alive memory) | LTFT cells, idle-air learn, purge learn, knock adaptives, DTCs (confirmed, pending, history), freeze frame, monitor status, counters, in-use performance counters, odometer | Key off. Wiped by Battery disconnect. |
| `nvm.json` | Permanent DTCs, VIN, CALID/CVN | Battery disconnect |
| `presets/*.json` | Scenario presets | |
| On exit | Thermal state and timestamp. On next launch, soak is computed from wall-clock time elapsed, capped. | |

---

## 11. Project structure (planned)
```
ecu/
  docs/DESIGN.md
  pyproject.toml
  ecusim/
    app.py                 # entry point
    core/
      clock.py             # sim time, acceleration, stepping
      snapshot.py          # immutable state published per step
      calib/               # tables (YAML/JSON): VE, spark, idle, enrichment, thresholds
    plant/
      air.py fuel.py combustion.py rotation.py thermal.py
      exhaust.py catalyst.py evap.py electrical.py vehicle.py
    sensors/
      base.py              # transfer functions, fault modes
      definitions.py       # every sensor's wiring and fault hooks
    ecu/
      modes.py torque.py idle.py fuelling.py spark.py vvt.py
      fan.py charging.py limp.py
      diag/
        dtc.py monitors.py freeze_frame.py mode06.py iumpr.py
        rationality.py misfire.py catalyst_mon.py o2_mon.py evap_mon.py
    faults/
      catalog.py presets.py
    obd/
      can_transport.py     # python-can wrapper, adapter discovery
      isotp.py
      server.py            # request routing, functional/physical rules
      services/
        mode01.py mode02.py mode03_07_0a.py mode04.py mode06.py mode09.py
      pids.py              # PID registry: id, length, encoder, supported flag
    persistence/
      kam.py nvm.py
    ui/
      main_window.py
      widgets/ gauge.py tach.py lamps.py pedal.py shifter.py key_switch.py
      panels/ live_data.py charts.py faults.py diagnostics.py monitors.py
              bus_trace.py environment.py vehicle_info.py settings.py
  tests/
    test_isotp.py test_pid_encoding.py test_obd_rules.py
    scenarios/ test_cold_start.py test_warmup.py test_faults_*.py
  tools/
    tester.py              # minimal OBD tester for vcan/virtual bus
```

---

## 12. Verification

### 12.1 Automated
- **Protocol unit tests:**
  - ISO-TP segmentation and reassembly, BS and STmin handling, timeouts.
  - Every PID encoder checked against J1979 worked examples.
  - Multi-PID requests.
  - Functional vs physical negative-response rules.
- **Headless scenario tests** (time-accelerated). Each asserts behaviour, for example:
  - Cold start at 15 °C: crank-to-run < 2 s, flare to 1400–1600 rpm, closed loop within 60 s, fuel status 1 → 2, cat > 300 °C within 45 s.
  - Warm-up at idle: ECT 15 → 90 °C in 10–16 min, idle 1150 → 750 rpm.
  - Healthy closed loop: |STFT| < 5%, |LTFT| < 3%, B1S2 0.6–0.75 V steady.
  - 1.5 mm vacuum leak: LTFT idle cell > +15% within 5 min. P0171 pending on trip 1, confirmed on trip 2 with MIL.
  - Catalyst at 20% health: P0420 pending after the first catalyst monitor run.
  - Misfire cylinder 3 at 20%: MIL flashing, P0303.
  - Mode 04: monitors all incomplete, DTCs gone, permanent DTC retained.
  - WOT 2nd gear pull: rev limiter at 6700, 0–100 km/h in a plausible 10–12 s.

### 12.2 With real tools
- Linux `vcan0` plus `tools/tester.py`, then a real adapter loopback (two adapters).
- python-OBD, ELM327-based phone apps (Torque, Car Scanner), OBDLink MX+/SX, and a generic handheld scan tool. Check protocol auto-detect, VIN read, readiness, DTC read and clear, freeze frame, Mode 06.
- Test 250 kbit/s and 29-bit modes against tools that probe every protocol.

---

## 13. Milestones
| # | Deliverable | Done when |
|---|---|---|
| M1 | Headless plant + core ECU (air, fuel, spark, idle, thermal, closed loop), plotting script | Cold-start and warm-up scenario tests pass |
| M2 | CAN transport, ISO-TP, Mode 01 / 09 server on vcan or virtual bus | python-OBD reads RPM, ECT and VIN live |
| M3 | Qt UI: driver controls, gauges, live data, charts, CAN setup, vehicle model | Can drive away, shift, stall, cruise |
| M4 | Diagnostics: DTC manager, monitors, freeze frame, Mode 02/03/04/06/07/0A, persistence | Two-trip logic and Mode 04 scenarios pass |
| M5 | Fault catalogue + presets, EVAP, catalyst and O2 monitors, drive-cycle helper | All fault scenario tests pass |
| M6 | Hardware validation on Linux, Windows and macOS with CANable and PCAN; packaging | Real scan tools work on all 3 OSes |
| M7 | ELM327 adapter emulation (§7.7): TCP, pty, serial, Bluetooth, each switchable | python-OBD works over TCP and pty; an Android app works over Wi-Fi and Bluetooth; a Windows app works through com0com |

---

## 14. Open items / future
- UDS (ISO 14229) on the same ISO-TP stack: `0x10`, `0x22` (VIN `F190`, pulse width, VVT and other internal values), `0x19`, `0x14`, `0x3E`, `0x27` with a simulated seed/key.
- Optional second ECU (TCM / ABS) to test tools that scan multiple modules.
- Automatic gearbox variant.
- Replaying recorded real-car logs to tune the calibration tables.
- Gamepad, wheel and pedal input.
- BLE (Bluetooth Low Energy) ELM327 emulation, for iPhone apps that only support BLE adapters.
