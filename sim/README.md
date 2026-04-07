# VESC FOC Simulator — Analisi dei Requisiti

Simulatore ROS2 del firmware VESC ([vedderb/bldc](https://github.com/vedderb/bldc)) per emulare il controllo FOC di un inverter Formula Student prima del deployment su hardware reale.

---

## Indice

1. [Contesto e obiettivo](#1-contesto-e-obiettivo)
2. [Architettura del firmware VESC](#2-architettura-del-firmware-vesc)
3. [Algoritmo FOC — principio di funzionamento](#3-algoritmo-foc--principio-di-funzionamento)
4. [Requisiti funzionali](#4-requisiti-funzionali)
5. [Requisiti non funzionali](#5-requisiti-non-funzionali)
6. [Parametri del sistema](#6-parametri-del-sistema)
7. [Sezioni della repo da emulare e da ignorare](#7-sezioni-della-repo-da-emulare-e-da-ignorare)
8. [Architettura del simulatore ROS2](#8-architettura-del-simulatore-ros2)
9. [Modalità operative e variabili di controllo](#9-modalità-operative-e-variabili-di-controllo)
10. [Dati necessari e come ottenerli](#10-dati-necessari-e-come-ottenerli)
11. [Roadmap di sviluppo](#11-roadmap-di-sviluppo)

---

## 1. Contesto e obiettivo

### Contesto

Il progetto riguarda lo sviluppo del firmware di controllo per l'inverter di un veicolo Formula Student elettrico. L'inverter è basato sul firmware open-source VESC (`vedderb/bldc`), che implementa il controllo FOC (Field Oriented Control) per motori PMSM/BLDC su microcontrollore STM32.

### Obiettivo

Realizzare un simulatore in ROS2 che riproduca fedelmente il comportamento del loop di controllo FOC del firmware VESC, con le seguenti finalità:

- Validare la logica di controllo (PI di corrente, observer, limiti, fault) prima del deployment su hardware reale
- Testare le transizioni tra i regimi operativi (trazione, coast, rigenerazione) in ambiente sicuro
- Permettere il tuning dei parametri di controllo con visualizzazione in tempo reale tramite PlotJuggler
- Fornire una base di test per l'integrazione con la ECU del veicolo via CAN

### Motivazione della scelta ROS2

ROS2 consente di strutturare il simulatore come insieme di nodi disaccoppiati, comunicanti via topic, con integrazione nativa con PlotJuggler per la visualizzazione delle variabili interne del loop FOC. Permette inoltre di riusare l'infrastruttura di comunicazione (es. topic `/iq_ref`) identica a quella che verrà usata in produzione.

---

## 2. Architettura del firmware VESC

Il firmware è scritto in C e gira su ChibiOS/RT come sistema operativo real-time su STM32F4/F7. La struttura a layer è la seguente:

```
ChibiOS/RT
└── main.c                  — boot, init periferiche
    ├── /motor              — algoritmi di controllo motore
    │   ├── mc_interface.c  — API unificata (set_current, limiti, fault)
    │   ├── mcpwm_foc.c     — loop FOC principale (ISR ~20 kHz)
    │   ├── foc_math.c      — Clarke, Park, SVPWM (funzioni pure)
    │   └── /encoder        — Hall, ABI, SPI (AS5047, ecc.)
    ├── /comm               — CAN, UART, USB, BLE
    │   ├── comm_can.c
    │   └── commands.c      — dispatcher comandi
    ├── /applications       — logica applicativa
    │   ├── app_can.c       — setpoint via CAN
    │   └── app_ppm.c
    ├── /hwconf             — configurazione hardware (pinout, shunt)
    ├── datatypes.h         — struct mc_configuration e tipi globali
    └── conf_general.h      — parametri compile-time
```

---

## 3. Algoritmo FOC — principio di funzionamento

Il loop FOC gira come ISR a ~20 kHz in `mcpwm_foc.c`. Ad ogni ciclo esegue la seguente catena:

```
ADC (Ia, Ib, Ic)
  → Clarke transform     Ia,Ib,Ic  →  Iα, Iβ
  → Park transform       Iα,Iβ,θ   →  Id, Iq
  → PI Id                errore Id  →  Vd
  → PI Iq                errore Iq  →  Vq
  → Park inversa         Vd,Vq,θ   →  Vα, Vβ
  → SVPWM               Vα,Vβ      →  duty cycle U,V,W
  → Timer STM32          duty       →  gate driver → motore
```

La posizione elettrica del rotore `θ` è stimata ad ogni ciclo dall'**observer di Luenberger**, che integra il modello matematico del motore PMSM. A velocità prossima a zero, dove la back-EMF è nulla e l'observer diverge, subentra l'**HFI** (High Frequency Injection).

### Grandezze fondamentali

| Grandezza | Simbolo | Significato |
|-----------|---------|-------------|
| Asse d | Id | Componente di flusso della corrente (= 0 per PMSM in condizioni nominali) |
| Asse q | Iq | Componente di coppia: `τ = (3/2) · p · λ · Iq` |
| Flux linkage | λ | Concatenamento di flusso del rotore; lega Iq alla coppia |
| Posizione elettrica | θ | Angolo del rotore in radianti elettrici; stimato dall'observer |
| Velocità elettrica | ω_e | `ω_e = p · ω_m` con p = coppie polari |

---

## 4. Requisiti funzionali

### RF-01 — Modello fisico PMSM

Il simulatore deve integrare numericamente le equazioni differenziali del motore PMSM nel sistema di riferimento dq:

```
dId/dt = (Vd − R·Id + ω_e·Lq·Iq) / Ld
dIq/dt = (Vq − R·Iq − ω_e·Ld·Id − ω_e·λ) / Lq
dω_m/dt = (τ_e − τ_load − B·ω_m) / J
dθ_e/dt = p · ω_m
```

dove `τ_e = (3/2) · p · λ · Iq` (motore isotropo SPM).

### RF-02 — Loop di controllo FOC

Il simulatore deve replicare la catena di controllo del firmware:

- Trasformata di Clarke (`foc_math.c :: foc_clarke()`)
- Trasformata di Park (`foc_math.c :: foc_park()`)
- Regolatori PI indipendenti su Id e Iq con anti-windup
- Trasformata di Park inversa
- Modulazione SVPWM (`foc_math.c :: foc_svm()`)

### RF-03 — Observer di Luenberger

Il simulatore deve implementare `observer_update()` da `mcpwm_foc.c` per stimare θ e ω in modo sensorless, con parametro `foc_observer_gain` configurabile.

### RF-04 — Modalità operative

Il simulatore deve supportare le tre modalità operative principali e le relative transizioni:

| Modalità | Iq_ref | Comportamento |
|----------|--------|---------------|
| Trainante | > 0 | Coppia positiva, potenza da batteria a motore |
| Coast / idle | = 0 | Coppia nulla, gate driver attivi, observer mantenuto |
| Frenante / regen | < 0 | Coppia negativa, potenza da motore a batteria |

### RF-05 — Limiti e protezioni

Il simulatore deve riprodurre i limiti operativi di `mc_interface.c`:

- `l_current_max` / `l_current_min` — limite corrente lato motore
- `l_in_current_max` / `l_in_current_min` — limite corrente lato batteria
- `l_max_vin` / `l_min_vin` — protezione tensione bus (critica in regen)
- `l_max_erpm` — limite di velocità

### RF-06 — Field weakening

Il simulatore deve implementare la riduzione automatica di Id sotto zero quando `ω > ω_base`, per permettere il funzionamento oltre la velocità nominale.

### RF-07 — Watchdog / timeout

Il simulatore deve replicare il comportamento del timeout di `mc_interface.c`: se il setpoint `Iq_ref` non viene aggiornato entro il periodo configurato, il sistema transisce automaticamente in coast.

### RF-08 — Interfaccia CAN simulata

Il simulatore deve esporre un'interfaccia ROS2 equivalente al canale CAN fisico, accettando setpoint di corrente tramite topic e pubblicando i dati di stato (velocità, corrente, tensione bus) con la stessa periodicità del firmware reale.

### RF-09 — Pubblicazione variabili interne

Il simulatore deve pubblicare su topic ROS2 tutte le variabili interne del loop FOC per consentire l'analisi in PlotJuggler.

---

## 5. Requisiti non funzionali

### RNF-01 — Frequenza di simulazione

Il nodo `pmsm_plant` deve integrare le ODE a una frequenza almeno 10× superiore al loop PI. Con PI a 20 kHz (50 µs) il passo di integrazione deve essere ≤ 5 µs, oppure si usa un integratore RK4 a 50 µs con step intermedi.

### RNF-02 — Parametrizzazione via YAML

Tutti i parametri del motore e del controllore devono essere caricabili da file YAML, con struttura corrispondente ai campi della struct `mc_configuration` di `datatypes.h`.

### RNF-03 — Compatibilità PlotJuggler

I topic ROS2 devono usare tipi standard (`std_msgs/Float64`, `sensor_msgs/JointState`) compatibili con il plugin PlotJuggler per ROS2 senza configurazioni aggiuntive.

### RNF-04 — Separazione modello / controllore

Il modello fisico (`pmsm_plant`) e il controllore (`vesc_controller`) devono essere nodi separati, comunicanti solo via topic. Questo permette di sostituire `pmsm_plant` con dati reali dall'hardware quando disponibile, senza modificare il controllore.

---

## 6. Parametri del sistema

### 6.1 Parametri elettrici motore

| Parametro | Simbolo | Unità | Fonte | Note |
|-----------|---------|-------|-------|------|
| Resistenza di fase | R | Ω | XML / datasheet | Verificare convezione fase-fase o fase-neutro |
| Induttanza asse d | Ld | H | XML / datasheet | Ld = Lq per SPM |
| Induttanza asse q | Lq | H | XML / datasheet | Chiedere entrambi per IPM |
| Flux linkage | λ | Wb | **calcolato** | `λ = kt · 2/(3·p)` |
| Costante di coppia | kt | Nm/A | datasheet | Alternativa: Kv in RPM/V |
| Coppie polari | p | — | datasheet | Numero poli / 2 |
| Corrente nominale | I_nom | A | datasheet | Limite termico continuativo |
| Corrente di picco | I_peak | A | datasheet | Con durata ammessa |
| Tensione nominale | V_nom | V | datasheet | Range min/max bus DC |

### 6.2 Parametri meccanici

| Parametro | Simbolo | Unità | Fonte | Note |
|-----------|---------|-------|-------|------|
| Inerzia rotore | J_motore | kg·m² | datasheet / stima | Stima iniziale: 0.001–0.005 kg·m² |
| Rapporto trasmissione | n | — | team telaio | 1 se direct drive |
| Inerzia ruota | J_ruota | kg·m² | team telaio | Stima iniziale: ~0.5 kg·m² |
| Inerzia equivalente | J_tot | kg·m² | **calcolato** | `J_tot = J_motore + J_ruota/n²` |
| Attrito viscoso | B | Nm·s/rad | stima | Partire da 0 |
| Coppia di carico | τ_load | Nm | team telaio / stima | Partire da 0 per validazione loop corrente |
| Massa veicolo | m | kg | team telaio | Per stima τ_load in accelerazione |

### 6.3 Parametri configurazione VESC

Tutti ricavabili dal file `.xml` esportato da VESC Tool:

| Campo XML | Significato | Unità |
|-----------|-------------|-------|
| `foc_motor_r` | Resistenza di fase | Ω |
| `foc_motor_l` | Induttanza (media Ld, Lq) | H |
| `foc_motor_ld_lq_diff` | Differenza Ld − Lq | H |
| `foc_motor_flux_linkage` | Flux linkage λ | Wb |
| `foc_current_kp` | Guadagno proporzionale PI corrente | — |
| `foc_current_ki` | Guadagno integrale PI corrente | — |
| `foc_observer_gain` | Guadagno observer Luenberger | — |
| `foc_openloop_rpm` | Soglia open-loop a bassa velocità | ERPM |
| `l_current_max` | Limite corrente motore in trazione | A |
| `l_current_min` | Limite corrente motore in regen (negativo) | A |
| `l_in_current_max` | Limite corrente batteria in trazione | A |
| `l_in_current_min` | Limite corrente batteria in regen (negativo) | A |
| `l_max_erpm` | Velocità massima | ERPM |
| `l_min_vin` | Tensione bus minima | V |
| `l_max_vin` | Tensione bus massima | V |

### 6.4 Parametri derivati (calcolati internamente)

```python
lambda_flux  = kt * 2 / (3 * p)           # flux linkage da costante di coppia
J_tot        = J_motore + J_ruota / n**2  # inerzia equivalente al motore
omega_base   = (Vbus * 0.95) / (sqrt((Ld * I_max)**2 + lambda_flux**2) * p)  # velocità base FW
Kp           = L / tau_c                  # se non fornito dall'XML
Ki           = R / tau_c                  # tau_c tipicamente 1–2 ms
```

---

## 7. Sezioni della repo da emulare e da ignorare

### Da portare in ROS2

| File / modulo | Cosa estrarre |
|---------------|---------------|
| `motor/foc_math.c` + `.h` | `foc_clarke()`, `foc_park()`, `foc_park_inv()`, `foc_svm()` — funzioni pure, nessuna dipendenza HW |
| `motor/mcpwm_foc.c` | PI di corrente con anti-windup, `observer_update()`, clamping Vd/Vq al cerchio di tensione |
| `motor/mc_interface.c` | `set_current()`, gestione limiti, logica fault, timeout watchdog |
| `datatypes.h` | Struct `mc_configuration` — usare come schema per il file YAML dei parametri |

### Da adattare (sostituire con equivalente ROS2)

| File / modulo | Sostituzione |
|---------------|--------------|
| `/comm/comm_can.c` | Topic ROS2 `std_msgs/Float64` su `/iq_ref` |
| `/applications/app_can.c` | Nodo subscriber ROS2 (`ecu_sim`) |
| `timeout.c` | Timer ROS2 + logica watchdog nel nodo `vesc_controller` |

### Da ignorare completamente

| Modulo | Motivo |
|--------|--------|
| `ChibiOS_3.0.5/` | RTOS real-time, rimpiazzato dal loop ROS2 |
| `/hwconf` | Configurazione pin STM32, non rilevante in simulazione |
| `/driver` | Driver periferiche hardware (DMA, SPI, I2C) |
| `flash_helper.c` | Gestione flash STM32 per persistenza parametri |
| `/imu` | Sensore IMU fisico |
| `LispBM/` | Interprete Lisp per scripting embedded |
| `/blackmagic` | Debugger hardware |
| `irq_handlers.c` | Handler interrupt hardware |

---

## 8. Architettura del simulatore ROS2

```
┌─────────────────┐        /iq_ref (Float64)       ┌──────────────────────┐
│    ecu_sim      │ ──────────────────────────────> │  vesc_controller     │
│                 │                                  │                      │
│ Simula la ECU   │ <────────────────────────────── │ PI Id/Iq             │
│ del veicolo FS  │     /foc/state (custom msg)     │ Observer Luenberger  │
└─────────────────┘                                  │ Limiti e fault       │
                                                     │ Watchdog timeout     │
                                                     └──────────┬───────────┘
                                                                │ /foc/vd, /foc/vq
                                                                ▼
                                                     ┌──────────────────────┐
                                                     │   pmsm_plant         │
                                                     │                      │
                                                     │ ODE dq PMSM          │
                                                     │ Dinamica meccanica   │
                                                     │ Integrazione RK4     │
                                                     └──────────┬───────────┘
                                                                │ /foc/id, /foc/iq
                                                                │ /foc/theta_real
                                                                │ /foc/omega
                                                                ▼
                                                           PlotJuggler
```

### Topic ROS2 principali

| Topic | Tipo | Direzione | Contenuto |
|-------|------|-----------|-----------|
| `/iq_ref` | `Float64` | ecu_sim → controller | Setpoint coppia [A] |
| `/foc/id` | `Float64` | plant → controller | Corrente asse d misurata [A] |
| `/foc/iq` | `Float64` | plant → controller | Corrente asse q misurata [A] |
| `/foc/theta_real` | `Float64` | plant → controller | Posizione elettrica reale [rad] |
| `/foc/omega` | `Float64` | plant → controller | Velocità elettrica [rad/s] |
| `/foc/vd` | `Float64` | controller → plant | Tensione asse d [V] |
| `/foc/vq` | `Float64` | controller → plant | Tensione asse q [V] |
| `/foc/theta_est` | `Float64` | controller → * | Posizione stimata observer [rad] |
| `/foc/torque` | `Float64` | plant → * | Coppia meccanica erogata [Nm] |
| `/foc/pi_d_err` | `Float64` | controller → * | Errore PI asse d (debug windup) |
| `/foc/pi_q_err` | `Float64` | controller → * | Errore PI asse q |
| `/foc/vbus` | `Float64` | * → controller | Tensione bus DC [V] |

---

## 9. Modalità operative e variabili di controllo

### Variabili di stato del loop (cambiano ogni ciclo ISR)

```
Input:
  Iq_ref     — setpoint coppia [A], da CAN/topic
  Id_ref     — normalmente 0; negativo in field weakening
  Vbus       — tensione bus DC [V]

Stato stimato (observer):
  θ          — posizione elettrica rotore [rad]
  ω          — velocità elettrica [rad/s]

Stato interno PI:
  Id, Iq     — correnti dq misurate (output Clarke+Park)
  Vd, Vq     — uscite PI (tensioni di riferimento dq)
  integral_d — stato integratore PI asse d
  integral_q — stato integratore PI asse q
```

### Comportamento per regime operativo

| Regime | Iq_ref | Id_ref | Flusso potenza | Limite critico |
|--------|--------|--------|----------------|----------------|
| Trainante | > 0 | 0 (o < 0 per FW) | Batteria → Motore | `l_current_max`, `l_in_current_max` |
| Coast / idle | = 0 | 0 | Nessuno | `foc_openloop_rpm` (observer) |
| Regen / frenante | < 0 | 0 | Motore → Batteria | `l_max_vin` (protezione sovratensione bus) |

**Nota coast:** mantenere il loop FOC attivo con `Iq_ref = 0` (anziché disabilitare i gate) è preferibile in FS perché l'observer continua a stimare θ e ω, rendendo la transizione verso trazione o regen immediata e senza il transitorio di convergenza.

---

## 10. Dati necessari e come ottenerli

### Fonte primaria — file `.xml` VESC Tool

Il file di configurazione esportato da VESC Tool (menu Firmware → Export configuration) contiene tutti i parametri elettrici del motore già misurati e validati sull'hardware reale, i guadagni PI calcolati, e tutti i limiti operativi.

**Richiedere a:** responsabile elettrico / inverter del team.

Questo singolo file copre l'intera sezione 6.1 e 6.3 e permette di avviare il simulatore senza ulteriori misure.

### Parametri meccanici — stima iniziale accettabile

Per la validazione del loop di corrente (obiettivo primario) i parametri meccanici non sono bloccanti. Valori di partenza:

```yaml
J_tot:   0.005   # kg·m² — stima conservativa per motore FS
n:       1.0     # assumere direct drive se non noto
B:       0.0     # attrito viscoso — aggiungere in seguito
tau_load: 0.0   # coppia di carico — zero per test a vuoto
```

Raffinare iterativamente quando disponibili i dati reali dal team telaio.

---

## 11. Roadmap di sviluppo

### Fase 1 — Loop di corrente (solo XML necessario)

Implementare `pmsm_plant` con le ODE dq e `vesc_controller` con i PI. Alimentare θ_real direttamente dal modello (encoder perfetto). Verificare che Iq tracking `Iq_ref` con la banda attesa. Visualizzare `/foc/iq` vs `/foc/iq_ref` in PlotJuggler.

### Fase 2 — Observer sensorless

Sostituire θ_real con l'output di `observer_update()`. Confrontare `/foc/theta_est` vs `/foc/theta_real`. Verificare la convergenza e il comportamento a bassa velocità (HFI opzionale in questa fase).

### Fase 3 — Modalità operative e limiti

Implementare le transizioni trainante → coast → regen. Aggiungere i limiti di corrente e tensione bus. Testare il watchdog timeout. Verificare il comportamento in over-voltage durante regen (`l_max_vin`).

### Fase 4 — Field weakening e dinamica meccanica

Aggiungere la dinamica meccanica completa con J_tot e τ_load reali. Implementare il field weakening (Id < 0 per ω > ω_base). Testare cicli di accelerazione/frenata rappresentativi di un giro in pista.

### Fase 5 — Validazione su hardware

Confrontare le risposte simulate con dati reali dall'inverter. Affinare i parametri del modello. Trasferire la configurazione validata sul firmware reale.
