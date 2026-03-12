# F5L Model Aircraft Altimeter

## Hardware
- Seeed XIAO ESP32-C3 board.
- BMP280 barometric sensor breackout board.

## Interface
- BMP280: SPI with DMA.
- ESC output: RMT.
- Throttle input: GPIO ISR.

## Base framework
- ESP-IDF / PlatformIO (C++).

## Wiring XIAO → BMP280
### Option 1: to attach sensor directly on the back of the Xiao
- GPIO 20		        → SDO (MISO)
- GPIO 8		        → CSB (CS)
- GPIO 9		        → SDA (MOSI)
- GPIO 10		        → SCL (SCK)
- 3V3		            → VCC
- GND		            → GND
- All wires align, except for 3V3 and GND that are crossed, solder one of them from the other side.

### Option 2
- GPIO 8  (SCK)         → SCL/SCK
- GPIO 9  (MISO)        → SDO
- GPIO 10 (MOSI)        → SDA/SDI
- GPIO 20 (CS)          → CSB
- 3.3V                  → VCC
- GND                   → GND

## Wiring altimeter → RC receiver
- GPIO 3 (THR_IN_PIN)   → Receiver CH3 signal wire.
- GND                   → Receiver GND (← MUST share common ground!).
- WARNING!              → DO NOT connect receiver +5V output directly to XIAO 3.3V input, you will possibly burn the pin, add 10kΩ/10kΩ voltage divider instead, to lower the input voltage to a safe 2V5 range.
- Remember 3-pins servo colors code: RED → VCC, BLACK/BROWN → GND, WHITE/ORANGE → signal.

## Wiring altimeter → Motor ESC
- GPIO 21 (ESC_OUT_PIN) → ESC signal wire.
- GND                   → ESC GND (← MUST share common ground!).
- NOTICE                → no need for voltage divider or level shifter here, the 3V3 level output from the altimeter will be perfectly fine for any ESC nowadays.

## Protections
- Isolate the sensor board from the Xiao, to avoid temperature transfer. Use wood plate or stick tape.
- Cover the sensor area (the small metalic square) with a small wood / plastic cap, to protect it from direct airflow, leaving a small hole in a side for sensing.
- When soldering is finished, cover the entire altimeter with shrink tube, leaving only the two 3-pins connectors and the USB C end outside. Of course, make sure the covering have a small gap, we don't want the sensor to be completely sealed, just protected from direct airflow, temperature and sunlight.

## Signal flow
```
Receiver CH3 ──► GPIO ISR (pulse width measure) ──► state machine
                                                         │
                                                         ▼
                                                     RMT output
                                                         │
                                                         ▼
                                                        ESC
```

## State machine on code
### [GROUND]
- Sends PWM_MIN_US pulse periodicaly to the ESC, to keep motor disabled.
- Takes care of pressure drift compensation every 30 secs.
- If throttle > LAUNCH_THROTTLE_PCT, starts cutoff timer and changes to CLIMBING state.

### [CLIMBING]
- Passes throttle signal 1:1 from receiver to ESC.
- Tracks and keeps register of peak altitude.
- If any of:
    - elapsed >= MAX_MOTOR_TIME_S (30 secs)
    - peak altitude >= MAX_ALTITUDE_M (90 meters)
    - stick full-down for >= FULLDOWN_DEBOUNCE_MS  (manual cutoff)
- occurs, moves to COASTING state.

### [COASTING]
- Sends PWM_MIN_US pulse periodicaly to the ESC, to keep motor disabled.
- Writes flight data to NVS.

## NVS flight record
- Stored as a single binary blob under key "last_flight":
    - launch_ms         — ms since boot at launch.
    - cutoff_ms         — ms since boot at motor cutoff.
    - peak_motor_m      — highest altitude with motor running.
    - peak_flight_m     — highest altitude reached on the flight.
    - cutoff_altitude_m — altitude at moment of cutoff. 
    - alt_10s_m         — altitude 10 secs after cutoff.
    - cutoff_reason     — what triggered cutoff: TIME_LIMIT | ALT_LIMIT | MANUAL.

## Serial console output
- NVS flight record once at every boot, before waiting for launch, format:
    - Motor runtime (secs).
    - Cutoff reason.
    - Altitude peaks.
- Current status once per sec (1 Hz), format:
    - GROUND  | Alt:   +1.43 m   Thr:  12%
    - CLIMB   | Peak:  +45.30 m   T+12s   Thr:  87%
    - COAST   | Peak:  +67.43 m   Alt: +61.20 m
    - Set constant SERIAL_REPORT_ENABLED to enable / disable this log.

## Todo
    - Organize code in modules as usual (config, helpers, etc), it will be done after first testings.
