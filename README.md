# Volcaniarm Firmware

ESP32 firmware for the Volcaniarm motor controller. It drives two stepper motors via STEP/DIR pins, reads two limit switches for homing, and talks to the ROS 2 hardware interface over USB serial.

## Requirements

- ESP32 NodeMCU-32S (or compatible board)
- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)

## Pinout

| Function | GPIO |
|----------|------|
| Stepper 1 STEP / DIR (right joint) | 16 / 17 |
| Stepper 2 STEP / DIR (left joint)  | 18 / 19 |
| Right limit switch (NC to GND) | 25 |
| Left limit switch (NC to GND)  | 26 |
| Status LED (built-in) | 2 |

## Serial Protocol

USB serial at 115200 baud.

| Command sent to ESP | Meaning |
|--------------------|---------|
| `P1 <s1> P2 <s2>` | Move both motors in coordinated mode to step targets `s1`, `s2` |
| `P1<steps>` / `P2<steps>` | Move single motor to step target |
| `H` or `Z` | Run homing sequence (limit switches), reply `H 0 0` when done |

The ESP streams the current position every 20 ms as `S <s1> <s2>`.

## Build & Upload

### From the CLI

```bash
pio run                # build
pio run -t upload      # build + flash
pio device monitor     # view serial output
```

### From VS Code

Install the [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) extension and open this folder. Use the PlatformIO toolbar at the bottom of the window:

| Button | Action |
|--------|--------|
| ✓ (Build) | Compile the firmware |
| → (Upload) | Build and flash the connected ESP |
| 🔌 (Serial Monitor) | Open the serial monitor |
| 🗑 (Clean) | Wipe build artifacts |


## Related repos

- ROS 2 workspace: [volcaniarm_ws](https://github.com/LevinTamir/volcaniarm_ws)
