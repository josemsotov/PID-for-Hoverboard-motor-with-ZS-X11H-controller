# PID-for-Hoverboard-motor-with-ZS-X11H-controller
PID for Hoverboard motor with ZS-X11H controller.
In this video, I show how to implement a PID for a Hoverboard motor driven by a ZS-X11H controller.
https://youtu.be/sB7PSKbTVgw?si=Z2BokXZ8rifnfhNm
![Couv10](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/15543f20-b7f5-4e8c-bfe5-acd26c008c72)

![BancTests-L-2](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/9046e1f6-1d88-4253-a7c8-9d00167cbfef)
![CircuitBreaker2](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/574ce172-b206-4b01-b1c0-40b5936bf36b)
![Battery](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/9d9bdae3-2f71-43e5-b0c9-389608a6dc7b)
![MultiConnector](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/e0a2a6ca-954f-44ba-b024-46aba4faeecf)
![Microcontroller](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/2500bce7-f0f7-475b-9254-9131910dbbc1)
![CircuitBreaker2](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/3799d78a-a232-4161-aff6-5b0ebb81951f)
![ZS-X11H](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/06d46669-42ea-4a4a-906b-ddd56a0c21dc)
![Zoom](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/8495182d-9d14-425d-bab1-2c7d6b506b7f)
![PotPush](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/23c1fc33-3525-449e-b9ed-3c61904c1ec5)
![OnOffSwitch](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller/assets/31382964/b31fdaeb-2a40-42ed-be27-e38b7cc9174a)

## Dual Motor (Arduino Uno) Extension

This repository now includes an Arduino Uno dual-motor PID sketch (`DualMotor_PID.ino`) and a cross-platform Python GUI (`gui/controller.py`) to control a differential drive robot via USB Serial at 115200 baud.

### Pin Assignment (Arduino Uno)

- Left Motor
  - `HALL_L`: `2` (INT0)
  - `PWM_L`: `6` (PWM)
  - `DIR_L`: `7`
  - `BRAKE_L`: `10`
  - `STOP_L`: `12`
- Right Motor
  - `HALL_R`: `3` (INT1)
  - `PWM_R`: `9` (PWM)
  - `DIR_R`: `8`
  - `BRAKE_R`: `11`
  - `STOP_R`: `13`Reserved: `4`, `5` (not used). Serial USB `0`, `1` not used for motors.

### Physical Parameters

- Wheel diameter: 22 cm
- Pulses per revolution (hall): 55

### Serial Commands

- `ADELANTE v_mmps` — both wheels forward at v (mm/s)
- `ATRAS v_mmps` — both wheels backward at v
- `GIRAR_IZQ v_mmps` — left backward, right forward
- `GIRAR_DER v_mmps` — left forward, right backward
- `PARADA` — stop outputs and engage STOP (active LOW)
- `BRAKE` — engage electromagnetic brake (active HIGH)
- `VEL vL_mmps vR_mmps` — set individual target velocities (mm/s)
- `PWM pL pR` — open-loop PWM [0..255]
- `KP x`, `KI y`, `KD z` — tune PID gains (both wheels)
- `GET` — returns telemetry line starting with `DATA`

### Quick Start

1. Open and flash `DualMotor_PID.ino` to Arduino Uno (115200 baud).
2. Install Python requirements: `pip install pyserial`.
3. Run GUI:

```
python gui/controller.py
```

4. Select the COM port, click Connect. Use buttons to command motion. Telemetry appears in the text panel. The Arduino replies with `READY` on boot and `ACK/ERR` to commands.

