# Reference hardware for AWAKE 2027

This is a reliable starting architecture for the competition car. It is not the
only valid design, but it keeps the system buildable within the 900 € budget and
easy to model in the simulator.

## Architecture

```text
                     ┌────────────────────┐
                     │ Raspberry Pi 5     │
                     │ perception / plan  │
                     └───────┬────────────┘
                             │ UART / USB
                     ┌───────▼────────────┐
                     │ RP2040 / Teensy    │
                     │ real-time safety   │
                     └───┬──────┬─────┬───┘
                         │      │     │
                    steering  sonars encoders
                         │
                     ┌───▼──────────────┐
                     │ VESC             │
                     │ motor + telemetry│
                     └───┬──────────────┘
                         │
                      brushless motor
                         │
                    4WD drivetrain
```

The Pi is **not** in the hard real-time loop. The low-level board owns the
steering servo, ultrasonic timing, encoder capture, watchdog and safe stop.

## Recommended bill of materials

Prices are indicative EUR ranges, not quotations.

| Item | Suggested part | Qty | Est. price |
|---|---|---:|---:|
| Chassis | Tamiya TT-02 / TT-02D roller or equivalent 1/10 4WD touring car | 1 | 130–220 € |
| Motor | Sensored brushless 13.5T (or 17.5T for lower speed) | 1 | 45–80 € |
| Motor controller | VESC 6 / Flipsky 6.6 class | 1 | 60–110 € |
| Steering servo | Low-profile digital high-torque servo | 1 | 30–55 € |
| Computer | Raspberry Pi 5 8 GB | 1 | 80–100 € |
| Storage | 32 GB+ microSD | 1 | 10–20 € |
| Real-time board | RP2040 (Pico), Teensy 4.0 or STM32 | 1 | 5–35 € |
| Lidar | LD19 / LD06 class, mounted at 12–14 cm | 1 | 90–120 € |
| Ultrasonic | HC-SR04 or equivalent, 3–4 units | 3–4 | 10–20 € |
| IMU | BNO085 or ICM-42688 | 1 | 20–35 € |
| Encoders | AS5600 magnetic encoder + magnets | 2 | 10–20 € |
| Battery | 2S LiPo 5000 mAh, 30–50C | 1 | 50–80 € |
| Pi supply | 5 V / 5 A BEC, fused | 1 | 15–25 € |
| Servo supply | 6 V / 5 A BEC or suitable UBEC | 1 | 10–20 € |
| Wiring / protection | Fuse, switch, XT60, JST, connectors, standoffs | — | 40–70 € |
| Mechanical | Sensor mast, electronics tray, foam bumper, fasteners | — | 40–80 € |
| Spares | Spare low-level board / servo / cables | — | 50–80 € |

For a tight budget, choose the 17.5T motor and a cheaper Pi 4. A 2S battery is
recommended: it is safer indoors, easier on the drivetrain, and still fast enough
for a 5-lap reliability run.

## Sensor placement

- **Lidar**: 12–14 cm above the floor, below the 16 cm VMC pipe walls. Keep it
  clear of the sensor mast and wiring so it does not see its own structure.
- **Ultrasonic**: 8–10 cm above the floor, front corners angled outward, rear
  units for reversing recovery.
- **IMU**: close to the centre of mass, rigidly mounted, away from motor vibration.
- **Wheel encoders**: rear wheels or rear axle. VESC ERPM alone does not measure
  wheel slip.
- **Steering feedback**: AS5600 on the steering linkage. The stock servo does not
  report its actual angle.
- **Steering speed**: the servo's no-load speed is fixed by its internal control;
  for repeatable behaviour, implement a configurable command slew-rate limit in
  the low-level firmware. The simulator parameter is `steering.rate_deg_s`.
- **Camera**: optional; do not make it a dependency for the first working car.

## Power and safety

- Fuse the main battery lead.
- Separate the Pi BEC from the servo/motor supply.
- Common ground between the low-level board, VESC and sensors.
- Physical e-stop or a clearly accessible power switch.
- Low-voltage cutoff and a watchdog on the low-level board.
- Foam bumper and strain-relieved cables: wall contact is penalized.
- Bring a spare low-level board and spare servo to the competition.

## Competition constraints to preserve

- 1/10 scale, max 45 × 20 × 20 cm, min 5 mm ground clearance.
- Electric only; 4WD recommended, 2WD allowed.
- Minimum three ultrasonic sensors plus one lidar; GPS forbidden.
- Indoor 15 × 8 m track, 80–100 cm wide, 16 cm pipe walls, unknown shape.
- Five consecutive laps; wall deformation and interventions cost points.
