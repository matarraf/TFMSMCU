# Track Following & Maze Solver Micro-Controller Unit (TFMS MCU)

## Description

This repository contains the STM32 micro-controller firmware for the **Track Following & Maze Solver Project**.  
The MCU controls a small autonomous vehicle using IR line sensors, an ultrasonic distance sensor, a steering servo, and two DC motors.

The system continuously reads the IR sensors, decides the required steering direction, adjusts the servo angle, controls motor speed, and stops the vehicle if an obstacle is detected within the configured stopping distance.

***This work is part of the EEE8114 Group Project Design: Team 1 Track Following & Maze Solver Project.***

---

## Main Points of Interest

- `Core/Src/main.c`  
  Main application source file. Contains the control loop, sensor reading, motor control, servo control, ultrasonic sensing, and steering logic.

- `Core/Inc/main.h`  
  Header file containing GPIO pin definitions and function declarations generated/configured through STM32CubeIDE.

---

## Project Guidelines

The firmware is designed to:

- Follow a black track using 6 IR sensors.
- Steer using a PWM-controlled servo motor.
- Drive using two PWM-controlled DC motors.
- Detect obstacles using an ultrasonic sensor.
- Stop the motors when an obstacle is closer than the defined threshold.
- Support multiple track profiles through compile-time configuration.

---
### Key Capabilities
- **Autonomous Line Following**: Six IR sensors detect and follow black lines with adaptive steering
- **Obstacle Detection**: Ultrasonic sensor triggers emergency stop when objects approach
- **Adaptive Steering**: Smooth servo control with configurable turn intensities (hard, moderate, soft turns)
- **Dual Motor Control**: Independent left and right motor speed management
- **Noise Filtering**: Majority-voting sensor sampling reduces false detections
- **Real-Time Control Loop**: Fast, responsive decision-making at configurable frequencies
---

## Micro-Controller Unit

The project is based on an STM32 micro-controller using the STM32 HAL library.

### Main Peripherals Used

| Peripheral | Purpose |
|---|---|
| GPIO | IR sensors, motor direction pins, ultrasonic sensor pin (trigger) |
| TIM2 | Servo PWM output |
| TIM3 | Left and right motor PWM outputs |
| TIM4 | Microsecond timing for ultrasonic sensor echo|
| USART2 | UART communication/debugging |
| I2C1 | Reserved for I2C peripherals, such as OLED display |

---

## Pin Mapping

| Component | Pin |
|---|---|
| IR Sensor 1 | PC0 |
| IR Sensor 2 | PC1 |
| IR Sensor 3 | PC2 |
| IR Sensor 4 | PC3 |
| IR Sensor 5 | PC4 |
| IR Sensor 6 | PC5 |
| Servo PWM | PA0 |
| Left Motor PWM | PA6 |
| Right Motor PWM | PA7 |
| Left Motor Direction | PB4 |
| Right Motor Direction | PB5 |
| Ultrasonic Sensor | PB7 |
| USART TX | PA2 |
| USART RX | PA3 |
| OLED SCL | PB8 |
| OLED SDA | PB9 |

---

## IDE

This project was developed using:

- STM32CubeIDE
- STM32 HAL drivers
- C programming language

To run the project:

1. Open the project in STM32CubeIDE.
2. Build the project.
3. Connect the STM32 board using ST-Link.
4. Flash the firmware to the board.
5. Power the vehicle hardware and test on the selected track.

---

## Breakdown
 
### System Architecture
 
The firmware operates on a **sense-decide-act** cycle:
 
1. **SENSE**: Read IR sensors and ultrasonic distance
2. **DECIDE**: Determine steering command and motor speed
3. **ACT**: Apply servo angle, motor speed, and obstacle avoidance
### Hardware Configuration
 
#### Inputs
- **IR Sensors** (6 total, PC0–PC5)
  - Track 1: IR2 (left), IR5 (right) – single sensor per side
  - Track 2: IR2+IR3 (left), IR4+IR5 (right) – dual sensors per side
  - Detects black line against white background
  
- **Ultrasonic Sensor** (PB7)
  - Measures distance to obstacles (0–5 meters typical range)
  - Triggers motor stop if obstacle is closer than 100 mm (configurable)
#### Outputs
- **Servo Motor** (PA0, TIM2 PWM)
  - Controls steering angle (0–180°, 90° = straight)
  - Pulse width: 800–2400 µs
  
- **Drive Motors** (PA6, PA7 PWM + PB4, PB5 direction)
  - Speed control via PWM (0–255)
  - Direction via GPIO pins (forward/backward)
#### Control Parameters
 
Two tuning profiles are selectable via `TRACK_PROFILE` define:
 
**Track 1**:
- Motor speeds: Straight=50, Soft turn=45, Hard turn=45
- Servo angles: HARD_LEFT=20°, LEFT=40°, SOFT_LEFT=65°, STRAIGHT=90°, etc.
- IR sampling: 5 samples per cycle, 200 µs delay
- Servo step limit: 10° per control loop
- Control loop frequency: 100 Hz (10 ms delay)

**Track 2**:
- Motor speeds: Straight=40, Soft turn=45, Hard turn=45
- Servo angles: HARD_LEFT=5°, LEFT=25°, SOFT_LEFT=45°, STRAIGHT=90°, etc.
- IR sampling: 3 samples per cycle, 50 µs delay
- Servo step limit: 15° per control loop
- Control loop frequency: 1 kHz (1 ms delay)
### Core Functions
 
#### Servo Control
- `servo_set_pulse_us(uint16_t pulse_us)` – Low-level PWM pulse control
- `servo_set_angle(float angle)` – Angle-based interface (0–180°)
- Pulse width calculation: `1000 + (angle/180) × 1000` µs
#### Motor Control
- `motor_set_raw(uint32_t channel, uint8_t value)` – Raw PWM (0–255)
- `motor_set_left(uint8_t speed)` / `motor_set_right(uint8_t speed)` – Individual control
- `motor_set_both(uint8_t speed)` – Synchronized speed
- `motor_stop()` – Emergency stop
- `motor_set_forward_direction()` – Set GPIO pins for forward motion
#### IR Sensor Reading
- `ir_read_black(uint8_t idx)` – Single sensor digital read
- `ir_read_mask()` – Read all 6 sensors with majority-vote filtering
  - Takes `IR_SAMPLE_COUNT` samples per sensor
  - Applies `(count > half)` majority rule to reject noise
  - Returns 8-bit mask where bit i represents sensor i
#### Ultrasonic Measurement
- `us_trigger()` – Send 10 µs trigger pulse
- `us_echo_pulse_us(uint32_t timeout_us)` – Measure echo duration
- `us_read_distance_mm()` – Convert pulse width to millimeters
  - Formula: `distance = (pulse_width × 343) / 2000` (speed of sound ≈ 343 m/s)
  - Accounts for round-trip propagation
#### Steering Logic
- `steering_get_command(uint8_t ir_mask)` – Decode sensor mask to steering direction
  - Counts active sensors on left and right sides
  - Returns: HARD_LEFT, LEFT, SOFT_LEFT, STRAIGHT, SOFT_RIGHT, RIGHT, HARD_RIGHT, KEEP_LAST
  
- `steering_filter_command(steer_cmd_t new_cmd)` – Smooth command transitions
  - Requires `STEER_CMD_CONFIRM_COUNT` consecutive identical commands
  - Prevents jitter from sensor noise
  
- `steering_apply_command(steer_cmd_t cmd)` – Convert steering command to servo angle
  - Limits servo speed to `SERVO_MAX_STEP_PER_LOOP` degrees per cycle
  - Ensures smooth, natural motion
  
- `speed_for_steering(steer_cmd_t cmd)` – Adjust motor speed based on turn intensity
  - Hard turns → slowest speed
  - Straight → fastest speed
#### Main Control Loop (`control_loop()`)
1. Read IR mask and ultrasonic distance
2. Compute raw steering command from IR sensors
3. Filter command for stability
4. Determine motor speed based on steering demand
5. Check obstacle distance; stop if too close
6. Apply servo angle and motor speed
7. Sleep for `CONTROL_LOOP_DELAY_MS` milliseconds
### Initialization Flow
 
1. HAL and system clock initialization
2. GPIO, I2C, timer, and UART configuration
3. Start PWM timers (servo, motors) and base timer (ultrasonic)
4. Center servo at 90° (straight)
5. Set motors to forward direction
6. Apply brief startup kick to motors
7. Enter main control loop
### Testing Utilities
 
Two test functions are included (commented out by default):
- `servo_sweep_test()` – Cycles servo through all preset angles
- `motor_test()` – Tests each motor direction and speed
---
 
## Configuration & Tuning
 
To optimize for your specific track:
 
1. **Set the track profile** in `main.c`:
   ```c
   #define TRACK_PROFILE  2U  // Change to 1U or 2U
   ```
 
2. **Adjust motor speeds** (0–100, 0 = stopped, 100 = full):
   - `MOTOR_SPEED_STRAIGHT` – Speed on straightaways
   - `MOTOR_SPEED_SOFT_TURN` – Speed during gentle turns
   - `MOTOR_SPEED_HARD_TURN` – Speed during sharp turns
3. **Adjust servo angles** (in degrees, 0–180):
   - `SERVO_STRAIGHT_ANGLE` – Center steering
   - `SERVO_HARD_LEFT_ANGLE`, `SERVO_LEFT_ANGLE`, etc. – Turn intensities
4. **Tune sensor filtering**:
   - `IR_SAMPLE_COUNT` – More samples = more noise filtering but slower responsiveness
   - `IR_SAMPLE_DELAY_US` – Larger delay = more stable but slower
   - `STEER_CMD_CONFIRM_COUNT` – More confirmations = smoother but less responsive
5. **Control loop speed**:
   - `CONTROL_LOOP_DELAY_MS` – Smaller = faster response, larger = more stability
6. **Servo ramp speed**:
   - `SERVO_MAX_STEP_PER_LOOP` – Limits servo acceleration for smooth motion
7. **Obstacle distance**:
   - `OBSTACLE_STOP_DISTANCE_MM` – Distance threshold for emergency stop
---
 
## Authors
Mohamed Tarraf  
Dylan Forbes

**Team Members**     
Ezhilvijaykarthick (Karthick) Kalirajan     
Aung Tun (Kroos)    
Annabelle Gamble