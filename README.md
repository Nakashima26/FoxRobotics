# WRO 2026 — Future Engineers | Self-Driving Car

> **Team:** FoxRobotics
> **| Members:** Erick Blanco · Jesse Banda · Cesar Ahumada
> **| Coach:** Daniel Millan
> **| Country / Region:** México — Baja California
> **| Season:** 2026

---

## Table of Contents

1. [Vehicle Overview](#1-vehicle-overview)
2. [Mechanical Design & Mobility](#2-mechanical-design--mobility)
3. [Power Architecture & Sensors](#3-power-architecture--sensors)
4. [Software Architecture](#4-software-architecture)
5. [Systemic Thinking & Engineering Decisions](#5-systemic-thinking--engineering-decisions)
6. [How to Build & Run](#6-how-to-build--run)
7. [Repository Structure](#7-repository-structure)
8. [Videos](#8-videos)
9. [Photos](#9-photos)

---

# 1. Vehicle Overview

Our vehicle is a custom-built autonomous car for the WRO 2026 Future Engineers — Self-Driving Cars challenge. It needs to complete 3 laps around a randomized track, detect and correctly pass colored traffic sign pillars (red on the right, green on the left), and park itself at the end of the Obstacle Challenge round.

**Key specifications:**

| Parameter | Value |
|---|---|
| Dimensions | 210 × 140 × 80 mm |
| Weight | < 1.5 kg |
| Drive type | Rear-wheel drive (RWD) |
| Steering | Ackermann servo steering |
| Main controller | Raspberry Pi 4 Model B |
| Secondary controller | ESP32 (motor & sensor handling) |
| Vision | Raspberry Pi Camera v2 (FOV 62) |
| Distance sensors | HC-SR04(5V) x 2 (left, right) + Level Shifter 5V - 3.3V |
| IMU | MPU-6050 (gyroscope + accelerometer) |
| Drive motor | DC sg90 180° servo |
| Battery (motor) | 3S LiPo 11.1V 2200 mAh |
| Power | Step Down Mini560 5V - 5A |

# Bill of Materials (BOM)

---

## Custom Manufactured Parts

| ID | Component | Description | Quantity | Supplier | Approximate Cost |
|---|---|---|---|---|---|
| 1 | BaseChasis | Main body of the vehicle. | 1 | Custom made, 3D printed | 1.6 USD |
| 2 | TopShell | Upper part of the vehicle. | 1 | Custom made, 3D printed | 1 USD |
| 3 | Cremallera | Rack used for steering. | 1 | Custom made, 3D printed | 0.1 USD |
| 4 | ServoGearDirection | Pinion for the steering system, directly connected to the servo. | 1 | Custom made, 3D printed | 0.5 USD |
| 5 | LinkageDirection | Linkage between the rack and steering knuckle. | 2 | Custom made, 3D printed | 0.05 USD |
| 6 | SteeringKnuckle | Allows lateral wheel movement for steering. | 2 | Custom made, 3D printed | 0.15 USD |
| 7 | DCsupport | Holds the DC motor in place. | 1 | Custom made, 3D printed | 0.05 USD |
| 8 | RingDifferential | Main driven gear of the differential that transfers power to the axle assembly. | 1 | Custom made, 3D printed | 0.25 USD |
| 9 | PlanetDifferential | Allows torque distribution between both wheels while enabling different wheel speeds during turns. | 2 | Custom made, 3D printed | 0.1 USD |
| 10 | PinionDifferential | Transfers rotational motion from the motor to the differential gear system. | 1 | Custom made, 3D printed | 0.1 USD |
| 11 | SunDifferential | Transfers torque from the differential gears to the wheel axle. | 2 | Custom made, 3D printed | 0.1 USD |
| 12 | CameraBase | Holds the camera frame in place. | 1 | Custom made, 3D printed | 0.1 USD |
| 13 | CameraBack | Rear section of the camera frame. Connects to the CameraBase. | 1 | Custom made, 3D printed | 0.1 USD |
| 14 | CameraFront | Camera frame. | 1 | Custom made, 3D printed | 0.2 USD |
| 15 | UltrasonicSupport | Holds the ultrasonic sensor in place. | 2 | Custom made, 3D printed | 0.2 USD |

---

## LEGO Components

| ID | Component | Description | Quantity | Supplier | Approximate Cost |
|---|---|---|---|---|---|
| 16 | Lego 6135494 | Front wheel axle. | 2 | LEGO | 0.8 USD |
| 17 | Lego 4535768 | Rear wheel axle. | 2 | LEGO | 1 USD |
| 18 | Lego 6121485 | Reduces friction on the rear wheels. | 2 | LEGO | 0.4 USD |
| 19 | Lego 4299389 | Rim for rear wheels. | 2 | LEGO | 1.2 USD |
| 20 | Lego 4184286 | Tires for rear wheels. | 2 | LEGO | 3.5 USD |
| 21 | Lego 6251174 | Rim for front wheels. | 2 | LEGO | 1.2 USD |
| 22 | Lego 6182551 | Tires for front wheels. | 2 | LEGO | 3.82 USD |

---

## Electronics

| ID | Component | Description | Quantity | Supplier | Approximate Cost |
|---|---|---|---|---|---|
| 23 | ESP32 | Main microcontroller responsible for sensor processing, control algorithms, and overall robot operation. | 1 | Unit Electronics | 8 USD |
| 24 | Raspi4 Model B | Processes computer vision tasks and handles high-level autonomous navigation functions. | 1 | Amazon | 60 USD |
| 25 | Mini560 5V - 3A | Voltage regulator used to power the Raspberry Pi, ESP32, and peripherals. | 1 | Amazon | 6 USD |
| 26 | MPU 6050 | 6-axis IMU sensor used to measure acceleration, angular velocity, and robot orientation. | 1 | Unit Electronics | 3 USD |
| 27 | HC-SR04 | Ultrasonic distance sensor used for obstacle detection and distance measurement. | 2 | Unit Electronics | 7 USD |
| 28 | Level Shifter | Logic level converter used to safely interface devices operating at different voltage levels. | 1 | Unit Electronics | 7 USD |
| 29 | Driver TB6612FNG | Dual motor driver used to control the speed and direction of the DC motor. | 1 | Unit Electronics | 5 USD |
| 30 | RaspiCamera V2 | Camera module used for computer vision. | 1 | Amazon | 10 USD |
| 31 | Custom PCB | Printed circuit board used for power distribution and electronic connections. | 1 | JLCPCB | 5 USD |

---

## Power System

| ID | Component | Description | Quantity | Supplier | Approximate Cost |
|---|---|---|---|---|---|
| 32 | Ovonic 2200mAh 3S | 3-cell LiPo battery used to power the robot's electronic and drive systems. | 1 | E-Bay | 13 USD |

---

## Actuators

| ID | Component | Description | Quantity | Supplier | Approximate Cost |
|---|---|---|---|---|---|
| 33 | N20 with 50:1 reduction | DC gear motor with a 50:1 reduction ratio used to provide high torque for robot movement. | 1 | Unit Electronics | 6 USD |
| 34 | SG90 Servo | Micro servo motor used for steering control. | 1 | Unit Electronics | 6 USD |

---

## Fasteners

| ID | Component | Description | Quantity | Supplier | Approximate Cost |
|---|---|---|---|---|---|
| 35 | M3 Screws | Used for structural assembly and component mounting. | 23 | Local Hardware Store | 1 USD |
| 36 | M3 Nuts | Used to secure structural and electronic components. | 1 | Local Hardware Store | 0.05 USD |
| 37 | M2 Nuts | Used to secure servo motor and servo pinion | 3 | Local Hardware Store | 0.15 USD |

## Total Estimated Cost

| Total |
|---|
| ~151 USD |


---

# 2. Mechanical Design & Mobility

### 2.1 Chassis selection

The chassis was designed from scratch to fit all the components we needed for competition. We printed it in red and black SUNLU PLA on a Bambu Lab A1, keeping the structure as light as possible without sacrificing the rigidity the drivetrain and electronics require.

For wheel transmission shafts, we used LEGO axles throughout: part 6135494 for the front and part 4535768 for the rear. This wasn't an aesthetic choice — LEGO axles have much better dimensional consistency than fully printed shafts, which flex under load and cause alignment issues. We also added LEGO bushings (6121485) to the rear axle to cut down on friction between moving parts.

| Component   | Material / Part | Purpose                 |
| ----------- | --------------- | ----------------------- |
| Chassis     | SUNLU PLA       | Main robot structure    |
| Front Axles | LEGO 6135494    | Power transmission      |
| Rear Axles  | LEGO 4535768    | Rear drivetrain support |
| Bushings    | LEGO 6121485    | Friction reduction      |
| Printer     | Bambu Lab A1    | Manufacturing process   |

We used PLA because it prints fast, doesn't warp like ABS, and is stiff enough for indoor competition conditions. PETG has better impact resistance and ABS handles heat better, but neither of those properties matters much for a robot running on a flat mat indoors. What did matter was being able to iterate quickly and get consistent parts, and PLA delivered on both.

### 2.2 Wheels

We tested several wheel options before settling on LEGO parts. The rear wheels use rim 4299389 with tire 4184286, and the front uses rim 6251174 with tire 6182551. Both have a 43 mm outer diameter, which keeps the ride height even front to rear.

Since the drivetrain already uses LEGO axles, LEGO-compatible rims were the obvious fit — they mount directly without adapters, which removes a potential source of wobble at the wheel interface.

The 43 mm diameter was also validated against our motor RPM target. At our gear ratio, this diameter puts normal operation in the 40–70% PWM range, which gives us enough resolution for smooth speed control. A noticeably larger wheel would push operating duty cycles above 85% and compress the usable throttle range.

We ran the car at full throttle on the competition mat surface and saw no detectable slip at the rear. The LEGO rubber compound grips the white matte WRO field without any prep needed.

### 2.3 Drive system (torque vs. speed analysis)

The main drive is an N20 DC motor with a 50:1 internal gearbox — compact at 12 × 10 × 26 mm and still enough torque for a ~1.5 kg vehicle. Between the motor output shaft and the differential, we added a 2:1 LEGO gear stage, bringing the total drivetrain reduction to **100:1** and estimated rear axle torque to about 1.76 kg·cm.

**Why 100:1 and not just 50:1?**

We ran the 50:1 configuration first. It was faster, but corner exits were inconsistent — the rear wheels would occasionally lose traction under acceleration, causing yaw disturbances the PID couldn't fully catch. At 100:1, top speed dropped but torque delivery became smooth and predictable all the way through the speed range. Corner exits became repeatable, which turned out to be a prerequisite for reliable PID tuning.

The ratio also keeps the motor in its efficient RPM band during most of the run, which matters because the motor shares a power rail with the ESP32 logic.

The car completes all three Open Challenge laps in about 12 seconds at just 60% motor speed. That headroom is useful if we need to push speed in later iterations.

**The takeaway:** we prioritized consistency over raw speed. A robot that reliably finishes 3 laps scores more than one that's faster but unpredictable.

### 2.4 Steering — rack-and-pinion with Ackermann geometry

Steering is controlled by an SG90 servo connected to both front knuckles through a rack-and-pinion mechanism with symmetric tie rods.

**Mechanism specs:**

| Parameter | Value |
|---|---|
| Servo | SG90 (0°–180° range, center at 90°) |
| Pinion | Module 1, 14 teeth |
| Pinion pitch diameter | 14 mm (dp = m × z = 1 × 14) |
| Pinion outer diameter | 16 mm (dp + 2×addendum = 14 + 2) |
| Rack | Module 1, 11 teeth, 27.5 mm total travel |
| Rack displacement per servo degree | 0.1222 mm/° |
| Transmission ratio | 2.50° servo : 1° wheel |
| Effective wheel steering angle | ±43° from center |

**Kinematic analysis:**

The 14 mm pitch diameter gives a pitch circumference of 43.98 mm. Each degree of servo rotation moves the rack 0.1222 mm. At 90° per side from center, the rack travels 11.00 mm per side — producing ±43° at the front wheels.

The outer diameter of 16 mm confirms the module (d_outer = dp + 2×m = 14 + 2 = 16 mm).

The 2.50:1 ratio reduces steering sensitivity at the wheel, which gives the PID inner loop finer control at its 50 Hz update rate. A lower ratio would make steering response too abrupt.

The ±43° range clears the tightest corners in the Open Challenge (minimum corridor 600 mm) and handles the parallel parking maneuver, all within the servo's rated mechanical range.

**Servo PWM calibration:**

| Servo position | PWM | Wheel angle |
|---|---|---|
| Full left | 500 µs | +43° |
| Center (straight) | 1500 µs | 0° |
| Full right | 2500 µs | −43° |

**Why rack-and-pinion over a direct servo arm?**

A direct arm only controls one wheel directly. The opposite knuckle, linked by a fixed-length tie rod, gets an angle that's geometrically correct only at center — producing toe error everywhere else. The rack pushes both tie rods symmetrically, so both wheels get the right angle at every steering position. Combined with the Ackermann geometry of the knuckles, each wheel tracks its own correct turning radius. No lateral scrub, and steering response is linear across the full range.

---

# 3. Power Architecture & Sensors

### 3.1 Power budget

The whole vehicle runs off an Ovonic 2200mAh 3S LiPo. Power splits two ways: the motor driver takes battery voltage directly for the DC motor, and everything else goes through a MINI560 step-down converter producing a stable 5V at up to 5A.

| Rail | Source | Consumers | Max Current Draw |
|---|---|---|---|
| 5V Logic | MINI560 Step-Down Converter | Raspberry Pi 4 Model B + RaspiCam V2, ESP32, HC-SR04, MPU6050, sg90 | ~4.0 A |
| Battery / Main Power | Ovonic 2200mAh 3S LiPo | Entire vehicle power distribution | ~35 A discharge capability |
| Motor Power | Motor Driver directly from 3S LiPo | DC drive motor | ~1.6 A peak |
| 3.3V Internal | ESP32 internal regulator | I²C communication and internal ESP32 logic, Level Shifter | ~200 mA |

The Ovonic battery is rated 120C continuous, which is theoretically 264 A. We're drawing about 4.5 A peak — under 2% of its discharge capability. There's no shortage of headroom here.

### 3.2 Sensor selection and placement

| Sensor | Purpose | Placement | Notes |
|---|---|---|---|
| Raspberry Pi Camera Module V2 | Lane and obstacle detection | Front-center, 15° downward tilt | Main vision system using OpenCV |
| HC-SR04 ×2 | Wall distance measurement | Left and right sides of the vehicle | Filtered readings for stable navigation |
| MPU-6050 | Heading and turn estimation | Center of chassis | Used for orientation correction |

#### Camera System (Raspberry Pi Camera V2 with wide-angle lens)

The camera handles lane detection, traffic sign identification, and colored pillar detection. Image processing runs on the Raspberry Pi 4 using OpenCV in real time.

It's mounted at the front-center with a 15° downward tilt. We tested horizontal mounting first, but that captured too much background, slowing detection and generating noise. Tilting it down focused the field of view on the track and obstacle zones, cutting both false positives and computational load.

#### HC-SR04 ultrasonic sensors

Two HC-SR04s measure distance to the surrounding walls. They drive lateral positioning, wall following, and corner navigation.

We originally tested VL53L0X time-of-flight sensors. On paper they win — ±3 mm accuracy versus ±15 mm for the HC-SR04, and a narrower beam. In practice, the black competition walls absorbed their 940 nm IR signal and consistently returned out-of-range readings at 300 mm. The HC-SR04 reflects off any surface regardless of color. We went with reliability over precision and made up the accuracy difference with software filtering.

Both sensors are mounted on opposite sides of the chassis. Readings are averaged and invalid values rejected before they reach the PID.

#### MPU-6050 IMU

The MPU-6050 gives us heading and rotation data during cornering and fast maneuvers. The gyroscope estimates the robot's heading and feeds the inner PID loop, complementing what the camera sees.

On startup, the IMU averages multiple readings while the robot is stationary to calculate gyroscope bias. That offset is subtracted throughout the run, which cuts drift and keeps angular estimation accurate over the course of three laps.

---

## 4. Software Architecture

### 4.1 System overview

The current implementation runs entirely on the ESP32, which handles sensor reads, PID computation, state transitions, and actuator control in a single real-time loop.

```
┌──────────────────────────────────────────────────┐
│                     ESP32                        │
│                                                  │
│  ┌─────────────┐   ┌─────────────┐   ┌────────┐  │
│  │ HC-SR04 ×2  │──►│             │──►│  SG90  │  │
│  └─────────────┘   │ Cascade PID │   │ Servo  │  │
│  ┌─────────────┐   │   + FSM     │   └────────┘  │
│  │  MPU-6050   │──►│             │──►┌────────┐  │
│  └─────────────┘   └─────────────┘   │  N20   │  │
│                                      │ Motor  │  │
│  [ Bluetooth — debug telemetry ]     └────────┘  │
└──────────────────────────────────────────────────┘
```

### 4.2 Finite State Machine (FSM)

The firmware uses a 3-state FSM:

```
      ┌─────────────┐
      │  SIGUIENDO  │  Normal lane following — cascade PID active
      └──────┬──────┘
             │  lateral aperture > 100 cm AND cooldown elapsed
             ▼
      ┌─────────────┐
      │   GIRANDO   │  90° corner — servo locked, speed ramped down
      └──────┬──────┘
             │  |anguloGyro| >= 82°
             ▼
      ┌─────────────┐
      │  SIGUIENDO  │  anguloObjetivo updated to new heading
      └─────────────┘
```

A fourth state `EVADIENDO` exists in the codebase for Bluetooth-triggered lateral avoidance during bench testing. It doesn't run during competition.

**Corner detection:**

A corner fires when either ultrasonic reads above 100 cm, meaning a wall has disappeared to one side. Two guards prevent false triggers:
- `millis() > 9000` — ignores the first 9 seconds after start
- `cooldownGiro = 2000 ms` — prevents the same corner from being counted twice

**Turn direction:**

The first detected corner sets the driving direction for the entire run:

```cpp
if (!primerGiro) {
    direccionIzquierda = (distL > distR);
    primerGiro = true;
}
```

If the left wall disappears first, the vehicle turns left — counterclockwise. All subsequent corners follow the same direction, which matches the WRO rules requirement.

### 4.3 Lane following — dual cascade PID

**The problem with a single-loop PID:**

The original controller used a single error signal:

```cpp
error = distL - distR
```

This worked fine when the vehicle was parallel to the walls. Past about 30° of yaw, the HC-SR04's conical beam hit the wall at an oblique angle. Both sensors simultaneously over-reported their distances, so `distL - distR` sat near zero even as the vehicle drifted toward a wall. The controller saw no error and did nothing — causing wall contacts in roughly 30% of test runs.

**The solution — cascade architecture:**

```
                ┌────────────────────┐   heading    ┌────────────────────┐
  distL─distR ─►│     OUTER PID      │─ setpoint ──►│     INNER PID      │──► servo
                │   (wall centering) │              │  (heading control) │
                └────────────────────┘              └────────────────────┘
                         ▲                                    ▲
                  HC-SR04 readings                    MPU-6050 yaw
```

The outer PID produces a target heading from the lateral wall error. The inner PID drives the servo to that heading using the IMU, which doesn't care about wall color or beam geometry and stays accurate at any vehicle angle.

```cpp
// OUTER PID — wall centering
errorWall    = constrain((float)(distL - distR), -50.0f, 50.0f);
integralWall += errorWall * dt;
float derivWall  = (errorWall - prevErrorWall) / dt;
float outputWall = KpWall * errorWall + KiWall * integralWall
                 + KdWall * derivWall;

// INNER PID — heading control  
errorGyro    = anguloObjetivo - anguloGyro;
float derivGyro  = (errorGyro - prevErrorGyro) / dt;
float outputGyro = KpGyro * errorGyro + KiGyro * integralGyro
                 + KdGyro * derivGyro;

// Combined output → servo
float output = constrain(outputWall + outputGyro, -45.0f, 45.0f);
escribirServo(centroServo + (int)output);
```

**Tuned gains:**

| Parameter | Value | Rationale |
|---|---|---|
| KpWall | 1.0 | Outer proportional |
| KiWall | 0.0 | Disabled — integral windup caused overshoot at corners |
| KdWall | 1.2 | Dampens lateral oscillation |
| KpGyro | 2.0 | Inner proportional |
| KiGyro | 0.0 | Disabled |
| KdGyro | 0.5 | Dampens heading oscillation |

**Why no integral terms?**

With Ki enabled, the integral built up error during straight sections. At corner entry, the stored value produced a large steering impulse that overcorrected toward the opposite wall. Removing Ki from both loops fixed this entirely. The derivative terms provide enough steady-state correction on their own.

### 4.4 Sensor filtering — exponential moving average

Raw HC-SR04 readings spike occasionally from floor reflections in curved sections. An EMA filter smooths them before they reach the PID:

```cpp
float filtroEMA(float nueva, float anterior) {
    return alpha * nueva + (1 - alpha) * anterior;  // alpha = 0.75
}
```

Alpha = 0.75 weights recent readings heavily for fast response to genuine wall changes while still killing single spike values. We tested down to 0.3; anything below 0.5 introduced lag that slowed the outer PID's reaction to real lateral drift.

### 4.5 Corner turning — progressive speed ramp

During a corner, speed drops in steps as IMU yaw accumulates:

```cpp
if      (delta < 45) velocidadMotor = 165;
else if (delta < 70) velocidadMotor = 145;
else                 velocidadMotor = 120;
```

This gives better positioning accuracy at corner exit. Once `|anguloGyro| >= 82°`, the servo returns to center and `anguloObjetivo` updates to anchor the inner PID to the new straight heading.

### 4.6 IMU heading integration

Yaw is computed by integrating the MPU-6050 Z-axis each loop iteration:

```cpp
float gyroZ = mpu.getGyroZ();
if (abs(gyroZ) < 1.0) gyroZ = 0;   // 1°/s deadband suppresses drift
anguloGyro += gyroZ * dt;
```

The 1°/s deadband prevents MEMS thermal noise from accumulating into the integrated heading during straight travel. Without it, the inner PID applies a constant small steering offset to correct phantom heading drift, which slowly degrades straight-line tracking.

### 4.7 Development status

| Module | Status |
|---|---|
| ESP32 lane following (Open Challenge) | Complete |
| Corner detection and turn execution | Complete |
| IMU-based heading control | Complete |
| Camera pillar detection (Raspberry Pi) | In development |
| ESP32 ↔ Raspberry Pi UART communication | In development |
| Obstacle avoidance FSM integration | In development |
| Parking maneuver | In development |

---

## 5. Systemic Thinking & Engineering Decisions

### 5.1 Subsystem interaction map

Every subsystem affects at least one other. Here's how data flows from sensors to actuators in the current Open Challenge implementation:

```
    [HC-SR04 L] ──dist_L──►┐
                            ├──► OUTER PID ──heading_sp──► INNER PID ──► SG90 Servo
    [HC-SR04 R] ──dist_R──►┘                               ▲
                                                           │
    [MPU-6050]  ──yaw──────────────────────────────────────┤
                                                           │
    [MPU-6050]  ──corner_count──► FSM state transitions    │
                                        │                  │
                                        └──speed_cmd──► N20 Motor
```

The ESP32 runs all of this in a single loop. The IMU samples at ~100 Hz, ultrasonics at ~20 Hz (limited by HC-SR04 pulse timing), and the FSM evaluates state transitions after every sensor update.

### 5.2 Key engineering trade-offs

**Trade-off 1: VL53L0X ToF vs. HC-SR04 ultrasonic**

The VL53L0X has ±3 mm precision versus ±15 mm for the HC-SR04 and a narrower beam. But the WRO 2026 field walls are matte black (rules 13.4 and 13.6), which absorbs 940 nm infrared. In our tests the VL53L0X returned out-of-range values on the black inner wall for most measurements at 300 mm distance. The HC-SR04 reflects off any surface regardless of color. We chose reliability and made up the difference with the EMA filter and cascade PID.

**Trade-off 2: Single-loop PID vs. cascade PID**

A single loop based on `distL - distR` is simpler to tune — 2 gains instead of 4. But it fails when vehicle yaw exceeds ~30° because both sensors simultaneously over-report distance at oblique angles, hiding the real lateral error. The cascade approach adds an IMU-based inner loop that corrects heading independently of ultrasonic geometry. The extra tuning complexity was worth it: it directly eliminated wall contacts that happened in ~30% of single-loop test runs.

**Trade-off 3: Integral term enabled vs. disabled**

Full PID with Ki was implemented in v1.0 for both loops. The integral of the outer loop accumulated error on long straights. At corner entry, the stored value produced a large steering impulse that overcorrected toward the opposite wall. Removing Ki from both loops fixed this. The derivative terms alone handle steady-state correction.

**Trade-off 4: ESP32 only vs. ESP32 + Raspberry Pi**

Running everything on a single ESP32 would simplify the system — no inter-device communication, no Linux boot latency, no UART protocol to maintain. But real-time computer vision creates constraints that make a single-controller setup impractical.

On the Raspberry Pi 4, a single HSV masking pass on a 640×480 frame takes 8–15 ms. That's fine for vision alone. Combine it with simultaneous motor PWM generation, ultrasonic triggering, and IMU integration on the same processor, and you get non-deterministic scheduling delays. Linux can't guarantee a GPIO pulse fires within a specific microsecond window when an OpenCV operation is running concurrently.

The solution is splitting responsibility by hardware strength:

| Responsibility | Controller | Reason |
|---|---|---|
| Ultrasonic reads | ESP32 | Requires precise µs-level GPIO timing |
| IMU integration | ESP32 | Needs consistent ~100 Hz sampling |
| Servo + motor PWM | ESP32 | Hardware PWM, < 1 ms jitter |
| Cascade PID | ESP32 | Low-latency control loop |
| Camera capture | Raspberry Pi | Native CSI interface, zero-copy |
| OpenCV color detection | Raspberry Pi | Requires full CPU core |
| High-level FSM (Obstacle) | Raspberry Pi | Coordinates vision + avoidance commands |

Each controller runs within its strengths. The ESP32 handles the vehicle at the hardware level with deterministic timing; the Pi handles vision at its own frame rate without touching the control loop. Commands from the Pi arrive over UART and update the ESP32 FSM state — if a UART packet is delayed 10 ms because OpenCV is busy, the ESP32 keeps running on the last valid command rather than stalling.

### 5.3 Iteration log

| Version | Change | Problem solved | Outcome |
|---|---|---|---|
| v1.0 | Single-loop PID on distL−distR, VL53L0X ToF | Baseline | ToF failed on black walls; wall contacts at high yaw |
| v1.1 | Replaced VL53L0X with HC-SR04 | ToF absorbed by black walls | Wall detection reliable across full distance range |
| v1.2 | Removed Ki from both PID loops | Integral windup at corners | Overshoot at corner entry eliminated |
| v1.3 | Added cascade PID (outer lateral + inner heading) | HC-SR04 oblique-angle error at >30° yaw | Wall contacts eliminated |
| v1.4 | Added EMA filter (alpha=0.75) on ultrasonic reads | Spike readings in curved sections | Lateral noise reduced significantly |
| v1.5 | Progressive speed ramp during corner turn | Positioning error at corner exit | More consistent corner exit heading |
| v1.6 | 1°/s deadband on gyro integration | MEMS thermal drift accumulating in straight travel | Straight-line heading stability improved |

### 5.4 Risk analysis

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| HC-SR04 spike reading in curved section | Medium | Medium — brief PID disturbance | EMA filter (alpha=0.75); cascade inner loop dampens effect |
| Gyro drift accumulation over 3 laps | Low | Medium — heading offset grows | 1°/s deadband; anguloObjetivo reset after each corner |
| Corner missed by detection logic | Low | High — wrong lap count, wrong direction | 100 cm threshold tuned conservatively; 2s cooldown prevents double-count |
| Motor voltage sag under load | Low | Medium — speed inconsistency | Separate logic and motor power rails |
| Servo reaching mechanical end stop | Low | Low — rack has physical limits | ±45° software clamp in constrain() call |

---

# 6. How to Build & Run

### 6.1 Hardware requirements

- Raspberry Pi 4 (2 GB RAM minimum)
- ESP32 DevKit v1
- Raspberry Pi Camera Module v2 + wide-angle lens adapter
- HC-SR04 sensors × 2
- MPU-6050 IMU
- SG90 Servo
- 12V N20 DC motor with 1:50 gearbox
- TB6612FNG motor driver (or equivalent)
- 3S LiPo 2200 mAh
- Level Shifter
- MINI560 Step Down

### 6.2 Software dependencies

**Raspberry Pi (Python 3.10+):**
```bash
pip install opencv-python picamera2 pyserial numpy RPi.GPIO smbus2
```

**ESP32 (Arduino IDE 2.x / PlatformIO):**
```
Libraries: Wire.h, MPU6050_tockn
```

### 6.3 Flashing & running

**Step 1 — Flash ESP32:**
```bash
cd src/ESP32
# Open in Arduino IDE or PlatformIO and upload to ESP32
# Select board: ESP32 Dev Module, Port: /dev/ttyUSB0
```

**Step 2 — Configure Raspberry Pi:**
```bash
git clone https://github.com/Nakashima26/FoxRobotics.git
cd FoxRobotics
pip install -r requirements.txt
# Enable camera: sudo raspi-config → Interface Options → Camera → Enable
```

**Step 3 — Run (Open Challenge):**
```bash
cd src/RASPI
python wroPI.py --mode open_challenge
```

**Step 4 — Run (Obstacle Challenge):**
```bash
python wroPI.py --mode obstacle_challenge
```

---

# 7. Repository Structure

```
FoxRobotics/
│
├── electrical/
│   ├── WRO_RevA/               # KiCad project for the PCB
│   └── PCB_dimensions.png      # Dimensions used for designing the PCB
│
├── src/
│   ├── RASPI/
│   │   └── cam/                   # Raspi4 camera + CV for obstacle challenge
│   │        ├── __pycache__/   
│   │        ├── pista/       
│   │        ├── calibration.py                 
│   │        ├── control.py                 
│   │        ├── controlPI.py                
│   │        ├── DistGyro.py                
│   │        ├── Serial.py                
│   │        ├── Vision.py                  
│   │        ├── Vision_2.py               
│   │        ├── wro.py    
│   │        ├── wroPI.py   
│   │        └── wroSave.py              
│   │
│   └── esp32/
│       ├── Controller.ino                  # Main ESP32 controller
│       └── TestCodes/
│           ├── LeerSerialESP32/            # Test code for serial read
│           │    └── LeerSerialESP32.ino  
│           ├── LeerUltrasonicosESP32/      # Test code for ultrasonic reading
│           │    └── LeerUltrasonicosESP32.ino
│           ├── TestDiretional/             # Test code for directional movement
│           │    └── TestDiretional.ino
│           ├── TestGyro/                   # Test code for gyroscope reading
│           │    └── TestGyro.ino  
│           └── TestDcMotor/                # Test code for DC motor PWM
│                └── TestDcMotor.ino
│
├── models/
│   ├── cad/                          # SolidWorks CAD
│   └── stl/                          # 3D printable parts
│
├── t-photos/                         # Team photos
│
├── v-photos/                         # Vehicle photos
│
├── video/                            # Operating vehicle
│
└── README.md                         # This file
```

---

# 8. Videos

| Challenge | Video Link | Duration |
|---|---|---|
| Open Challenge — 3 laps autonomous | [YouTube link] | |

---

# 9. Photos

> Photos are in [`v-photos/`](v-photos/)

| View | Filename |
|---|---|
| Front | `front.jpg` |
| Rear | `rear.jpg` |
| Left side | `left.jpg` |
| Right side | `right.jpg` |
| Top | `top.jpg` |
| Bottom (undercarriage) | `bottom.jpg` |
| Team photo | `team.jpg` |
| Electronics close-up | `electronics.jpg` |

---

## License

This repository is public as required by WRO Future Engineers rules and will remain public for at least 12 months after the competition.

*WRO Future Engineers 2026 — FoxRobotics — México*
