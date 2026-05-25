# 🤖 WRO 2026 — Future Engineers | Self-Driving Car

> **Team:** FoxRobotics
> **| Members:** Erick Blanco · Jesse Banda · Cesar Ahumada
> **| Coach:** Daniel Millan
> **| Country / Region:** México — Baja California
> **| Season:** 2026

---

## 📋 Table of Contents

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

Our vehicle is a custom-built autonomous car designed for the WRO 2026 Future Engineers — Self-Driving Cars challenge. It must complete 3 laps around a randomized track, detect and correctly pass colored traffic sign pillars (red on the right, green on the left), and perform a parallel parking maneuver at the end of the Obstacle Challenge round.

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
 Power | Step Down Mini560 5V - 5A|

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
| 32 | Ovonic 2200mAh 3S | 3-cell LiPo battery used to power the robot’s electronic and drive systems. | 1 | E-Bay | 13 USD |

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

### 2.1 Chassis Selection & Justification

The chassis was fully custom-designed to integrate all the components required to successfully complete the different challenges of the competition. The main structure was 3D printed using red and black SUNLU PLA filament on a Bambu Lab A1 printer. The design focused on maintaining a lightweight structure while still providing enough rigidity to support the drivetrain, sensors, and electronic components.

To increase the mechanical strength and reliability of the drivetrain, LEGO axles were used for all wheel transmission shafts. Specifically, two LEGO axle pieces (6135494) were used for the front section, while two LEGO axle pieces (4535768) were implemented on the rear axle assembly. These components were selected because they provide higher rigidity and better dimensional consistency compared to fully 3D-printed shafts, reducing bending and improving power transmission efficiency.

Additionally, LEGO bushings (6121485) were incorporated into the rear axle system to reduce friction between moving parts. This helped improve rotational smoothness, minimize mechanical resistance, and reduce unwanted energy losses during operation.

| Component   | Material / Part | Purpose                 |
| ----------- | --------------- | ----------------------- |
| Chassis     | SUNLU PLA       | Main robot structure    |
| Front Axles | LEGO 6135494    | Power transmission      |
| Rear Axles  | LEGO 4535768    | Rear drivetrain support |
| Bushings    | LEGO 6121485    | Friction reduction      |
| Printer     | Bambu Lab A1    | Manufacturing process   |

The chassis was manufactured using PLA filament because it offered an effective balance between rigidity, print quality, manufacturing speed, and dimensional accuracy. Since the robot was designed for indoor competition conditions without exposure to high temperatures or extreme impacts, PLA provided sufficient mechanical strength while remaining lightweight and easy to manufacture.

Compared to other materials such as PETG or ABS, PLA allowed faster prototyping and more consistent print results, which was especially important during the iterative design and testing process. Its higher stiffness also helped reduce flexing in structural components, improving drivetrain alignment and overall mechanical stability.

Another important factor was reliability during printing. PLA produces less warping than ABS, enabling more precise parts and reducing manufacturing errors, which helped accelerate development and replacement of components during testing.

Although materials such as PETG offer greater impact resistance and ABS provides better thermal resistance, these properties were not critical for the competition environment. Therefore, PLA was selected because its advantages in rigidity, dimensional precision, and rapid manufacturing better matched the requirements of the robot.

### 2.2 Wheels
The vehicle uses LEGO wheels selected after evaluating multiple options for diameter, axle compatibility, and traction performance. The rear wheels use rim 4299389 paired with tire 4184286, while the front wheels use rim 6251174 paired with tire 6182551. Both axles use the same 43 mm outer diameter, which simplified the mechanical design by keeping the vehicle ride height consistent front to rear and avoiding any pitch bias in the chassis.

Since the drivetrain relies on LEGO axles for the differential and steering linkage, LEGO-compatible rims were a natural fit — they mount directly onto the axles without custom adapters, eliminating potential wobble or play that an adapter-based solution could introduce at the wheel interface.

The 43 mm diameter was validated against our target motor RPM: at the selected gear ratio, this diameter produces a linear speed that keeps PWM duty cycles in the 40–70% range during normal operation, preserving sufficient resolution for smooth speed control in both directions. Choosing a significantly larger diameter would have pushed operating duty cycles above 85%, compressing the usable throttle range and reducing fine-speed control precision.

Traction was validated empirically on the competition mat surface: the vehicle was run at full throttle on a straight section and through 
representative corners, confirming that the rear tires produced no detectable slip on the white matte WRO field surface. The rubber compound of the LEGO tires provided sufficient grip without requiring any surface treatment or tire preparation.

### 2.3 Drive System (Torque vs. Speed Analysis)

The robot uses an N20 DC motor with a 50:1 internal gearbox as the main 
propulsion system. This motor was selected for its compact form factor 
(12 × 10 × 26 mm) while still delivering sufficient torque for the 
vehicle's target weight of approximately 1.5 kg.

Between the motor output shaft and the differential input, an additional 
2:1 gear reduction was implemented using a LEGO gear stage. This brings 
the overall drivetrain reduction to **100:1**, producing an estimated 
output torque of approximately 1.76 kg·cm at the rear axle.

Regarding the differential, this was also printed using PLA, 

**Why 100:1 and not a lower reduction?**

We evaluated the 50:1 configuration first (bypassing the additional gear 
stage). At 50:1, the vehicle reached higher top speed but exhibited 
inconsistent behavior when exiting corners — the rear wheels occasionally 
lost traction under acceleration, producing unpredictable yaw disturbances 
that the PID controller could not fully compensate for. At 100:1, top speed 
decreased but torque delivery became smooth and consistent through the full 
speed range. Corner exit behavior became repeatable, which was a prerequisite 
for reliable PID tuning.

The selected ratio also keeps the motor operating in its efficient RPM band 
for the majority of the run, reducing heat buildup and current draw — 
important given that the motor shares a power rail with the ESP32 logic. 

Using this configuration, the vehicle successfully completed all three laps 
of the Open Challenge course in approximately 12 seconds, operating at only 
60% of its maximum motor speed — demonstrating that the 100:1 drivetrain 
ratio provides ample torque margin and leaves room for speed optimization 
in future iterations if needed.

**Conclusion:** We prioritized consistency and controllability over raw 
speed. A robot that reliably completes 3 laps scores more than one that 
is faster but unpredictable.

### 2.4 Steering System — Rack-and-Pinion + Ackermann Geometry

Steering is controlled by an **SG90 servo** connected to both front 
knuckles through a **rack-and-pinion mechanism** with symmetric tie rods.

**Mechanism specifications:**

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

The pinion pitch diameter of 14 mm gives a pitch circumference of 
**43.98 mm**. Each degree of servo rotation therefore displaces the rack 
by **0.1222 mm**. With the SG90 operating across its full 90° per side 
from center, the rack travels **11.00 mm per side**, producing an 
effective front wheel steering angle of **±43°**.

Note: the outer diameter of the pinion measures 16 mm, which corresponds 
exactly to the standard gear addendum formula for module 1 
(d_outer = dp + 2×m = 14 + 2 = 16 mm), confirming the module and tooth 
count are correct.

The 2.50:1 transmission ratio reduces steering sensitivity at the wheel 
relative to the servo command, providing finer angular control resolution. 
A lower ratio would make steering response too abrupt for the PID inner 
loop to handle smoothly at its 50 Hz update rate.

The ±43° effective wheel angle is sufficient to complete the tightest 
corners of the Open Challenge course (minimum corridor width: 600 mm) 
and to execute the parallel parking maneuver, while keeping the servo 
within its rated mechanical range at all times.

**Servo PWM calibration:**

| Servo position | PWM | Wheel angle |
|---|---|---|
| Full left | 500 µs | +43° |
| Center (straight) | 1500 µs | 0° |
| Full right | 2500 µs | −43° |

**Why rack-and-pinion over a direct servo arm?**

A direct servo arm attached to a single knuckle only controls that wheel 
directly. The opposite knuckle, linked by a fixed-length tie rod, receives 
an angle that is geometrically correct only at center — producing toe-in 
or toe-out at all other positions. The rack distributes symmetric linear 
displacement to both tie rods simultaneously, so both front wheels receive 
the correct angular input at every steering position. Combined with the 
Ackermann geometry of the front knuckles, each wheel follows its correct 
turning radius independently, eliminating lateral tire scrub and making 
steering response linear and predictable across the full ±43° range.

---

# 3. Power Architecture & Sensors

### 3.1 Power Budget

Overall, the entire vehicle is powered by an Ovonic 2200mAh 3S LiPo battery. The power is distributed between the motor driver, which supplies the DC motor, and the rest of the electronic components through a MINI560 step-down converter. This converter regulates the voltage to a stable 5V at up to 5A, providing sufficient power for the Raspberry Pi 4 Model B, the ESP32, and the remaining peripherals.

| Rail | Source | Consumers| Max Current Draw |
| ---| --- | --- | --- |
| 5V Logic             | MINI560 Step-Down Converter | Raspberry Pi 4 Model B + RaspiCam V2, ESP32, HC-SR04, MPU6050, sg90| ~4.0 A  |
| Battery / Main Power | Ovonic 2200mAh 3S LiPo  | Entire vehicle power distribution | ~35 A discharge capability |
| Motor Power | Motor Driver directly from 3S LiPo | DC drive motor | ~1.6 A peak |
| 3.3V Internal | ESP32 internal regulator | I²C communication and internal ESP32 logic, Level Shifter | ~200 mA |


The Ovonic 2200mAh 3S LiPo battery is rated at 120C continuous discharge, which corresponds to a theoretical maximum discharge current of approximately 264 A. Considering that the vehicle operates at an estimated peak current draw of around 4.5 A, the system uses less than 2% of the battery’s maximum discharge capability. This provides a very large safety margin and ensures stable power delivery for both the propulsion and electronic systems.

### 3.2 Sensor Selection & Placement

| Sensor                        | Purpose                     | Placement                       | Notes                                   |
| ----------------------------- | --------------------------- | ------------------------------- | --------------------------------------- |
| Raspberry Pi Camera Module V2 | Lane and obstacle detection | Front-center, 15° downward tilt | Main vision system using OpenCV         |
| HC-SR04 ×2                    | Wall distance measurement   | Left and right sides of the vehicle | Filtered readings for stable navigation |
| MPU-6050                      | Heading and turn estimation | Center of chassis               | Used for orientation correction         |

#### Camera System (Raspberry Pi Camera V2 with Wide-Angle Lens)
- The main perception system of the vehicle is based on a Raspberry Pi Camera Module V2 equipped with a wide-angle lens. This camera is responsible for detecting lane boundaries, identifying traffic signs and colored pillars, and providing the visual data required for autonomous navigation. Image processing is performed onboard by the Raspberry Pi 4 Model B using OpenCV in real time.

- The camera is mounted at the front-center section of the robot with a slight downward inclination of approximately 15°. Several mounting positions were tested during development. Initially, the camera was placed horizontally; however, this configuration captured excessive background information, which increased unnecessary image processing and reduced detection efficiency. By tilting the camera downward, the field of view became focused primarily on the track and obstacle zones, improving detection reliability and reducing computational load.

#### HC-SR04 Ultrasonic Sensors
- The vehicle uses two HC-SR04 ultrasonic sensors to measure the distance between the robot and the surrounding walls. These sensors are used to assist with lateral positioning, wall following, and corner navigation throughout the autonomous run.

- During the development process, VL53L0X Time-of-Flight sensors were initially evaluated as an alternative because of their higher precision and faster response time. However, after extensive testing inside the arena, it was observed that the black walls frequently produced unstable or incorrect readings due to poor light reflection. This caused inconsistent distance measurements that negatively affected the stability of the vehicle during navigation.

- In comparison, although the HC-SR04 sensors can produce noisier measurements, their readings proved to be significantly more reliable under the competition conditions. By implementing software filtering techniques, such as averaging and invalid reading rejection, the ultrasonic sensors provided sufficiently stable and consistent distance measurements for autonomous operation.

- The sensors are mounted on both sides of the chasis. Their data is continuously processed by the control system to maintain a stable trajectory inside the arena.

#### MPU-6050 IMU (Gyroscope and Accelerometer)
- The vehicle also uses an MPU-6050 inertial measurement unit, which combines a gyroscope and accelerometer. This sensor provides orientation and rotational data that supports the navigation system during cornering and rapid maneuvers.

- The gyroscope is primarily used to estimate the robot’s heading and detect rotational movement while turning. This information helps the robot maintain directional stability and complements the visual information obtained from the camera. The sensor is mounted near the center of the chassis in a flat orientation to minimize vibration effects and improve measurement consistency.

- To improve reliability, the IMU performs an automatic calibration process during startup. While the robot remains stationary, multiple readings are averaged to determine the gyroscope bias. This offset is then compensated in software throughout operation, reducing drift and improving angular estimation accuracy during autonomous driving.

---

## 4. Software Architecture

### 4.1 System Overview

The current implementation runs entirely on the **ESP32**, which handles
all sensor reads, PID computation, state transitions, and actuator control
in a single real-time loop. No external computer is involved during the run.
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

The firmware implements a 3-state FSM:
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

A fourth state `EVADIENDO` exists in the codebase for Bluetooth-triggered
lateral avoidance during bench testing. It is not used during competition runs.

**Corner detection:**

A corner is triggered when either ultrasonic sensor reads above **100 cm**
— meaning the wall has disappeared to one side. Two guards prevent false
triggers:
- `millis() > 9000` — ignores the first 9 seconds after start
- `cooldownGiro = 2000 ms` — prevents the same corner from being counted twice

**Turn direction:**

The first detected corner determines the driving direction for the entire run:

```cpp
if (!primerGiro) {
    direccionIzquierda = (distL > distR);
    primerGiro = true;
}
```

If the left wall disappears first, the vehicle is turning left —
consistent with counterclockwise driving. All subsequent corners
use the same direction, matching the WRO rules requirement that the
vehicle maintains a consistent direction throughout the run.

### 4.3 Lane Following — Dual Cascade PID

**The problem with a single-loop PID:**

The original controller used a single error signal:

```cpp
error = distL - distR
```

This worked when the vehicle was aligned parallel to the walls.
However, when yaw exceeded ~**30°**, the HC-SR04's conical beam
hit the wall obliquely. Both sensors simultaneously over-reported
their distances, so `distL - distR` stayed near zero even as the
vehicle drifted toward a wall. The controller perceived no error
and did nothing — causing wall contacts in roughly 30% of test runs.

**The solution — cascade architecture:**
```
                ┌────────────────────┐   heading    ┌────────────────────┐
  distL─distR ─►│     OUTER PID      │─ setpoint ──►│     INNER PID      │──► servo
                │   (wall centering) │              │  (heading control) │
                └────────────────────┘              └────────────────────┘
                         ▲                                    ▲
                  HC-SR04 readings                    MPU-6050 yaw
```

The outer PID produces a target heading from the lateral wall error.
The inner PID drives the servo to achieve that heading using the IMU —
which is independent of wall color or beam geometry and remains accurate
at any vehicle angle.

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

With Ki enabled, the integral accumulated error during straight sections.
At corner entry, the stored integral produced a large steering impulse
that overcorrected toward the opposite wall. Removing Ki from both loops
eliminated this entirely.

### 4.4 Sensor Filtering — Exponential Moving Average

Raw HC-SR04 readings contain occasional spikes from floor reflections
in curved sections. An EMA filter smooths these before they reach the PID:

```cpp
float filtroEMA(float nueva, float anterior) {
    return alpha * nueva + (1 - alpha) * anterior;  // alpha = 0.75
}
```

Alpha = **0.75** weights recent readings heavily for fast response to
genuine wall changes, while still attenuating single spike values.
Lower values (tested down to 0.3) introduced lag that caused the outer
PID to react too slowly to real lateral drift.

### 4.5 Corner Turning — Progressive Speed Ramp

During a corner, speed reduces progressively as IMU yaw accumulates:

```cpp
if      (delta < 45) velocidadMotor = 165;
else if (delta < 70) velocidadMotor = 145;
else                 velocidadMotor = 120;
```

This improves positioning accuracy at corner exit. Once
`|anguloGyro| >= 82°`, the servo returns to center and
`anguloObjetivo` is updated to anchor the inner PID to the
new straight heading.

### 4.6 IMU Heading Integration

Yaw is computed by integrating the MPU-6050 Z-axis at each loop iteration:

```cpp
float gyroZ = mpu.getGyroZ();
if (abs(gyroZ) < 1.0) gyroZ = 0;   // 1°/s deadband suppresses drift
anguloGyro += gyroZ * dt;
```

The **1°/s deadband** prevents MEMS thermal noise from accumulating
into the integrated heading during straight travel. Without it,
the inner PID applies a constant small steering offset to correct
phantom heading drift, degrading straight-line tracking.

### 4.7 Development Status

| Module | Status |
|---|---|
| ESP32 lane following (Open Challenge) | ✅ Complete |
| Corner detection and turn execution | ✅ Complete |
| IMU-based heading control | ✅ Complete |
| Camera pillar detection (Raspberry Pi) | 🔄 In development |
| ESP32 ↔ Raspberry Pi UART communication | 🔄 In development |
| Obstacle avoidance FSM integration | 🔄 In development |
| Parking maneuver | 🔄 In development |
---

## 5. Systemic Thinking & Engineering Decisions

### 5.1 Subsystem Interaction Map

Every subsystem in the vehicle affects at least one other. The diagram
below shows how data flows from sensors to actuators in the current
Open Challenge implementation:
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

The ESP32 runs all of this in a single loop. The IMU is sampled at
every iteration (~100 Hz effective), ultrasonics at ~20 Hz (limited
by HC-SR04 pulse timing), and the FSM evaluates state transitions
after every sensor update.

### 5.2 Key Engineering Trade-offs

**Trade-off 1: VL53L0X ToF vs. HC-SR04 ultrasonic**

The VL53L0X offers ±3 mm precision vs. ±15 mm for the HC-SR04, and
a narrower beam angle. However, the WRO 2026 field walls are matte
black (rules 13.4 and 13.6), which absorbs 940 nm infrared. In our
tests, the VL53L0X returned out-of-range values on the black inner
wall in the majority of measurements at 300 mm distance. The HC-SR04
reflects off any surface regardless of color. We chose reliability
over precision and compensated with the EMA filter and the cascade
PID architecture.

**Trade-off 2: Single-loop PID vs. cascade PID**

A single loop based on `distL - distR` is simpler to tune (2 gains
instead of 4). However, it fails when vehicle yaw exceeds ~30° because
both ultrasonic sensors simultaneously over-report distance at oblique
angles, masking the real lateral error. The cascade architecture adds
an IMU-based inner loop that corrects heading independently of
ultrasonic geometry. The additional tuning complexity was accepted
because it directly eliminated wall contacts that occurred in ~30%
of test runs with the single-loop approach.

**Trade-off 3: Integral term enabled vs. disabled**

Full PID (with Ki) was implemented in v1.0 for both loops. The
integral of the outer loop accumulated error during long straight
sections. At corner entry, the stored integral produced a large
steering impulse that overcorrected toward the opposite wall.
Removing Ki from both loops eliminated this behavior. The derivative
terms alone provide sufficient steady-state correction.

**Trade-off 4: Monolithic architecture (ESP32 only) vs. 
heterogeneous architecture (ESP32 + Raspberry Pi)**

Running all logic on a single ESP32 would simplify the system —
no inter-device communication, no Linux boot latency, no UART
protocol to maintain. However, real-time computer vision for
traffic sign detection introduces constraints that make a single-
controller approach impractical.

The Raspberry Pi Camera Module captures frames that must be decoded,
color-space converted, and masked via OpenCV before a decision can
be made. On the Raspberry Pi 4, a single HSV masking pass on a
640×480 frame takes approximately 8–15 ms. While this is acceptable
for vision alone, combining it with simultaneous motor PWM generation,
ultrasonic triggering, and IMU integration on the same processor
introduces non-deterministic scheduling delays — Linux cannot
guarantee that a GPIO pulse fires within a specific microsecond
window when a CPU-intensive OpenCV operation is running concurrently.

The solution is a **heterogeneous split by responsibility**:

| Responsibility | Controller | Reason |
|---|---|---|
| Ultrasonic reads | ESP32 | Requires precise µs-level GPIO timing |
| IMU integration | ESP32 | Needs consistent ~100 Hz sampling |
| Servo + motor PWM | ESP32 | Hardware PWM, < 1 ms jitter |
| Cascade PID | ESP32 | Low-latency control loop |
| Camera capture | Raspberry Pi | Native CSI interface, zero-copy |
| OpenCV color detection | Raspberry Pi | Requires full CPU core |
| High-level FSM (Obstacle) | Raspberry Pi | Coordinates vision + avoidance commands |

This architecture allows each controller to operate within its
strengths. The ESP32 handles the vehicle at the hardware level with
deterministic timing, while the Raspberry Pi handles vision at its
own frame rate without affecting the control loop. Commands from
the Pi arrive over UART and update the ESP32 FSM state — if a
UART packet is delayed by 10 ms due to OpenCV processing, the ESP32
continues executing the last valid command rather than stalling.

### 5.3 Iteration Log

| Version | Change | Problem Solved | Outcome |
|---|---|---|---|
| v1.0 | Single-loop PID on distL−distR, VL53L0X ToF | Baseline | ToF failed on black walls; wall contacts at high yaw |
| v1.1 | Replaced VL53L0X with HC-SR04 | ToF absorbed by black walls | Wall detection reliable across full distance range |
| v1.2 | Removed Ki from both PID loops | Integral windup at corners | Overshoot at corner entry eliminated |
| v1.3 | Added cascade PID (outer lateral + inner heading) | HC-SR04 oblique-angle error at >30° yaw | Wall contacts eliminated |
| v1.4 | Added EMA filter (alpha=0.75) on ultrasonic reads | Spike readings in curved sections | Lateral noise reduced significantly |
| v1.5 | Progressive speed ramp during corner turn | Positioning error at corner exit | More consistent corner exit heading |
| v1.6 | 1°/s deadband on gyro integration | MEMS thermal drift accumulating in straight travel | Straight-line heading stability improved |

### 5.4 Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| HC-SR04 spike reading in curved section | Medium | Medium — brief PID disturbance | EMA filter (alpha=0.75); cascade inner loop dampens effect |
| Gyro drift accumulation over 3 laps | Low | Medium — heading offset grows | 1°/s deadband; anguloObjetivo reset after each corner |
| Corner missed by detection logic | Low | High — wrong lap count, wrong direction | 100 cm threshold tuned conservatively; 2s cooldown prevents double-count |
| Motor voltage sag under load | Low | Medium — speed inconsistency | Separate logic and motor power rails |
| Servo reaching mechanical end stop | Low | Low — rack has physical limits | ±45° software clamp in constrain() call |

---

# 6. How to Build & Run

### 6.1 Hardware Requirements

- Raspberry Pi 4 (2 GB RAM minimum)
- ESP32 DevKit v1
- Raspberry Pi Camera Module v2 + wide-angle lens adapter
- HC-sr04 sensors × 2
- MPU-6050 IMU
- sg90 Servo
- 12V n20 DC motor with 1:50 gearbox
- tb6612fng motor driver (or equivalent)
- 3S LiPo 2200 mAh 
- Level Shifter
- MINI560 Step Down

### 6.2 Software Dependencies

**Raspberry Pi (Python 3.10+):**
```bash
pip install opencv-python picamera2 pyserial numpy RPi.GPIO smbus2
```

**ESP32 (Arduino IDE 2.x / PlatformIO):**
```
Libraries: Wire.h, MPU6050_tockn
```

### 6.3 Flashing & Running

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
│   ├── WRO_RevA/               #Contains all the KiCad project for the PCB
│   └── PCB_dimensions.png      #Dimensions used for designing the PCB
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
│       ├── Controller.ino                  # Main esp32 controller
│       └── TestCodes/
│           ├── LeerSerialESP32/            # Test code for serial read
│           │    └── LeerSerialESP32.ino  
│           ├── LeerUltrasonicosESP32/      # Test code for Ultrasonic reading
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
├── v-photos/                         #Vehicle photos
│
├── video/                            #Operating vehicle
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

> Photos are located in [`v-photos/`](v-photos/)

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
