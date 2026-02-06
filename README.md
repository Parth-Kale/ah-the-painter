# Project Report: B.A.L.L.S.
## Bot Autonomous Locomotion & Logistics Swarm

**Project Team:** B.A.L.L.S.
**Course:** Advanced Topics in Robotics
**Date:** February 4, 2026

---

## 1. Main Part

### 1.1 Problem Statement and Motivation

Traditional mobile robotics platforms, specifically differential wheeled robots and Ackermann steering vehicles, face inherent limitations in unstructured environments. These limitations include:
1.  **Non-holonomic constraints:** inability to move instantly in any direction without rotation.
2.  **Inversion instability:** vulnerability to being overturned, rendering the robot inoperable.
3.  **Environmental sealing:** exposed electronics and mechanical parts are susceptible to dust and moisture.

The **B.A.L.L.S.** project aimed to solve these issues by developing a **Spherical Drive Robot**. The spherical robot encapsulates all mechanics and electronics within a sealed shell, which offers intrinsic environmental protection. Furthermore, by utilizing an internal omnidirectional drive mechanism, the robot achieves holonomic motion, allowing it to move in any vector immediately.

**State-of-the-Art:**
Current existing solutions in spherical robotics typically fall into two categories:
*   **Pendulum Drive (e.g., Sphero, BB-8):** Uses a heavy weight on a single axis to shift the center of mass. This is mechanically simple but suffers from poor stability and slow directional changes.
*   **Internal Car (e.g., Hamster Ball):** A small car drives inside. This requires a very high friction and generally struggles with lateral movement.

**Novelty of the Proposed Solution:**
Our solution utilizes a **4-Motor Internal Drive Unit (IDU)** in a cross (+) configuration using custom angular wheels. This allows for:
1.  **True Holonomic Motion:** Independent control of X and Y vectors.
2.  **High-Speed Capability:** Utilization of Nidec 24H industrial brushless motors (6400 RPM (No Load)).
3.  **"Unflippable" Design:** A stabilizing bearing array ensures the internal drive unit maintains contact with the shell regardless of the sphere's orientation.

### 1.2 Theoretical Basis & Innovation (State-of-the-Art)

The foundational inspiration for this project stems from recent developments in spherical robotics, specifically the research presented in **SpheriDrive: A Spherical Robot** (ScienceDirect, 2025). This study demonstrated the viability of internal drive units for spherical locomotion and established the core physics for center-of-mass shifting.

**Gap in Existing Literature:**
While the referenced paper and similar state-of-the-art solutions successfully demonstrate spherical movement, they typically rely on:
*   **Pendulum Drives:** With a single axle with a hanging weight (mechanically simple but usually unstable during rapid direction changes).
*   **Triangular Omni-drives:** Three omni-wheels arranged in a tetrahedral configuration. This couples the X and Y axes mathematically, often leading to control singularities and complex friction mapping.

**The B.A.L.L.S. Innovation:**
To the best of our knowledge, no existing approach utilizes the specific **Dual-Differential "Plus" Configuration** implemented in this project.
1.  **4-Motor Orthogonal Layout:** We positioned four motors at $90^\circ$ intervals. This allows for mechanical decoupling of the X and Y axes (Motors 1 & 3 drive X; Motors 2 & 4 drive Y), simplifying the inverse kinematics compared to triangular designs.
2.  **Custom Angular Wheels:** Standard omni-wheels have poor contact patches with concave surfaces. We developed custom 3D-printed angular wheels that match the $100mm$ internal radius of the sphere, significantly increasing traction.
3.  **Top-Bearing Stabilization:** Unlike the reference paper, which relied on low Center of Mass (CoM) alone, we implemented a mechanical bearing array that contacts the roof of the sphere, physically preventing the internal unit from flipping during high-torque acceleration.


---

## 2. Solution

### 2.1 Idea of the Solution
The system consists of a 20cm acrylic spherical shell which contains a 3D-printed chassis (the IDU). The IDU presses against the inner surface of the shell using gravity and the weight of the battery pack (6S LiPo). Four Nidec 24H BLDC motors are mounted at 90° intervals.

By spinning the motors, the IDU attempts to "climb" the inner wall of the sphere. The shifting center of mass and the friction between the angular wheels and the shell cause the sphere to roll in the opposite direction of the IDU's rotation.

To keep the IDU stable, and to make sure it returns back to initial position at rest, we add 4 bearings at the top of Base Plate, and add 1 bearing to the bottom.

### 2.2 System Architecture
The system is divided into three main sub-systems:

1.  **Mechanical:**
    *   Spherical Shell (20cm diameter).
    *   Internal Chassis (PLA).
    *   Traction System (4x Angular Wheels + 5x Stabilizing Bearings).
2.  **Electronics (Custom PCB):**
    *   **MCU:** ESP32-S3-WROOM-1 (Dual-core, 240MHz).
    *   **Sensors:** Bosch BNO085/6 9-DOF IMU (for orientation and stability control).
    *   **Actuation:** 4x Nidec 24H BLDC Motors (Integrated Drivers).
    *   **Power:** 24V Main Rail (Motor Power) and Logic Regulation Circuitry.
3.  **Software:**
    *   **Kinematic Engine:** Converts global velocity vectors (v_x, v_y, ω) into individual wheel speeds.
    *   **Control Loop:** PID controller running at 100Hz.
    *   **Comms:** UDP over WiFi for low-latency remote control.

### 2.3 Tools Used
*   **PCB Design:** Altium Designer 26.1
*   **Firmware Development:** Cursor & Arduino IDE
*   **Mechanical CAD:** Autocad 2026
*   **Simulation:** Unity 6 (Physics Engine for Kinematic Validation).
*   **Fabrication:** JLC PCB (Board Fab).

---

## 3. Details of the Solution

### 3.1 Kinematic Modeling
The core of the control system is the Inverse Kinematics (IK) algorithm. Since the wheels are mounted at a fixed angle relative to the ground contact patch, a coupling matrix is required to translate target robot velocity into wheel angular velocity.

Given a target velocity vector $V = [v_x, v_y]^T$ and rotational velocity $\omega_z$, and a wheel mounting angle $\alpha = 45^\circ$, the required velocity for the $i$-th motor ($V_{mi}$) is calculated as:

$$
\begin{bmatrix} V_{m1} \\ V_{m2} \\ V_{m3} \\ V_{m4} \end{bmatrix} =
\begin{bmatrix}
\sin(\alpha) & \cos(\alpha) & 1 \\
\cos(\alpha) & -\sin(\alpha) & 1 \\
-\sin(\alpha) & -\cos(\alpha) & 1 \\
-\cos(\alpha) & \sin(\alpha) & 1
\end{bmatrix}
\begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}
$$

In the firmware, this is implemented as:
```cpp
m1.targetRaw = ( target_Vx * SIN_A + target_Vy * COS_A + target_Wz);
m2.targetRaw = ( target_Vx * COS_A - target_Vy * SIN_A + target_Wz);
// ... m3 and m4 symmetric ...
```

### 3.2 Electronics & PCB Design
A custom PCB was designed to handle the specific requirements of the Nidec motors.
*   **Microcontroller:** The ESP32-S3 was chosen for its native USB capabilities (eliminating external UART chips) and sufficient GPIOs for 4 motors (PWM, Direction, Brake, Encoder A/B for each).
*   **Motor Interface:** The Nidec 24H motors have internal drivers. The PCB acts as a carrier board, routing the 24V power and providing 20kHz PWM signals generated by the ESP32's LEDC peripheral.
*   **Feedback Loop:** The motors provide Quadrature Encoder outputs (100 pulses per revolution). The ESP32 uses pin-change interrupts to track the speed of all four motors simultaneously to feed the PID loop.

### 3.3 Firmware Logic
The firmware utilizes an Object-Oriented approach. A `Motor` class handles the state of each actuator, including:
1.  **Velocity Calculation:** $\Delta \text{Encoder} / \Delta \text{Time}$.
2.  **Acceleration Limiting (Ramping):** A smoothing factor (`ACCEL_FACTOR = 0.05`) prevents sudden torque spikes which would cause the internal robot to climb the wall and tumble (the "Hamster Wheel" effect) rather than rolling the sphere.
3. **PID Control:**
   
   Output = Kₚ × e(t) + Kᵢ ∫ e(t) dt + K_d × (de/dt)
   
   where e(t) = TargetRPM - CurrentRPM

---

## 4. Simulation and Virtual Validation

Given the high complexity of holonomic spherical motion and the lead times on hardware fabrication, the control logic was first developed and validated in a virtual environment.

### 4.1 Simulation Environment
*   **Engine:** Unity 3D (6) utilizing the NVIDIA PhysX engine.
*   **Scripting:** C# (Logic ported directly from the ESP32 C++ firmware).
*   **Physics Setup:**
    *   A rigid body sphere with mass and friction coefficients approximating the acrylic shell.
    *   A simulated internal drive unit utilizing "Wheel Colliders" to apply force vectors relative to the internal surface normals.

### 4.2 Kinematic Validation
The primary goal was to validate the **Coupling Matrix** derived in Section 3.1. The simulation fed the `target_Vx`, `target_Vy`, and `target_Wz` vectors into the matrix.
*   **Scenario A (Forward X):** Input $[1, 0, 0]$.
    *   *Result:* Motors 1 & 3 spun forward/backward; Motors 2 & 4 acted as stabilizers. Sphere moved purely in X.
*   **Scenario B (Diagonal):** Input $[1, 1, 0]$.
    *   *Result:* All 4 motors spun at equal RPM. The sphere moved at $45^\circ$.
*   **Scenario C (Rotation):** Input $[0, 0, 1]$.
    *   *Result:* Differential wheel pairs spun in opposition. Sphere rotated on the Z-axis (Yaw).

### 4.3 Simulation Limitations
While the logic was sound, the simulation failed to model several critical physical realities:
1.  **EMI/Electrical Noise:** The simulation assumes perfect signal integrity.
2.  **Battery Dynamics:** The "pendulum" effect of the battery weight shifting inside the ball was underestimated in the physics engine, leading to over-confidence in acceleration values.
3.  **Surface Friction:** The simulation used a uniform friction coefficient, whereas the real-world interface between PLA wheels and polished Acrylic is highly inconsistent.

---

## 5. PCB Design Process

The electronics design was the critical path for the project. The objective was to create a "Nervous System" capable of handling high-voltage industrial motors and low-voltage logic simultaneously.

### 5.1 Design Specifications
*   **EDA Tool:** Altium Designer 26.2.0
*   **Stack-up:** 4-Layer FR-4 (Signal / GND / Power / Signal).
*   **Form Factor:** $85mm \times 70mm$ (Designed to mount directly atop the motor array).
*   **Finish:** ENIG for Better Hand Soldering.

### 5.2 Power Architecture Design
The design attempted to isolate the "noisy" motor environment from the "quiet" logic environment:
1.  **Star-Point Grounding:** A single point where the High-Current Motor Ground meets the Logic Ground to prevent ground loops.
2.  **Internal Ground Plane:** Layer 2 was dedicated entirely to GND to act as an EMI shield.
3.  **Power Domains (Layer 3):** Dedicated copper pours for the intended 5V/3.3V Rails.

### 5.3 Component Selection Rationale
*   **MCU:** ESP32-S3-WROOM-1 was selected over the standard ESP32 for its native USB interface (simplifying the BOM by removing the CP2102) and higher GPIO count.
*   **Connectors:** XT60 for the main battery (high current handling) and JST-XH for motor phases (keyed to prevent reverse polarity).

### 5.4 Basic Overview

The PCB was split into 2 sections, with lower having the special 24V rails, and 24V to 5V conversion architecture and systems, while the top half had the Logic and related components, such as ESP32-S3 and USB-C connectors. 
This approach allowed for separate testing for logic and motor sections, as well as running the entire system on a singular input, from the 6S battery, rather than having to carry 2 separate batteries.

---

## 6. Mechanical Design Process (3D Fabrication)

The mechanical design utilized an iterative approach using SolidWorks and PrusaSlicer.

### 6.1 Phase 0: The Base Chassis
The initial design was a simple cross-frame. The challenge identified immediately was the "Hamster Wheel" effect—if the internal robot is too light or has too much torque, it climbs the wall and flips over instead of rolling the sphere.

### 6.2 Phase 1: Stabilization & Weight Distribution
To counter the flipping issue:
*   **Battery Placement:** A dedicated tray was designed to hold the heavy 6S LiPo at the absolute lowest point of the sphere to lower the Center of Mass (CoM).
*   **Stabilization Arms:** Four arms extending upwards with bearings were added. These bearings roll against the *top* hemisphere of the shell, physically preventing the drive unit from flipping upside down.

### 6.3 Phase 2: Traction System
Standard rubber tires were too wide and flat for the spherical curvature.
*   **Solution:** Custom "Angular Wheels" were designed and printed in PLA. The profile of the wheel is concave, matching the $100mm$ radius of the sphere interior to maximize contact patch area.

---

## 7. Real-World Tests and Failure Analysis

### 7.1 Pre-Integration Testing
Prior to the final assembly, individual subsystems were validated:
*   **Motor Test:** A single Nidec 24H motor was driven using the PCB and the final 6S Battery. Speed control via PWM was successful.
*   **Logic Test:** The ESP32 firmware logic was tested on the same PCB, testing the abilities of the ESP32 using the buttons and controlling on board LEDs.

### 7.2 The Integration Failure ("Ghost in the Machine")
Upon final assembly, the PCB was populated, the 6S battery connected, and the motors attached.

**The Event:**
*   The system powered on (verified by power LEDs).
*   The ESP32 was receiving voltage (verified by multimeter at 3.3V).
*   **Failure:** The system was completely non-responsive. The MCU did not boot into the firmware loop, and the motors did not engage.
*   **Anomaly:** Unlike typical over-voltage failures, there was no "magic smoke" at the moment of failure, and the ESP32 chip did not become hot to the touch. This indicates the failure was not a simple short of 24V to the 3.3V line.

### 7.3 Engineering Root Cause Analysis
Since the thermal signs of a direct short were absent, the failure is attributed to **Transient Dynamics** and **EMI**, rather than DC over-voltage.

1.  **Inrush Current / Inductive Spikes:** Connecting a 24V LiPo via an XT60 connector generates a massive $dV/dt$ spike (potentially >40V for microseconds). The PCB lacked Transient Voltage Suppression (TVS) diodes or a Soft-Start circuit. This spike likely latched up the MCU or damaged the internal silicon of the regulators without causing thermal destruction.
2.  **EMI Swamping:** The Nidec motors are high-power inductive loads. Without opto-isolators, the startup noise on the shared ground plane likely corrupted the ESP32's boot sequence or flash memory integrity immediately upon power-up.
3.  **Potential Human Mistake:** By actively testing the input voltages using a multimeter, there is a chance that the multimeter accidentaly created a short circuit between high and low voltage components. Though this is unlikely as the ESP32 doesn't show signs of internal damage or "Fried" device.

---

## 8. Results

| Metric | Simulation (Unity) | Real World (Prototype) |
| :--- | :--- | :--- |
| **Locomotion** | Holonomic, Smooth | Stationary (Failure) |
| **Control Latency** | <10ms (Localhost) | N/A |
| **Stability** | Perfect | Mechanical stability verified manually |
| **Cost** | $0 | ~$150 (BOM) |

**Comparison:**
The simulation proved that the *mathematics* and *logic* of the B.A.L.L.S. project are sound. The failure in the real world was strictly an **electrical hardware engineering failure**, not a conceptual one. The gap between simulation and reality was defined by the lack of electrical protection (EMI/Transient handling) which physics engines do not simulate.

---

## 9. Possible Enhancements (Further Work)

To bring this project from a non-functional prototype to a working product, the following changes are required:

1.  **Signal Isolation:**
    *   Implement **Optocouplers** (e.g., PC817) on all motor control lines (PWM/DIR/BRAKE). This physically separates the 24V motor electrical environment from the sensitive 3.3V ESP32 logic.
2.  **Traction Improvement:**
    *   Replace PLA wheels with TPU (flexible filament) or coat them in liquid rubber (rather than the Rubber Tape method used) to increase friction coefficient $\mu$, allowing for higher acceleration without slippage.
3. **Better Planning:**
    *   Try and test out all the devices, such as motors and components at the start, rather than testing a few, and assuming everything works.

---

## 10. Summary

The B.A.L.L.S. project was an ambitious attempt to create a high-performance, swarm-capable spherical robot. Technically, the project succeeded in:
1.  Deriving and validating the complex kinematic model for internal spherical drive.
2.  Designing a compact mechanical chassis that solves the center-of-mass stability issue.
3.  Developing a low-latency UDP control architecture.

However, the project failed operationally due to a lack of robustness in the power electronics design and severe logistical issues. The choice of incompatible voltage regulator topologies and the underestimation of electrical noise in a high-voltage motor system resulted in a "silent" failure of the control board.

The project serves as a valuable case study in **System Integration**: having perfect code and perfect mechanics means nothing if the power delivery system is not robust against the harsh realities of physics.

---

## 11. Appendices

### 11.1 Project Plan Deviations

| Milestone | Original Plan | Actual Result | Deviation Reason |
| :--- | :--- | :--- | :--- |
| **Differential Drive** | Week 3 | Completed | N/A |
| **Spherical Mechanics** | Week 7 | Completed | N/A |
| **PCB Fabrication** | Week 9 | **Failed** | Electrical Failiure and manufacturing delays. |
| **Swarm/Autonomy** | Week 13 | **Cancelled** | Dependent on functional hardware. |

### 11.2 Final Task Distribution

*   **Parth Kale:** PCB Design, Firmware Architecture, Failure Analysis.
*   **Angiras Astakala:** Mechanical Chassis Design, Wheel Geometry, 3D Printing.
*   **Archana Ullatil:** Electrical wiring harness, component soldering.
*   **Syeda Nawar Azim:** Simulation setup, testing protocols, documentation.

