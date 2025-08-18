<center><h1> SQ-391 WRO 2025 </center>
<p align="center">
  <img src="./other/figs/logo.png" alt="logo" width="60%">
</p>

<p align="center">
  <a href="https://manara.edu.sy/index.php?lang=1">
    <img src="https://img.shields.io/badge/Website-Visit-brightgreen?style=for-the-badge&logo=web&logoColor=white">
  </a>
  <a href="https://www.facebook.com/share/1FbUMCjNEs/">
    <img src="https://img.shields.io/badge/Facebook-%231877F2.svg?style=for-the-badge&logo=Facebook&logoColor=white">
  </a>
  <a href="https://www.instagram.com/manarauni?igsh=OWlpZjd6Nm52ZmV3">
    <img src="https://img.shields.io/badge/Instagram-%23E4405F.svg?style=for-the-badge&logo=Instagram&logoColor=white">
  </a>
</p>


This repository contains the documentation for the SQ-391 team's robot for the 2025 World Robot Olympiad Future Engineers competition. The robot was designed and built by a team of three students and one coach.

## Table of Contents

- [The Team](#team)
- [The Challenge](#challenge)
- [The Robot](#robot-image)
- [Performance Video](#video)
- [Mobility Management](#mobility-management)
  - [Vehicle Main Body](#vehicle-main-body)
    - [3D Designed Parts](#3d-designed-parts)
  - [Motion Mechanism](#motion-mechanism)
    - [Ackermann Steering Mechanism](#ackermann-steering-mechanism)
    - [Differential Gear](#differential-gear)
    - [Motor Driver](#motor-driver)
  - [Wheels](#wheels)
  - [Engineering Principles](#engineering-principles)
- [Power and Sense Management](#power-and-sense-management)
  - [Electrical Parts](#electrical-parts)
  - [Overall Circuit Usage](#overall-circuit-usage)
  - [Circuit Grounding](#circuit-grounding)
  - [Power Supply](#power-supply)
  - [Overall Scheme](#overall-scheme)
  - [Wiring Diagram](#wiring-diagram)
  - [Assembly List](#assembly-list)
  - [Shopping List](#shopping-list)
- [Obstacle Management](#obstacle-management)
  - [ROS Architecture](#ros-architecture)
  - [Open Challenge](#open-challenge)
  - [Computer Vision](#computer-vision)
  - [Image Processing](#image-processing)
  - [Obstacle Challenge](#obstacle-challenge)
  - [Problems We Encountered](#problems-we-encountered)
- [Suggestions and Future Modifications](#suggestions-and-future-modifications)
- [Resources](#resources)

---

## The Team <a class="anchor" id="team"></a>

### Alhadi Zidan

<p align="center">
  <img src="./other/images/team/alhadi-zidan.jpeg" alt="Alhadi Zidan" width="80%">
</p>

**Role:** Coach

**Age:** 22

**Description:** Robotics and Intelligent Systems Engineer - AlManara University.

---

### Ammar Daher

<p align="center">
  <img src="./other/images/team/ammar-daher.jpg" alt="Ammar Daher" width="80%">
</p>

**Age:** 21

**Description:** Studying Robotics and Intelligent Systems at AlManara University (fifth year).

---

### Lama Alsakher

<p align="center">
  <img src="./other/images/team/lama-alsakher.jpg" alt="Lama Alsakher" width="80%">
</p>

**Age:** 20

**Description:** Studying Robotics and Intelligent Systems at AlManara University (third year).

---

### Ibrahim Alsheikh

<p align="center">
  <img src="./other/images/team/ibrahim-alsheikh.jpg" alt="Ibrahim Alsheikh" width="80%">
</p>

**Age:** 20

**Description:** Studying Robotics and Intelligent Systems at AlManara University (third year).

---

### Team photo

<p align="center">
  <img src="./t-photos/Official.jpg" alt="Team" width="80%">
</p>

## The Challenge <a class="anchor" id="challenge"></a>

The **[WRO 2025 Future Engineers - Self-Driving Cars](https://wro-association.org/)** challenge invites teams to design, build, and program a robotic vehicle capable of driving autonomously on a racetrack that changes dynamically for each round. The competition includes two main tasks: completing laps while navigating randomized obstacles and successfully performing a precise parallel parking maneuver.

Our approach emphasizes:

- **Advanced Navigation:** Using ROS (Robot Operating System) for real-time sensor fusion and control
- **Computer Vision:** HSV-based color detection for pillar identification
- **Precision Mechanics:** Ackermann steering geometry for optimal maneuverability
- **Robust Engineering:** Systematic approach to power management and component integration

Learn more about the challenge [here](https://wro-association.org/wp-content/uploads/WRO-2025-Future-Engineers-Self-Driving-Cars-General-Rules.pdf).

## Photos of our robot <a class="anchor" id="robot-image"></a>

| <img src="./other/images/robot/front.png" width="90%" /> |  <img src="./other/images/robot/back.png" width="85%" />  |
| :------------------------------------------------: | :-------------------------------------------------: |
|                      _Front_                       |                       _Back_                        |
| <img src="./other/images/robot/left.png" width="90%" />  | <img src="./other/images/robot/right.png" width="85%" />  |
|                       _Left_                       |                       _Right_                       |
|  <img src="./other/images/robot/top.png" width="90%" />  | <img src="./other/images/robot/bottom.png" width="85%" /> |
|                       _Top_                        |                      _Bottom_                       |

<br>

## Our video of the robot <a class="anchor" id="video"></a>

<br>

# Mobility Management <a class="anchor" id="mobility-management"></a>

## Vehicle Main Body <a class="anchor" id="vehicle-main-body"></a>

The vehicle's chassis is constructed from two precision-cut layers of plexiglass to ensure rigidity, durability, and a lightweight structure. The upper layer accommodates the primary control and processing units, including the microprocessors, controllers, actuators, and the onboard camera. The lower layer is dedicated to the navigation system and electrical circuitry, with optimized component placement to minimize interference and maintain a low center of gravity.

The main processing unit is a **Raspberry Pi 4 Model B**, responsible for executing all high-level operational tasks, sensor data processing, and control algorithms using ROS Noetic. The actuation system includes:

- **TowerPro MG996R Servo Motor** - dedicated to steering actuation with 12.0 kg·cm torque
- **JSUMO Core DC Motor (6V, 400 RPM)** - responsible for propulsion with 48:1 gear ratio
- **DRV8871 Motor Driver** - regulates the DC motor's speed and current up to 3.6A peak

### 3D Designed Parts <a class="anchor" id="3d-designed-parts"></a>

We designed all components in SOLIDWORKS and fabricated them using a combination of additive and subtractive manufacturing techniques. The structural parts (base and upper layer) were precision-cut using a CNC laser machine, while functional components were 3D-printed in PLA for lightweight customization.

#### Base Layer

The structural base layer, CNC laser-cut from plexiglass, provides mounting for the Ackermann steering and differential mechanisms on its bottom face and hosts the electronic circuit assembly on its top face.

#### Second Layer

The structural second layer accommodates the Raspberry Pi, ESP32, Power Bank, and camera assembly on its top surface, with DC-DC converters mounted on its underside.

#### Custom 3D-Printed Components

- **Lego-axle to DC motor axle coupler** - High precision interface connecting DC motor shaft to Lego axle
- **Ultrasonic Sensor Holder** - Secures ultrasonic sensors to servo motors for directional scanning
- **Camera Holder** - Two-part assembly with integrated IMU mounting
- **DC Motor Holder** - Rigid mount with vibration reduction
- **Support Wall** - Structural connector between chassis layers

## Motion Mechanism <a class="anchor" id="motion-mechanism"></a>

### Ackermann Steering Mechanism <a class="anchor" id="ackermann-steering-mechanism"></a>

Our design incorporates an Ackermann steering mechanism for efficient and responsive steering performance. The system minimizes tire slippage during turns by arranging steering linkages so the inner wheel turns at a sharper angle than the outer wheel.

**Key Mathematical Relations:**

For wheelbase L, front track t, and turn radius R, the ideal relations are:

```
tan(δ_in) = L/(R - t/2)
tan(δ_out) = L/(R + t/2)
```

**Steering Ratio:** The relationship between servo input angle and resulting road-wheel angle:

```
SR = θ_input / δ_avg
δ_avg ≈ θ_input / SR
```

**Servo Torque:** Required actuator torque:

```
τ_servo ≈ M_req / (SR × η)
```

### Differential Gear <a class="anchor" id="differential-gear"></a>

We implemented a Lego differential gear system that ensures rear wheels can spin at different rates while maintaining balanced power distribution. The differential is crucial during turns when the inner and outer wheels must rotate at different speeds.

**Components:**

- Ring Gear: Large gear receiving power from the motor
- Pinion Gear: Smaller gear connecting to drive shaft
- Side Gears: Two gears linking differential to axle shafts
- Spider Gears: Allow wheels to rotate at varying speeds

**Gear Ratio Calculation:**

```
GR = R/S1 + R/S2
N_wheel = N_Motor_no_load / GR
```

Where R is ring gear teeth, S1 and S2 are side gear teeth.

### Motor Driver <a class="anchor" id="motor-driver"></a>

The DRV8871 H-bridge motor driver provides:

- **Current Capability:** Up to 3.5A continuous, 3.6A peak
- **Voltage Range:** 6.5-45V input
- **PWM Control:** 0-200kHz recommended frequency
- **Protection:** Overcurrent, thermal shutdown, undervoltage lockout
- **Control:** Simple two-pin interface (IN1, IN2)

## Wheels <a class="anchor" id="wheels"></a>

**Specifications:**

- **Diameter:** 65mm
- **Width:** 25-30mm
- **Material:** Rubber tire with plastic hub
- **Weight:** Lightweight for optimal performance
- **Coupling:** Compatible with hex connections

## Engineering Principles <a class="anchor" id="engineering-principles"></a>

1. **Torque and Speed Balance:** Motor selection based on τ = F × r relationship
2. **Power Output:** Verification using P = τ × ω, providing 4.2W sufficient power
3. **Weight Distribution:** Strategic component placement for stability
4. **Maneuverability:** Differential drive system with ~0.1m turning radius
5. **Energy Efficiency:** Optimized gear ratios for peak efficiency operation

# Power and Sense Management <a class="anchor" id="power-and-sense-management"></a>

## Electrical Parts <a class="anchor" id="electrical-parts"></a>

### ESP32-WROOM-32E

- **Core:** Dual-core Xtensa LX6 up to 240MHz
- **Memory:** 520KB SRAM, 4/8/16MB QSPI flash
- **I/O:** Up to 26 GPIO pins, 3.3V logic
- **Communication:** 3×UART, 4×SPI, 2×I2C, Wi-Fi, Bluetooth
- **ADC/DAC:** Two 12-bit SAR ADCs, two 8-bit DACs

### Raspberry Pi 4 Model B

- **Processor:** Broadcom BCM2711, quad-core Cortex-A72 @ 1.5GHz
- **Memory:** Up to 8GB LPDDR4 with ECC
- **Connectivity:** Dual-band Wi-Fi, Bluetooth 5.0, Gigabit Ethernet
- **Ports:** 2×USB 3.0, 2×USB 2.0, 2×micro HDMI (4K support)
- **GPIO:** Standard 40-pin header
- **Power:** 5V DC via USB-C (minimum 3A)

### Raspberry Camera Module 3

- **Resolution:** 11.9 Megapixels (4608 x 2592 pixels)
- **Sensor:** Sony IMX708 with 1.4µm pixel size
- **Field of View:** 102° horizontal, 67° vertical (75° variant available)
- **Focus:** Motorized with ~5cm to ∞ depth of field
- **Features:** HDR support, up to 112s exposure time

### BNO086 IMU (SparkFun VR IMU Breakout)

- **Sensors:** 3D accelerometer, gyroscope, magnetometer
- **Processor:** On-chip Cortex-M0+ running SH-2 sensor fusion
- **Outputs:** Quaternions, linear acceleration, gravity vectors
- **Interface:** I2C (Qwiic), SPI, UART support
- **Reports:** 9-axis, 6-axis, geomagnetic rotation vectors

### DRV8871 Motor Driver

- **Type:** Single H-bridge with integrated current sensing
- **Current:** Up to 3.5A continuous, 3.6A peak
- **Voltage:** 6.5-45V supply range
- **Control:** Two-pin PWM interface (IN1, IN2)
- **Protection:** Overcurrent, thermal shutdown, UVLO

### JSUMO Core DC Motor

- **Voltage:** 6V nominal
- **Speed:** 400 RPM no-load
- **Gear Ratio:** 48:1
- **Torque:** 1.2 kg·cm continuous, 3.9 kg·cm stall
- **Current:** 120mA no-load, 3.2A stall
- **Dimensions:** 47×15×10mm, 21g weight

### TowerPro MG996R Servo

- **Torque:** 12.0 kg·cm at 6V
- **Speed:** 0.2 sec/60° at 6V
- **Voltage:** 4.8-7.2V operating range
- **Control:** Standard PWM (50Hz)
- **Gears:** Metal construction for durability

### URM09 Ultrasonic Sensors

- **Range:** 2-500cm with 1cm resolution
- **Accuracy:** ≈1% (flat target)
- **Interface:** Single-wire TDM (trigger/echo)
- **Rate:** Up to 25Hz measurement rate
- **Beam Angle:** ~60° directional coverage

### Power Supply Components

- **Primary Battery:** 2S LiPo (7.4V, 2200mAh, 50C) with Deans connector
- **Raspberry Pi Power:** INIU 10,000mAh USB-C power bank (22.5W)
- **Voltage Regulation:** XL4005 DC-DC step-down converters
- **Current Distribution:** PCA9685 servo driver board

## Overall Circuit Usage <a class="anchor" id="overall-circuit-usage"></a>

Our power management strategy employs dual power domains:

1. **Control Domain:** Raspberry Pi powered by dedicated USB power bank
2. **Actuation Domain:** Motors and sensors powered by LiPo battery

**Current Specifications:**

| Component      | Supply Voltage | Idle Current | Active Current | Peak Current |
| -------------- | -------------- | ------------ | -------------- | ------------ |
| Raspberry Pi 4 | 5.1V           | ~540mA       | ~1.0-1.3A      | up to 3.0A   |
| Camera Module  | 5V             | --           | ~200-250mA     | ~300-350mA   |
| ESP32          | 3.3V           | <5µA         | ~50-80mA       | ~120-200mA   |
| BNO086 IMU     | 3.3V           | ~3mA         | ~3-10mA        | --           |
| URM09 Sensors  | 5V             | --           | <20mA          | --           |
| MG996R Servo   | 6V             | ~10-20mA     | ~500-900mA     | ~2.0-2.5A    |
| Core DC Motor  | 6V             | --           | ~200-400mA     | ~2.2A        |

## Circuit Grounding <a class="anchor" id="circuit-grounding"></a>

Proper grounding ensures stable operation by providing a common electrical reference point for all components. Our star-grounding scheme minimizes electrical noise and prevents ground loops that could affect sensor readings.

## Power Supply <a class="anchor" id="power-supply"></a>

The separated power architecture prevents motor-induced voltage drops from affecting the Raspberry Pi, ensuring uninterrupted control system operation during high-current motor operations.

## Overall Scheme <a class="anchor" id="overall-scheme"></a>

![Circuit Electronic Scheme](./other/images/circuit/electronic-scheme.png)

## Wiring Diagram <a class="anchor" id="wiring-diagram"></a>

![Circuit Wiring Scheme](./other/images/circuit/wiring-diagram.png)

## Assembly List <a class="anchor" id="assembly-list"></a>

| Label    | Part Type        | Properties                           |
| -------- | ---------------- | ------------------------------------ |
| Mod1     | Raspberry Pi 4B  | BCM2711 Quad-core Cortex-A72 @1.5GHz |
| U5       | ESP32S-HiLetgo   | 38 pins, NudeMCU-32S variant         |
| Part2    | SparkFun BNO086  | VR IMU Breakout (Qwiic)              |
| Part3    | Adafruit DRV8871 | H-bridge motor driver                |
| Part7    | PCA9685          | PWM/Servo breakout board             |
| M1       | JSUMO Core Motor | 6V 400 RPM with 48:1 gearing         |
| J1       | TowerPro MG996R  | High-torque servo motor              |
| SERVO1-3 | TowerPro SG90    | Micro servo motors                   |
| URM1-4   | URM09            | Ultrasonic distance sensors          |
| Part4    | INIU Power Bank  | 10,000mAh USB-C in/out               |
| Part5-6  | LiPo Battery     | 2200mAh 2S configuration             |

## Shopping List <a class="anchor" id="shopping-list"></a>

| Qty       | Part                  | Unit Price (€) | Total (€)  |
| --------- | --------------------- | -------------- | ---------- |
| 1         | Raspberry Pi 4B (4GB) | 61.08          | 61.08      |
| 1         | ESP32 Dev Kit         | 11.49          | 11.49      |
| 1         | SparkFun BNO086       | 49.90          | 49.90      |
| 1         | Adafruit DRV8871      | 7.68           | 7.68       |
| 1         | PCA9685 Servo Board   | 40.79          | 40.79      |
| 1         | JSUMO Core Motor      | 12.50          | 12.50      |
| 1         | TowerPro MG996R       | 12.00          | 12.00      |
| 3         | TowerPro SG90         | 2.99           | 8.97       |
| 4         | URM09 Sensors         | 14.50          | 58.00      |
| 1         | Power Bank 10Ah       | 29.95          | 29.95      |
| 2         | LiPo 2200mAh          | 19.99          | 39.98      |
| 2         | XL4005 DC-DC          | 4.92           | 9.84       |
| 1         | Cooling Fan           | 1.99           | 1.99       |
| **Total** |                       |                | **343.77** |

# Obstacle Management <a class="anchor" id="obstacle-management"></a>

## ROS Architecture <a class="anchor" id="ros-architecture"></a>

Our robot utilizes ROS Noetic for real-time sensor fusion and control:

**Node Structure:**

- **Control Node:** Central processing and decision making
- **Ultrasonic Node:** Range measurements at 25Hz
- **IMU Node:** Orientation data via rosserial bridge
- **Camera Node:** On-demand pillar detection service
- **Actuator Nodes:** Motor and servo command execution

**Communication:**

- Topics: `/ultra_sensors`, `/imu_sensor`, `/cmd_actuators`
- Services: `/camera/capture_pillars` for vision processing
- Parameters: Dynamic reconfiguration for field tuning

## Open Challenge <a class="anchor" id="open-challenge"></a>

### Navigation Strategy

We employ a dual-error PID controller combining heading and lateral positioning:

**Error Functions:**

```
e_θ(t) = wrap(θ(t)) - wrap(θ_ref)
e_d(t) = d_R(t) - d_L(t)
```

**Corner Detection:**

```
max{d_L(t), d_R(t)} > d_corner ≈ 1.0m
```

**Control Law:**
The composite error drives a PD controller for steering commands while maintaining constant forward velocity.

## Computer Vision <a class="anchor" id="computer-vision"></a>

### HSV Color Space Selection

We use HSV (Hue, Saturation, Value) instead of RGB for robust color detection:

**Advantages:**

- **Lighting Independence:** Hue channel unaffected by illumination changes
- **Intuitive Filtering:** Easy threshold definition for color ranges
- **Robustness:** Better performance under varying environmental conditions

**Color Thresholds:**

- **Red Pillars:** Lower: [100, 43, 130], Upper: [141, 255, 255]
- **Green Pillars:** Lower: [30, 24, 90], Upper: [76, 213, 255]
- **Pink Parking:** Lower: [122, 87, 181], Upper: [134, 236, 255]

### Image Processing Pipeline <a class="anchor" id="image-processing"></a>

1. **Camera Setup:** Adjust shutter speed for lighting conditions
2. **Preprocessing:** Frame cropping to remove distracting elements
3. **Color Masking:** HSV threshold application for color isolation
4. **Contour Detection:** Boundary identification for object segmentation
5. **Feature Extraction:** Area, centroid, and bounding rectangle calculation
6. **Distance Estimation:** Calibrated area-to-distance mapping

**Contour Analysis:**

```python
cv2.contourArea(contour)      # Object area calculation
cv2.boundingRect(contour)     # Bounding rectangle
cv2.moments(contour)          # Centroid computation
```

## Obstacle Challenge <a class="anchor" id="obstacle-challenge"></a>

### Three-Phase Algorithm

1. **Parking Exit:** Navigate from starting position to first corner
2. **Section Traversal:** Corner-to-corner navigation with pillar avoidance
3. **Final Parking:** Precise parallel parking maneuver

### Pillar Avoidance Strategy

**Vision-Based Detection:** Camera service identifies pillar colors and positions
**Threshold Selection:** Choose inner/outer wall bias based on pillar colors:

- Red pillars → Outer wall preference
- Green pillars → Inner wall preference
  **Sequential Navigation:** Clear pillars one by one using ultrasonic feedback

### Caching Optimization

After first lap completion, cache per-section pillar configurations to eliminate camera service calls on subsequent laps, reducing latency and computational load.

## Problems We Encountered <a class="anchor" id="problems-we-encountered"></a>

### Wave Interference

**Issue:** Ultrasonic sensors interfering when operating simultaneously
**Solution:** Sequential sensor activation - alternating between opposite sides (R1+L2, then R2+L1, then F+B)

### Mechanical Limitations

**Issue:** Small turn angles and wheel slipping without differential
**Solution:** Implemented Lego differential gear system for smooth turning

### Camera Flickering

**Issue:** Brightness fluctuations in video stream
**Solution:** Manual shutter speed control: `picam2.set_controls({"ExposureTime": shutter_speed})`

### Real-Time Performance

**Issue:** Synchronizing multiple sensor streams
**Solution:** ROS middleware with timestamped messages and configurable update rates

# Suggestions and Future Modifications <a class="anchor" id="suggestions-and-future-modifications"></a>

## Mobility Management

### Mechanical Upgrades

- **Metal Differential:** Replace Lego components with metal differential and axles
- **Robust Coupling:** Metal motor-to-axle adapter for improved reliability
- **Enhanced Bearings:** Low-friction bearings for smoother operation

## Power and Sense Management

### Sensor Improvements

- **Direct IMU Integration:** Connect BNO086 directly to Raspberry Pi I2C
- **ToF Sensors:** Upgrade from ultrasonic to VL53L1X Time-of-Flight sensors
- **LiDAR Integration:** Consider RPLIDAR A1 for 360° obstacle detection

### Power Management

- **Battery Management System:** Implement BMS for cell monitoring and protection
- **Custom PCB:** Design integrated board for robust connections and clean wiring
- **Thermal Management:** Enhanced cooling for sustained operation

## Computer Vision

### Hardware Upgrade

- **Depth Camera:** Intel RealSense D435 for accurate distance measurement
- **Wider FOV:** Improved peripheral vision for obstacle detection

### Software Optimization

- **Persistent Mapping:** Cache pillar configurations after first lap
- **Path Optimization:** Direct section-to-section navigation on subsequent laps
- **Advanced Algorithms:** Machine learning for improved pillar classification

## System Integration

### Performance Enhancements

- **Real-Time OS:** Consider RT kernel for deterministic timing
- **Sensor Fusion:** Advanced Kalman filtering for state estimation
- **Predictive Control:** Model predictive control for optimal path planning

# Resources <a class="anchor" id="resources"></a>

## Technical Documentation

- [WRO 2025 Future Engineers Rules](https://wro-association.org/wp-content/uploads/WRO-2025-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [OpenCV HSV Color Space](https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html)

## Component Datasheets

- [Raspberry Pi 4 Model B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/)
- [ESP32-WROOM-32E](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf)
- [BNO086 IMU](https://cdn.sparkfun.com/assets/1/3/4/5/9/BNO086_Datasheet.pdf)
- [DRV8871 Motor Driver](https://www.ti.com/lit/ds/symlink/drv8871.pdf)

## Open Source Libraries

- [rosserial](http://wiki.ros.org/rosserial) - Arduino/ESP32 to ROS bridge
- [OpenCV](https://opencv.org/) - Computer vision processing
- [SH-2 Reference Manual](https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)

---

## Copyright

```
MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

© 2025 SQ-391 Team
```
