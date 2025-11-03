# Bicycle Kinematic Model and PID Control for Autonomous Navigation

This folder provides a comprehensive explanation of the bicycle kinematic model and how we use it in our robot.

## Table of Contents

- [Introduction](#introduction)
- [Bicycle Kinematic Model](#bicycle-kinematic-model)
  - [Model Description](#model-description)
  - [Kinematic Equations](#kinematic-equations)
  - [Ackermann Steering Geometry](#ackermann-steering-geometry)
- [Application to SQ-391 Robot](#application-to-sq-391-robot)
- [PID Control for Path Tracking](#pid-control-for-path-tracking)
  - [Error Functions](#error-functions)
  - [Control Law](#control-law)
  - [Parameter Tuning](#parameter-tuning)
- [Python Simulation](#python-simulation)
- [References](#references)

---

## Introduction

The **bicycle kinematic model** is a widely used simplified representation of car-like vehicle dynamics in robotics and autonomous vehicle research [[1](#ref1), [2](#ref2)]. Instead of modeling all four wheels of a vehicle, the bicycle model approximates the vehicle with two wheels: one at the front (steerable) and one at the rear (fixed orientation). This simplification significantly reduces computational complexity while maintaining sufficient accuracy for path planning and control applications [[3](#ref3)].

Our SQ-391 robot employs an Ackermann steering mechanism with a differential drive system, making it an ideal candidate for bicycle model-based control strategies. This folder explores the mathematical foundations of the bicycle model and demonstrates how PID control can be used for robust path tracking.

---

## Bicycle Kinematic Model

### Model Description

The bicycle model represents a vehicle with:
- **Wheelbase (L)**: Distance between front and rear axles
- **Steering angle (δ)**: Angle of the front wheel relative to vehicle orientation
- **Vehicle pose**: Position (x, y) and heading angle (θ) at the center of the rear axle

The model assumes:
- No wheel slip (pure rolling motion)
- Planar motion (no vertical dynamics)
- Low-speed kinematics (lateral forces negligible)
- Point contact between wheels and ground

### Kinematic Equations

The bicycle kinematic model is described by the following differential equations [[1](#ref1)]:

```
ẋ = v cos(θ)
ẏ = v sin(θ)
θ̇ = (v/L) tan(δ)
```

Where:
- `(x, y)` = Position of the rear axle in global coordinates
- `θ` = Heading angle (yaw) of the vehicle
- `v` = Linear velocity at the rear axle
- `δ` = Steering angle of the front wheel
- `L` = Wheelbase length

**Discrete-time update equations** (for simulation with time step Δt):

```
x(k+1) = x(k) + v·cos(θ(k))·Δt
y(k+1) = y(k) + v·sin(θ(k))·Δt
θ(k+1) = θ(k) + (v/L)·tan(δ(k))·Δt
```

### Ackermann Steering Geometry

The Ackermann steering mechanism ensures that all wheels follow concentric circular paths during turns, minimizing tire slip [[4](#ref4)]. The fundamental Ackermann steering equation relates the inner and outer wheel angles:

```
cot(δ_outer) - cot(δ_inner) = w/L
```

Where:
- `w` = Track width (lateral distance between left and right wheels)
- `L` = Wheelbase
- `δ_inner` = Steering angle of inner wheel (sharper)
- `δ_outer` = Steering angle of outer wheel (gentler)

**Turning radius** from the instantaneous center of curvature (ICC):

```
R = L / tan(δ)
```

Where `δ` is the equivalent bicycle model steering angle (approximated as the average of the front wheel angles).

**Ackermann percentage** in practical implementations [[4](#ref4)]:

Most real vehicles achieve 80-100% Ackermann geometry due to mechanical constraints. The SQ-391 robot uses a servo-actuated Ackermann linkage with a percentage that can be calibrated through the mechanical design.

---

## Application to SQ-391 Robot

Our robot incorporates several design elements that align with the bicycle kinematic model:

### Mechanical Design
- **Wheelbase (L)**: Distance between front Ackermann steering assembly and rear differential
- **Ackermann Steering**: Ensures proper turning geometry with minimal tire scrub
- **Differential Drive**: Allows rear wheels to rotate at different speeds during turns
- **Servo Control**: MG996R servo provides precise steering angle control

### Sensor Integration
- **IMU (BNO086)**: Provides heading angle `θ` and angular velocity `θ̇`
- **Ultrasonic Sensors (URM09)**: Measure lateral distances for error calculation
- **Camera Module**: Detects track boundaries and obstacles

### Control Architecture
The robot uses ROS Noetic with the following control structure:
1. **Localization**: IMU provides orientation; odometry estimates position
2. **Path Planning**: Generate reference trajectory based on track geometry
3. **Control**: PID controller computes steering commands based on error signals

---

## PID Control for Path Tracking

### Error Functions

Our robot employs a **dual-error PID controller** that combines heading error and lateral position error [[5](#ref5), [6](#ref6)]:

**1. Heading Error (Angle Error)**

```
e_θ(t) = wrap(θ_ref - θ(t))
```

Where:
- `θ_ref` = Desired heading angle (e.g., tangent to the path)
- `θ(t)` = Current heading from IMU
- `wrap()` = Function to normalize angle to [-π, π]

**2. Lateral Distance Error**

```
e_d(t) = d_right(t) - d_left(t)
```

Where:
- `d_right(t)` = Distance to right wall (from ultrasonic sensors)
- `d_left(t)` = Distance to left wall (from ultrasonic sensors)

This error represents the lateral deviation from the centerline. When `e_d > 0`, the robot is closer to the left wall; when `e_d < 0`, it's closer to the right wall.

### Control Law

The composite steering correction is calculated as [[7](#ref7)]:

```
corr(t) = kp1 · e_θ(t) + kp2 · e_d(t)
```

Where:
- `kp1` = Proportional gain for heading error (≈ 0.5)
- `kp2` = Proportional gain for lateral distance error (≈ -0.75)

The negative sign on `kp2` ensures proper corrective action: when `e_d > 0` (robot too far left), a negative correction steers right.

**Steering angle command**:

```
δ_cmd(t) = δ_nominal + corr(t)
```

Where `δ_nominal` is the feedforward steering angle for the desired path curvature.

### Parameter Tuning

**Tuning Guidelines** [[8](#ref8), [9](#ref9)]:

1. **kp1 (Heading Error Gain)**:
   - Start with `kp1 = 0.3` and increase gradually
   - Too low: Slow response to heading errors, poor corner entry
   - Too high: Oscillations around the path, overshoot
   - Optimal range: `0.4 - 0.6` for WRO-sized tracks

2. **kp2 (Lateral Error Gain)**:
   - Start with `kp2 = -0.5` and adjust magnitude
   - Too low magnitude: Slow lateral correction, wall proximity
   - Too high magnitude: Oscillations, "zigzag" motion
   - Optimal range: `-0.6 to -0.9` depending on sensor noise

**Tuning Process**:
1. Set both gains to zero and test open-loop steering
2. Increase `kp1` until heading tracking is acceptable
3. Add `kp2` gradually to center the robot on the track
4. Fine-tune both parameters simultaneously
5. Test at different speeds and track sections

**Advanced Considerations**:
- Add derivative terms (`kd1`, `kd2`) to reduce oscillations [[10](#ref10)]
- Implement gain scheduling: adjust gains based on velocity
- Use integral terms for steady-state error elimination (e.g., dealing with sensor bias)

---

## Python Simulation

The folder includes a complete Python simulation that demonstrates:

1. **Bicycle Model Dynamics**: Numerical integration of kinematic equations
2. **PID Controller**: Dual-error control law implementation
3. **Parameter Tuning Visualization**: Interactive plots showing effect of gain variations
4. **Path Tracking Performance**: Metrics including cross-track error, heading error, and control effort

See `pid_tuning_simulation.py` for the full implementation.

**Key Features**:
- Configurable track geometries (straight, circular, S-curve)
- Real-time visualization of robot trajectory
- Parameter sensitivity analysis
- Performance metrics calculation

---

## References

<a name="ref1"></a>[1] Dergipark (2022). "Trajectory tracking performance comparison of kinematic bicycle model with LQR and Lyapunov based controllers." *Uludağ Üniversitesi Mühendislik Fakültesi Dergisi*. Available: http://dergipark.org.tr/en/doi/10.55974/utbd.1130198

<a name="ref2"></a>[2] IEEE (2020). "Implementation of Kinematics Bicycle Model for Vehicle Localization using Android Sensors." *2020 IEEE 10th International Conference on System Engineering and Technology*. doi: 10.1109/ICSET51301.2020.9232453

<a name="ref3"></a>[3] MATLAB (2024). "Bicycle Kinematic Model - Compute car-like vehicle motion." *MathWorks Robotics System Toolbox Documentation*. Available: https://de.mathworks.com/help/robotics/ref/bicyclekinematicmodel.html

<a name="ref4"></a>[4] RAW Robotics (2024). "Introduction to Ackerman Steering." *RAW Robotics Knowledge Base*. Available: https://raw.org/book/kinematics/ackerman-steering/

<a name="ref5"></a>[5] Azam, S. et al. (2020). "Dynamic Control System Design for Autonomous Car." *SCITEPRESS Digital Library*. Available: https://www.scitepress.org/Papers/2020/93929/93929.pdf

<a name="ref6"></a>[6] MDPI Electronics (2023). "Evaluating the Performance of Fuzzy-PID Control for Lane Recognition and Lane-Keeping in Vehicle Simulations." *Electronics*, 12(3), 724. doi: 10.3390/electronics12030724

<a name="ref7"></a>[7] Martínez-Valdez, A., Ramirez-Neria, M., & González-Sierra, J. (2024). "Quanser Self Driving Car Trajectory Tracking by employing a Bicycle's Model." *CDSR 2024 Proceedings*. Available: https://avestia.com/CDSR2024_Proceedings/files/paper/CDSR_136.pdf

<a name="ref8"></a>[8] Journal of Robotics and Control (2021). "Lane Keeping Maneuvers Using Proportional Integral Derivative (PID) and Model Predictive Control (MPC)." *JRC*, 2(2), 78-82. Available: https://journal.umy.ac.id/index.php/jrc/article/view/8800

<a name="ref9"></a>[9] SAE International (2020). "Autonomous Lane Change Control Using Proportional-Integral-Derivative Controller and Bicycle Model." *SAE Technical Paper* 2020-01-0215. doi: 10.4271/2020-01-0215

<a name="ref10"></a>[10] IEEE (2023). "Trajectory Tracking and Lane-Keeping Assistance for Autonomous Systems Using PID and MPC Controllers." *2023 International Conference on Control, Automation and Diagnosis*. doi: 10.1109/ICCAD57653.2023.10200281

<a name="ref11"></a>[11] Qiu, N., Huang, Y., & Lu, Z. (2023). "Bicycle Trajectory Control Based on Kinematic and Dynamical Models." *Academic Journal of Mathematical Sciences*, 4(1), 32-43. doi: 10.25236/AJMS.2023.040106


