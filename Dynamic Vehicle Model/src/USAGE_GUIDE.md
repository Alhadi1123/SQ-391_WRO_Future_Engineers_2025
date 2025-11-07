# Usage Guide: PID Tuning Simulation

## Quick Start

### Installation

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Run the simulation:**
   ```bash
   python pid_tuning_sim.py
   ```

## Understanding the Simulation

### Bicycle Kinematic Model

The simulation implements a bicycle model with the following parameters:
- **Wheelbase (L)**: 0.15 m
- **Maximum steering angle**: ±30 degrees
- **Velocity**: 0.3 m/s (default)

### PID Control Law

The dual-error PID controller uses:

```python
correction = kp1 * angle_err + kp2 * dist_err
```

Where:
- `angle_err`: Heading error (rad) = desired_heading - current_heading
- `dist_err`: Lateral distance error (m) = d_right - d_left
- `kp1`: Gain for heading error (default: 0.5)
- `kp2`: Gain for lateral distance error (default: -0.75)

### Error Definitions

**1. Heading Error (angle_err):**
- Difference between desired and actual heading
- Positive: Robot heading left of desired
- Negative: Robot heading right of desired

**2. Lateral Distance Error (dist_err):**
- Deviation from track centerline
- Positive: Robot is left of centerline
- Negative: Robot is right of centerline
- Formula: `dist_err = d_right - d_left = -2 * (y - centerline_y)`

## Customizing the Simulation

### Modify Controller Parameters

Edit the `main()` function in `pid_tuning_sim.py`:

```python
results = run_simulation(
    kp1=0.5,          # Heading error gain
    kp2=-0.75,        # Lateral error gain
    velocity=0.3,      # Robot velocity (m/s)
    duration=15.0,     # Simulation time (s)
    initial_offset=0.2 # Initial lateral offset (m)
)
```


### Parameter Tuning Guidelines

**For kp1 (Heading Error Gain):**
- **Too low (< 0.3)**: Slow response, poor corner tracking
- **Optimal (0.4 - 0.6)**: Good tracking, minimal oscillation
- **Too high (> 0.7)**: Oscillations, overshoot

**For kp2 (Lateral Error Gain):**
- **Too low magnitude (> -0.5)**: Slow centering, wall proximity
- **Optimal (-0.6 to -0.9)**: Fast centering, stable
- **Too high magnitude (< -1.0)**: Oscillations, zigzag motion

## Interpreting Results

### Performance Metrics

The simulation outputs several metrics:

1. **Maximum Cross-Track Error**: Largest deviation from centerline
2. **Average Cross-Track Error**: Mean deviation over entire run
3. **Final Cross-Track Error**: Steady-state error
4. **Maximum Heading Error**: Largest orientation error
5. **Average Heading Error**: Mean orientation error

### Good Performance Indicators

- Max cross-track error < 10 cm
- Average cross-track error < 3 cm
- Final cross-track error < 2 cm
- No oscillations in plots
- Smooth steering input

## Output Plots

The simulation generates 6 plots:

1. **Robot Trajectory**: Top-down view of path
2. **Cross-Track Error**: Lateral deviation over time
3. **Heading Angle**: Orientation over time
4. **Steering Control**: Control input over time
5. **Error Components**: Heading and distance errors
6. **PID Correction**: Controller output

## Parameter Sensitivity Analysis

Run the built-in sensitivity analysis to see how parameters affect performance:

```python
parameter_sensitivity_analysis()
```

This tests:
- kp1: [0.3, 0.4, 0.5, 0.6, 0.7] with kp2 = -0.75
- kp2: [-0.5, -0.65, -0.75, -0.85, -1.0] with kp1 = 0.5

## Advanced Usage

### Add Derivative Control

Modify `DualErrorPIDController` class to include derivative terms:

```python
class DualErrorPIDController:
    def __init__(self, kp1=0.5, kp2=-0.75, kd1=0.1, kd2=-0.1):
        self.kp1, self.kp2 = kp1, kp2
        self.kd1, self.kd2 = kd1, kd2
        self.prev_angle_err = 0.0
        self.prev_dist_err = 0.0
    
    def compute_control(self, angle_err, dist_err, dt):
        # Proportional terms
        p_term = self.kp1 * angle_err + self.kp2 * dist_err
        
        # Derivative terms
        d_angle = (angle_err - self.prev_angle_err) / dt
        d_dist = (dist_err - self.prev_dist_err) / dt
        d_term = self.kd1 * d_angle + self.kd2 * d_dist
        
        correction = p_term + d_term
        
        # Update previous errors
        self.prev_angle_err = angle_err
        self.prev_dist_err = dist_err
        
        return correction
```

### Add Noise and Disturbances

Simulate sensor noise:

```python
def compute_errors_with_noise(self, robot_state, noise_std=0.01):
    angle_err, dist_err = self.compute_errors(robot_state)
    
    # Add Gaussian noise
    angle_err += np.random.normal(0, noise_std)
    dist_err += np.random.normal(0, noise_std)
    
    return angle_err, dist_err
```

## Troubleshooting

**Problem: Robot oscillates heavily**
- Solution: Reduce kp1 and/or increase kd1 (if using derivative)

**Problem: Robot doesn't center on track**
- Solution: Increase magnitude of kp2 (more negative)

**Problem: Robot drifts slowly**
- Solution: Increase kp1 or add integral term

**Problem: Robot overshoots corrections**
- Solution: Add derivative terms (kd1, kd2)


## References

For theoretical background, see the Dynamic Vehicle Model README.md file which includes:
- Bicycle kinematic model equations
- Ackermann steering geometry
- PID control theory
- Academic references
