"""
PID Tuning Simulation for Bicycle Kinematic Model
Based on SQ-391 WRO Future Engineers Robot

This simulation demonstrates the bicycle kinematic model with dual-error PID control
on a WRO Future Engineers track (3m x 3m square with 1m wide lanes).

- Heading error (angle_err): Difference between desired and actual heading
- Lateral distance error (dist_err): Deviation from centerline

Control law: corr = kp1 * angle_err + kp2 * dist_err
Default gains: kp1 ≈ 0.5, kp2 ≈ -0.75

Author: SQ-391 Team
Date: November 2025
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
import time


class BicycleModel:
    """
    Bicycle kinematic model for car-like vehicle simulation
    
    State: [x, y, theta]
    Inputs: [v, delta]
    
    Equations:
        x_dot = v * cos(theta)
        y_dot = v * sin(theta)
        theta_dot = (v / L) * tan(delta)
    """
    
    def __init__(self, wheelbase=0.15, max_steering=np.deg2rad(30)):
        """
        Initialize bicycle model
        
        Args:
            wheelbase: Distance between front and rear axles (m)
            max_steering: Maximum steering angle (rad)
        """
        self.L = wheelbase  # Wheelbase
        self.max_delta = max_steering  # Maximum steering angle
        
        # State: [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])
        
    def reset(self, initial_state):
        """Reset to initial state"""
        self.state = np.array(initial_state)
        
    def update(self, v, delta, dt):
        """
        Update state using Euler integration
        
        Args:
            v: Linear velocity (m/s)
            delta: Steering angle (rad)
            dt: Time step (s)
        """
        # Limit steering angle
        delta = np.clip(delta, -self.max_delta, self.max_delta)
        
        x, y, theta = self.state
        
        # Kinematic equations
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        theta_dot = (v / self.L) * np.tan(delta)
        
        # Euler integration
        self.state[0] += x_dot * dt
        self.state[1] += y_dot * dt
        self.state[2] += theta_dot * dt
        
        # Normalize theta to [-pi, pi]
        self.state[2] = self.wrap_angle(self.state[2])
        
        return self.state.copy()
    
    @staticmethod
    def wrap_angle(angle):
        """Wrap angle to [-pi, pi]"""
        return np.arctan2(np.sin(angle), np.cos(angle))


class DualErrorPIDController:
    """
    Dual-error PID controller for bicycle model path tracking
    
    Error 1: Heading error (angle_err)
    Error 2: Lateral distance error (dist_err)
    
    Control law: corr = kp1 * angle_err + kp2 * dist_err
    """
    
    def __init__(self, kp1=0.5, kp2=-0.75, max_correction=np.deg2rad(25)):
        """
        Initialize PID controller
        
        Args:
            kp1: Gain for heading error
            kp2: Gain for lateral distance error (typically negative)
            max_correction: Maximum correction angle (rad)
        """
        self.kp1 = kp1
        self.kp2 = kp2
        self.max_corr = max_correction
        
        # For logging
        self.angle_err_history = []
        self.dist_err_history = []
        self.correction_history = []
        
    def compute_errors(self, robot_state, track, desired_heading):
        """
        Compute heading and lateral distance errors for WRO track
        
        Args:
            robot_state: [x, y, theta]
            track: WROTrack object
            desired_heading: Desired heading angle (rad)
            
        Returns:
            angle_err: Heading error (rad)
            dist_err: Lateral distance error (m)
        """
        x, y, theta = robot_state
        
        # Heading error (angle error)
        angle_err = self.wrap_angle(desired_heading - theta)
        
        # Lateral distance error (distance from lane centerline)
        # Calculate which section of track we're on and distance from centerline
        dist_err = track.get_lateral_error(x, y)
        
        return angle_err, dist_err
    
    def compute_control(self, angle_err, dist_err):
        """
        Compute steering correction using dual-error PID law
        
        Args:
            angle_err: Heading error (rad)
            dist_err: Lateral distance error (m)
            
        Returns:
            correction: Steering correction angle (rad)
        """
        # Dual-error PID control law
        correction = self.kp1 * angle_err + self.kp2 * dist_err
        
        # Limit correction
        correction = np.clip(correction, -self.max_corr, self.max_corr)
        
        # Log for analysis
        self.angle_err_history.append(angle_err)
        self.dist_err_history.append(dist_err)
        self.correction_history.append(correction)
        
        return correction
    
    @staticmethod
    def wrap_angle(angle):
        """Wrap angle to [-pi, pi]"""
        return np.arctan2(np.sin(angle), np.cos(angle))


class WROTrack:
    """
    WRO Future Engineers track: 3m x 3m square with 1m wide lanes
    Track forms a square loop with outer walls and inner walls
    """
    
    def __init__(self):
        """
        Initialize WRO track
        Outer boundary: 3m x 3m
        Inner boundary: 1m x 1m (centered)
        Lane width: 1m
        """
        self.outer_size = 3.0  # Outer boundary (m)
        self.inner_size = 1.0  # Inner boundary (m)
        self.lane_width = 1.0  # Lane width (m)
        
        # Corner positions for path planning (clockwise from bottom-right)
        self.corners = [
            (2.5, 0.5),  # Bottom-right corner (start)
            (2.5, 2.5),  # Top-right corner
            (0.5, 2.5),  # Top-left corner
            (0.5, 0.5),  # Bottom-left corner
        ]
        
        # Track centerline for each straight section
        self.centerline_offset = 0.5  # Distance from walls to centerline
        
    def get_section(self, x, y):
        """
        Determine which section of the track the robot is on
        
        Returns: 0=bottom, 1=right, 2=top, 3=left
        """
        # Find closest corner
        distances = [np.hypot(x - cx, y - cy) for cx, cy in self.corners]
        closest_corner = np.argmin(distances)
        
        # Determine section based on position
        if y < 1.0 and x > 0  and x < 2:
            return 0  # Bottom section
        elif x > 2.0 and y > 0 and y < 2:
            return 1  # Right section
        elif y > 2.0 and x < 3 and x > 1:
            return 2  # Top section
        else:
            return 3  # Left section
    
    def get_desired_heading(self, x, y):
        """
        Get desired heading based on position on track
        
        Returns: desired heading in radians
        """
        section = self.get_section(x, y)
        
        if section == 0:  # Bottom: moving right (East)
            return 0.0
        elif section == 1:  # Right: moving up (North)
            return np.pi / 2
        elif section == 2:  # Top: moving left (West)
            return np.pi
        elif section == 3:  # Left: moving down (South)
            return -np.pi / 2
        
        return 0.0
    
    def get_lateral_error(self, x, y):
        """
        Calculate lateral distance error from lane centerline
        Positive: robot is toward outer wall
        Negative: robot is toward inner wall
        
        Returns: lateral error in meters
        """
        section = self.get_section(x, y)
        
        if section == 0:  # Bottom section
            centerline_y = self.centerline_offset
            dist_err = y - centerline_y
        elif section == 1:  # Right section
            centerline_x = self.outer_size - self.centerline_offset
            dist_err = centerline_x - x
        elif section == 2:  # Top section
            centerline_y = self.outer_size - self.centerline_offset
            dist_err = centerline_y - y
        elif section == 3:  # Left section
            centerline_x = self.centerline_offset
            dist_err = x - centerline_x
        else:
            dist_err = 0.0
        
        return dist_err
    
    def is_in_bounds(self, x, y):
        """Check if position is within track bounds"""
        # Outside outer walls
        if x < 0 or x > self.outer_size or y < 0 or y > self.outer_size:
            return False
        
        # Inside inner walls
        inner_margin = (self.outer_size - self.inner_size) / 2
        if (inner_margin < x < self.outer_size - inner_margin and 
            inner_margin < y < self.outer_size - inner_margin):
            return False
        
        return True


def run_simulation(kp1=0.5, kp2=-0.75, velocity=0.3, num_laps=2, 
                   initial_offset=0.1, animate=False):
    """
    Run complete simulation with bicycle model and PID controller on WRO track
    
    Args:
        kp1: Gain for heading error
        kp2: Gain for lateral distance error
        velocity: Constant velocity (m/s)
        num_laps: Number of laps around the track
        initial_offset: Initial lateral offset from centerline (m)
        animate: Whether to show animation
        
    Returns:
        results: Dictionary with simulation results
    """
    # Simulation parameters
    dt = 0.01  # Time step (s) - 20 Hz update rate
    
    # Calculate approximate duration (perimeter = 4 * 2m sides = 8m)
    track_perimeter = 8.0  # Approximate driving distance per lap
    lap_time = track_perimeter / velocity
    duration = num_laps * lap_time + 5.0  # Add 5s buffer
    steps = int(duration / dt)
    
    # Initialize components
    robot = BicycleModel(wheelbase=0.15)
    controller = DualErrorPIDController(kp1=kp1, kp2=kp2)
    track = WROTrack()
    
    # Initial state: Starting position (bottom-right, moving right along bottom)
    initial_x = 1.5
    initial_y = 0.5 + initial_offset
    initial_theta = 0.0  # Facing right (East)
    initial_state = [initial_x, initial_y, initial_theta]
    robot.reset(initial_state)
    
    # Storage
    time_history = []
    state_history = []
    control_history = []
    
    # Simulation loop
    for i in range(steps):
        t = i * dt
        current_state = robot.state.copy()
        
        # Check if out of bounds (hit wall)
        if not track.is_in_bounds(current_state[0], current_state[1]):
            print(f"Warning: Robot out of bounds at t={t:.2f}s")
            break
        
        # Desired heading based on track section
        desired_heading = track.get_desired_heading(current_state[0], current_state[1])
        
        # Compute errors
        angle_err, dist_err = controller.compute_errors(
            current_state, 
            track,
            desired_heading
        )
        
        # Compute control
        correction = controller.compute_control(angle_err, dist_err)
        
        # Steering command (nominal + correction)
        delta_nominal = 0.0  # Straight nominal steering on straights
        delta_cmd = delta_nominal + correction
        
        # Update robot state
        robot.update(velocity, delta_cmd, dt)
        
        # Store data
        time_history.append(t)
        state_history.append(current_state)
        control_history.append(delta_cmd)
        
        # Check if completed required laps (back near start)
        if i > 100 and t > num_laps * lap_time * 0.9:
            distance_to_start = np.hypot(robot.state[0] - initial_x, 
                                        robot.state[1] - initial_y)
            if distance_to_start < 0.3:  # Within 30cm of start
                print(f"Completed {num_laps} laps at t={t:.2f}s")
                break
    
    # Convert to arrays
    time_history = np.array(time_history)
    state_history = np.array(state_history)
    control_history = np.array(control_history)
    
    # Performance metrics
    lateral_errors = []
    for state in state_history:
        lat_err = track.get_lateral_error(state[0], state[1])
        lateral_errors.append(abs(lat_err))
    lateral_errors = np.array(lateral_errors)
    
    heading_error = np.abs(np.array(controller.angle_err_history))
    
    results = {
        'time': time_history,
        'state': state_history,
        'control': control_history,
        'angle_err': np.array(controller.angle_err_history),
        'dist_err': np.array(controller.dist_err_history),
        'correction': np.array(controller.correction_history),
        'metrics': {
            'max_lateral': np.max(lateral_errors),
            'avg_lateral': np.mean(lateral_errors),
            'max_heading_err': np.max(heading_error),
            'avg_heading_err': np.mean(heading_error),
            'final_lateral': lateral_errors[-1],
            'total_time': time_history[-1]
        }
    }
    
    return results


def plot_results(results):
    """
    Plot simulation results with WRO track visualization
    
    Args:
        results: Dictionary from run_simulation
    """
    time = results['time']
    state = results['state']
    control = results['control']
    angle_err = results['angle_err']
    dist_err = results['dist_err']
    correction = results['correction']
    
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
    fig.suptitle('WRO Future Engineers - Bicycle Model PID Control Simulation', 
                 fontsize=16, fontweight='bold')
    
    # 1. Track view with trajectory (larger plot)
    ax = fig.add_subplot(gs[:2, :2])
    
    # Draw track boundaries
    # Outer walls (3m x 3m)
    outer_wall = Rectangle((0, 0), 3.0, 3.0, fill=False, 
                           edgecolor='black', linewidth=3, label='Outer Walls')
    ax.add_patch(outer_wall)
    
    # Inner walls (1m x 1m, centered)
    inner_wall = Rectangle((1.0, 1.0), 1.0, 1.0, fill=False,
                           edgecolor='black', linewidth=3, label='Inner Walls')
    ax.add_patch(inner_wall)
    
    # Draw lane centerlines
    # Bottom
    ax.plot([0, 3], [0.5, 0.5], 'g--', linewidth=1.5, alpha=0.5, label='Centerline')
    # Right
    ax.plot([2.5, 2.5], [0, 3], 'g--', linewidth=1.5, alpha=0.5)
    # Top
    ax.plot([0, 3], [2.5, 2.5], 'g--', linewidth=1.5, alpha=0.5)
    # Left
    ax.plot([0.5, 0.5], [0, 3], 'g--', linewidth=1.5, alpha=0.5)
    
    # Shade track area
    track_coords = [
        # Outer boundary
        [(0, 0), (3, 0), (3, 3), (0, 3), (0, 0)],
    ]
    for coords in track_coords:
        poly = Polygon(coords, alpha=0.05, facecolor='gray')
        ax.add_patch(poly)
    
    # Plot robot trajectory
    ax.plot(state[:, 0], state[:, 1], 'b-', linewidth=2.5, label='Robot Path', alpha=0.8)
    
    # Mark start position
    ax.plot(state[0, 0], state[0, 1], 'go', markersize=12, 
           label='Start', markeredgecolor='darkgreen', markeredgewidth=2)
    
    # Mark end position
    ax.plot(state[-1, 0], state[-1, 1], 'ro', markersize=12,
           label='End', markeredgecolor='darkred', markeredgewidth=2)
    
    # Add direction arrows at intervals
    arrow_interval = len(state) // 15
    for i in range(0, len(state), arrow_interval):
        if i < len(state) - 1:
            dx = state[i+1, 0] - state[i, 0]
            dy = state[i+1, 1] - state[i, 1]
            ax.arrow(state[i, 0], state[i, 1], dx*3, dy*3,
                    head_width=0.08, head_length=0.08, 
                    fc='blue', ec='blue', alpha=0.6)
    
    ax.set_xlabel('X Position (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=12, fontweight='bold')
    ax.set_title('WRO Track - Top View (3m × 3m)', fontsize=13, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.set_xlim(-0.2, 3.2)
    ax.set_ylim(-0.2, 3.2)
    
    # 2. Lateral error over time
    ax = fig.add_subplot(gs[0, 2])
    lateral_err = dist_err * 100  # Convert to cm
    ax.plot(time, lateral_err, 'r-', linewidth=2)
    ax.axhline(y=0, color='g', linestyle='--', linewidth=1.5, alpha=0.7)
    ax.fill_between(time, -10, 10, alpha=0.15, color='green', label='±10cm tolerance')
    ax.set_xlabel('Time (s)', fontsize=10)
    ax.set_ylabel('Lateral Error (cm)', fontsize=10)
    ax.set_title('Distance from Centerline', fontsize=11, fontweight='bold')
    ax.legend(loc='best', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # 3. Heading angle over time
    ax = fig.add_subplot(gs[1, 2])
    ax.plot(time, np.rad2deg(state[:, 2]), 'b-', linewidth=2, label='Actual')
    ax.set_xlabel('Time (s)', fontsize=10)
    ax.set_ylabel('Heading (deg)', fontsize=10)
    ax.set_title('Vehicle Heading', fontsize=11, fontweight='bold')
    ax.legend(loc='best', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # 4. Heading error
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(time, np.rad2deg(angle_err), 'b-', linewidth=2)
    ax.axhline(y=0, color='g', linestyle='--', linewidth=1.5, alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=10)
    ax.set_ylabel('Heading Error (deg)', fontsize=10)
    ax.set_title('Heading Error', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # 5. Steering control
    ax = fig.add_subplot(gs[2, 1])
    ax.plot(time, np.rad2deg(control), 'purple', linewidth=2)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.5)
    ax.set_xlabel('Time (s)', fontsize=10)
    ax.set_ylabel('Steering (deg)', fontsize=10)
    ax.set_title('Control Input', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # 6. PID correction
    ax = fig.add_subplot(gs[2, 2])
    ax.plot(time, np.rad2deg(correction), 'g-', linewidth=2)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.5)
    ax.set_xlabel('Time (s)', fontsize=10)
    ax.set_ylabel('Correction (deg)', fontsize=10)
    ax.set_title('PID Correction', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    return fig


def parameter_sensitivity_analysis():
    """
    Analyze sensitivity of controller performance to parameter variations
    """
    print("\n" + "="*70)
    print("PID PARAMETER SENSITIVITY ANALYSIS - WRO TRACK")
    print("="*70)
    
    # Test different kp1 values
    kp1_values = [0.3, 0.4, 0.5, 0.6, 0.7]
    kp2_fixed = -0.75
    
    print(f"\n1. Varying kp1 (heading error gain) with kp2 = {kp2_fixed}")
    print("-" * 70)
    print(f"{'kp1':<8} {'Max Lat (cm)':<15} {'Avg Lat (cm)':<15} {'Time (s)':<12}")
    print("-" * 70)
    
    for kp1 in kp1_values:
        results = run_simulation(kp1=kp1, kp2=kp2_fixed, num_laps=1, initial_offset=0.1)
        metrics = results['metrics']
        print(f"{kp1:<8.1f} {metrics['max_lateral']*100:<15.2f} "
              f"{metrics['avg_lateral']*100:<15.2f} {metrics['total_time']:<12.2f}")
    
    # Test different kp2 values
    kp2_values = [-0.5, -0.65, -0.75, -0.85, -1.0]
    kp1_fixed = 0.5
    
    print(f"\n2. Varying kp2 (lateral error gain) with kp1 = {kp1_fixed}")
    print("-" * 70)
    print(f"{'kp2':<8} {'Max Lat (cm)':<15} {'Avg Lat (cm)':<15} {'Time (s)':<12}")
    print("-" * 70)
    
    for kp2 in kp2_values:
        results = run_simulation(kp1=kp1_fixed, kp2=kp2, num_laps=1, initial_offset=0.1)
        metrics = results['metrics']
        print(f"{kp2:<8.2f} {metrics['max_lateral']*100:<15.2f} "
              f"{metrics['avg_lateral']*100:<15.2f} {metrics['total_time']:<12.2f}")


def compare_controllers():
    """
    Compare different controller configurations on WRO track
    """
    configurations = [
        {'name': 'Default', 'kp1': 0.5, 'kp2': -0.75},
        {'name': 'High kp1', 'kp1': 0.7, 'kp2': -0.75},
        {'name': 'High kp2', 'kp1': 0.5, 'kp2': -1.0},
        {'name': 'Conservative', 'kp1': 0.3, 'kp2': -0.5},
    ]
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Controller Configuration Comparison - WRO Track', 
                 fontsize=16, fontweight='bold')
    
    for idx, config in enumerate(configurations):
        results = run_simulation(kp1=config['kp1'], kp2=config['kp2'], 
                               num_laps=1, initial_offset=0.1)
        
        ax = axes[idx // 2, idx % 2]
        time = results['time']
        dist_err = results['dist_err'] * 100  # Convert to cm
        
        ax.plot(time, dist_err, linewidth=2)
        ax.axhline(y=0, color='g', linestyle='--', linewidth=1.5, alpha=0.7)
        ax.fill_between(time, -10, 10, alpha=0.2, color='green')
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Lateral Error (cm)', fontsize=11)
        ax.set_title(f"{config['name']}: kp1={config['kp1']}, kp2={config['kp2']}", 
                    fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Add metrics text
        metrics = results['metrics']
        text = f"Max: {metrics['max_lateral']*100:.1f}cm\n"
        text += f"Avg: {metrics['avg_lateral']*100:.1f}cm\n"
        text += f"Time: {metrics['total_time']:.1f}s"
        ax.text(0.02, 0.98, text, transform=ax.transAxes, 
               fontsize=9, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    return fig


def main():
    """
    Main function to run complete simulation suite
    """
    print("\n" + "="*70)
    print(" " * 10 + "WRO FUTURE ENGINEERS - BICYCLE MODEL SIMULATION")
    print(" " * 15 + "Track: 3m × 3m Square with 1m Wide Lanes")
    print(" " * 20 + "SQ-391 Team")
    print("="*70)
    
    # Run main simulation with default parameters
    print("\nRunning simulation with default parameters...")
    print(f"  Track: 3m × 3m square with 1m lane width")
    print(f"  kp1 (heading error gain): 0.5")
    print(f"  kp2 (lateral error gain): -0.75")
    print(f"  Velocity: 0.3 m/s")
    print(f"  Number of laps: 2")
    print(f"  Initial offset: 0.1 m (10 cm from centerline)")
    
    results = run_simulation(kp1=0.5, kp2=-0.75, velocity=0.3, 
                            num_laps=2, initial_offset=0.1,animate=True)
    
    # Display metrics
    print("\n" + "-"*70)
    print("PERFORMANCE METRICS")
    print("-"*70)
    metrics = results['metrics']
    print(f"Maximum lateral error:      {metrics['max_lateral']*100:6.2f} cm")
    print(f"Average lateral error:      {metrics['avg_lateral']*100:6.2f} cm")
    print(f"Final lateral error:        {metrics['final_lateral']*100:6.2f} cm")
    print(f"Maximum heading error:      {np.rad2deg(metrics['max_heading_err']):6.2f} deg")
    print(f"Average heading error:      {np.rad2deg(metrics['avg_heading_err']):6.2f} deg")
    print(f"Total lap time:             {metrics['total_time']:6.2f} s")
    print("-"*70)
    
    # Plot results
    print("\nGenerating plots...")
    fig1 = plot_results(results)
    
    # Parameter sensitivity analysis
    parameter_sensitivity_analysis()
    
    # Compare different controller configurations
    print("\nComparing different controller configurations...")
    fig2 = compare_controllers()
    
    print("\n" + "="*70)
    print("Simulation complete! Close plot windows to exit.")
    print("="*70 + "\n")
    
    plt.show()


if __name__ == "__main__":
    main()
