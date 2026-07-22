---
name: "trajectory-planning"
description: "Provides algorithms and templates for robotic arm trajectory planning. Invoke when the user wants to generate smooth paths, interpolate points, or plan arm movements."
---

# Trajectory Planning (ROS2)

This skill provides methodologies and code examples for generating smooth, collision-free paths for robotic arms.

## When to Invoke
- When the user asks to move the arm from Point A to Point B smoothly.
- When generating intermediate waypoints (interpolation).
- When smoothing out raw sensor data to act as a target trajectory.

## Best Practices

1. **Interpolation Methods**:
   - **Linear Interpolation (Lerp)**: Good for simple, slow movements.
   - **Cubic Splines / Minimum Jerk Trajectories**: Essential for physical robotic arms to prevent sudden acceleration spikes that can damage motors.

2. **Time Parameterization**:
   - Ensure every generated waypoint has a strictly increasing timestamp.
   - Use `trajectory_msgs/msg/JointTrajectoryPoint` where `time_from_start` is explicitly defined.

3. **Collision Avoidance**:
   - Always validate the intermediate points of a planned trajectory against joint limits.
   - (Advanced) Use MoveIt 2 for full 3D environment collision checking.

## Template: Cubic Interpolation
```python
import numpy as np

def generate_cubic_trajectory(start_pos, end_pos, duration, steps):
    """
    Generates a smooth cubic trajectory from start_pos to end_pos.
    Returns an array of positions.
    """
    t = np.linspace(0, duration, steps)
    positions = []
    
    for time in t:
        # Cubic polynomial coefficients for start_vel=0, end_vel=0
        tau = time / duration
        s = 3 * (tau ** 2) - 2 * (tau ** 3)
        current_pos = start_pos + s * (end_pos - start_pos)
        positions.append(current_pos)
        
    return positions
```
