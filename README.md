# A Beginner's Guide to Holonomic Robot Control with ROS2

https://github.com/user-attachments/assets/7327101a-ef7d-4c2d-8e08-1557c5b62c47

Imagine you are driving a car. To park in a tight spot, you have to drive forward, back up, turn the wheel, and repeat. It's a hassle because a standard car is **non-holonomic**. It cannot slide sideways. Now imagine a robot that can move in any direction instantly- forward, backward, left, right, or diagonally without turning its body first. This is called **Holonomic Drive**.
The robot we are looking at today uses a specific type of holonomic drive called **Swerve Drive**. It has four wheels, and unlike a car, each wheel can independently:
1. **Steer**: Rotating around a vertical axis to face any direction (like a caster wheel on a chair, but motorized)
2. **Drive**: Spinning to move the robot (like a car wheel)

## The Robot Anatomy (URDF)
- **Base Link**: The central chassis
- **Steer Link**: Attached to the base, it rotates (Steers)
- **Wheel Link**: Attached to the steer link, it spins (Drives)

## Inverse Kinematics
![Image](https://github.com/user-attachments/assets/d5edf45a-f855-4752-9130-01cfde967131)
The robot receives a simple command from the pilot or an autonomous algorithm: "Move at velocity $V_x$ (forward), $V_y$ (strafe left), and rotate at speed $\omega$ (omega)."
However, the motors don't understand "move forward." They only understand:
- **Steering Motor 1:** Go to angle $\phi$
- **Drive Motor 1:** Spin at speed $S$
The process of converting **Body Motion** $(V_x, V_y, \omega)$ into **Wheel Commands** $(\phi, S)$ is called **Inverse Kinematics (IK)**.
To calculate the desired speed for a single wheel, we must combine three movements:
1. **Linear Forward Motion:** The robot moves at $V_x$
2. **Linear Sideways Motion:** The robot moves at $V_y$
3. **Rotational Motion:** The robot spins around the center
If the robot spins, a wheel far from the center moves faster than one near the center. If the robot rotates counter-clockwise, the wheels on the right side must move "forward" relative to the body, and wheels on the left must move "backward."
Mathematically, the velocity vector at the center of a specific wheel $i$ is:

$$\vec{V}_{x,wheel} = V_x - (\omega \cdot y_{pos}) \quad \vec{V}_{y,wheel} = V_y + (\omega \cdot x_{pos})$$

> **Note:** We subtract $\omega \cdot y$ for the X-component because if you rotate CCW (+ω), the part of the robot at positive Y actually moves backwards in X.

Once we have the velocity vector $(\vec{V}_{x,wheel}, \vec{V}_{y,wheel})$, we can tell the motors what to do:

#### 1. Steering Angle (φ): Which way should the wheel point?

$$\phi = \arctan2(V_{y,wheel}, V_{x,wheel})$$

#### 2. Wheel Speed (S): How fast should the wheel spin?

$$S = \frac{\sqrt{V_{x,wheel}^2 + V_{y,wheel}^2}}{R_{wheel}}$$

## Wheel Velocities

Let's look at how the C++ function `computeInverseKinematics` implements exactly these equations.
```cpp
void SwerveController::computeInverseKinematics(
    double vx, double vy, double omega,
    std::array<double, 4> & wheel_velocities,
    std::array<double, 4> & steer_angles)
{
  for (size_t i = 0; i < 4; ++i)
  {
    // --- 1. Calculate the Vector ---
    // Corresponds to: Vx = vx - ω * y (omega * y)
    double vx_wheel = vx - omega * wheel_modules_[i].y;
    
    // Corresponds to: Vy = vy + ω * x (omega * x)
    double vy_wheel = vy + omega * wheel_modules_[i].x;
    
    // Calculate magnitude of the vector (total speed)
    double wheel_speed = std::hypot(vx_wheel, vy_wheel);
    
    // --- 2. Calculate Angle ---
    // Corresponds to: phi = atan2(vy, vx)
    // Note: The code has to adjust for the swerve's kinematics, likely due to coordinate frame conversions in URDF
    double angle = std::atan2(vy_wheel, vx_wheel);
    
    if (wheel_speed < 1e-6)
    {
      // If the robot is stopped, don't change steering angle (avoids jitter)
      wheel_velocities[i] = 0.0;
      steer_angles[i] = wheel_modules_[i].steer_angle_prev;
    }
    else
    {
      steer_angles[i] = angle;
      
      // --- 3. Calculate Wheel RPM (rad/s) ---
      // Corresponds to: S = |Velocity| / Radius
      wheel_velocities[i] = wheel_speed / radius_;
      
      // Clamp to physical limits
      // (std::abs(wheel_velocities[i]) > max_wheel_velocity_)
      
      wheel_velocities[i] = std::clamp(wheel_velocities[i], -max_wheel_velocity_, max_wheel_velocity_);
    }
  }
}
```

The function runs for all 4 wheels, filling the `wheel_velocities` and `steer_angles` arrays with the exact commands needed to achieve the requested movement.

## Steering Optimization

There is a clever trick in swerve drives. If a wheel is currently pointing **East** (0 degrees) and we want to drive **West**, the math says "Rotate 180 degrees to West, then spin forward."

However, it is much faster to simply rotate 90 degrees (to South or North) and spin the wheel *backwards*. The result is the same: the robot moves West.

The code handles this in `optimizeSteeringAngles`. It checks if flipping the wheel 180 degrees and reversing the motor speed is faster than rotating the long way around.
```cpp
void OmniBaseController::optimizeSteeringAngles(
    std::array<double, 4>& & wheel_velocities,
    std::array<double, 4>& & steer_angles)
{
  for (size_t i = 0; i < 4; ++i)
  {
    if (std::abs(wheel_velocities[i]) < 1e-6) continue; // Don't optimize if stopped
    
    // Get current angle from hardware
    double current_angle = state_interfaces_[...].value();
    double target_angle = steer_angles[i];
    
    // ... (Normalization logic handled in normalizeAngle) ...
    
    // 1. Calculate rotation to the "Normal" target
    double direct_rotation = std::abs(normalizeAngle(target_angle - current_angle));
    
    // 2. Calculate rotation to the "Flipped" target (Target + 180 degrees)
    double flipped_target = normalizeAngle(target_angle + M_PI);
    double flipped_rotation = std::abs(normalizeAngle(flipped_target - current_angle));
    
    // 3. Decide: Is the flipped path shorter?
    if (flipped_rotation < direct_rotation)
    {
      // Flip the steering command 180 degrees
      steer_angles[i] = flipped_target;
      // Reverse the wheel speed (Drive backwards to go forwards)
      wheel_velocities[i] = -wheel_velocities[i];
    }
  }
}
```

## Launch command:
*ros2 launch testAMR7 testAMR7.launch.py*
**This repo inlcudes a custom hardware interface as well for serial communications** 
When using real-hardware:
*ros2 launch testAMR7 testAMR7.launch.py use_sim:=false*
