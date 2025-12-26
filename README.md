# A Beginner's Guide to Holonomic Robot Control with ROS2
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
