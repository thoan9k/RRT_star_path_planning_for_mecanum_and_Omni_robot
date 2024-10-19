import numpy as np


class MecanumRobot:
    def __init__(self, r, lx, ly):
        """
        Initialize the Mecanum wheeled robot.
        :param r: wheel radius (m)
        :param lx: half of the distance between front wheels (m)
        :param ly: half of the distance between front and rear wheels (m)
        """
        self.r = r
        self.lx = lx
        self.ly = ly
        self.position = np.array([0.0, 0.0, 0.0])  # [x, y, theta]

    def inverse_kinematics(self, v_x, v_y, omega_):
        """
        Calculate wheel velocities given robot velocities (inverse kinematics).
        :param v_x: robot x velocity (m/s)
        :param v_y: robot y velocity (m/s)
        :param omega_: robot angular velocity (rad/s)
        :return: wheel velocities [w1, w2, w3, w4] (rad/s)
        """
        return np.array([
            (v_x - v_y - (self.lx + self.ly) * omega_) / self.r,
            (v_x + v_y + (self.lx + self.ly) * omega_) / self.r,
            (v_x + v_y - (self.lx + self.ly) * omega_) / self.r,
            (v_x - v_y + (self.lx + self.ly) * omega_) / self.r
        ])

    def forward_kinematics(self, w1, w2, w3, w4):
        """
        Calculate robot velocities given wheel velocities (forward kinematics).
        :param w1: wheel 1 angular velocity (rad/s)
        :param w2: wheel 2 angular velocity (rad/s)
        :param w3: wheel 3 angular velocity (rad/s)
        :param w4: wheel 4 angular velocity (rad/s)
        :return: robot velocities [vx, vy, omega]
        """
        v_x = self.r * (w1 + w2 + w3 + w4) / 4
        v_y = self.r * (-w1 + w2 + w3 - w4) / 4
        omega_ = self.r * (-w1 + w2 - w3 + w4) / (4 * (self.lx + self.ly))

        return np.array([v_x, v_y, omega_])

    def resultant_velocity(self, v_x, v_y):
        """
        Calculate resultant velocity and its direction.
        :param v_x: robot x velocity (m/s)
        :param v_y: robot y velocity (m/s)
        :return: resultant velocity (m/s) and direction (radians)
        """
        v_r = np.sqrt(v_x ** 2 + v_y ** 2)
        rho = np.arctan2(v_y, v_x)
        return v_r, rho

    def update_position(self, v_x, v_y, omega_, d_t):
        """
        Update the robot's position based on its velocities.
        :param v_x: robot x velocity (m/s)
        :param v_y: robot y velocity (m/s)
        :param omega_: robot angular velocity (rad/s)
        :param d_t: time step (s)
        """
        # Rotate velocity vector by current orientation
        cos_theta = np.cos(self.position[2])
        sin_theta = np.sin(self.position[2])
        dx = (v_x * cos_theta - v_y * sin_theta) * d_t
        dy = (v_x * sin_theta + v_y * cos_theta) * d_t
        dtheta = omega_ * d_t

        self.position += np.array([dx, dy, dtheta])

    def simulate_motion(self, v_x, v_y, omega_, duration_, d_t):
        """
        Simulate the robot's motion over a period of time.
        :param v_x: constant x velocity (m/s)
        :param v_y: constant y velocity (m/s)
        :param omega_: constant angular velocity (rad/s)
        :param duration_: total simulation time (s)
        :param d_t: time step for simulation (s)
        :return: array of positions over time
        """
        steps = int(duration_ / d_t)
        positions_ = np.zeros((steps, 3))

        for i in range(steps):
            self.update_position(v_x, v_y, omega_, d_t)
            positions_[i] = self.position

        return positions_


# Example usage and testing
robot = MecanumRobot(r=0.1, lx=0.2, ly=0.2)

print("Example 1: Moving forward")
vx, vy, omega = 1, 0, 0  # Move forward at 1 m/s
wheel_velocities = robot.inverse_kinematics(vx, vy, omega)
print("Input velocities [vx, vy, omega]:", [vx, vy, omega])
print("Wheel velocities [w1, w2, w3, w4]:", wheel_velocities)

print("\nExample 2: Forward kinematics check")
robot_velocities = robot.forward_kinematics(*wheel_velocities)
print("Calculated robot velocities:", robot_velocities)
print("Error:", np.abs(np.array([vx, vy, omega]) - robot_velocities))

print("\nExample 3: Diagonal movement")
vx, vy, omega = 1, 1, 0  # Move diagonally
wheel_velocities = robot.inverse_kinematics(vx, vy, omega)
print("Input velocities [vx, vy, omega]:", [vx, vy, omega])
print("Wheel velocities [w1, w2, w3, w4]:", wheel_velocities)

print("\nExample 4: Rotation")
vx, vy, omega = 0, 0, 1  # Rotate in place
wheel_velocities = robot.inverse_kinematics(vx, vy, omega)
print("Input velocities [vx, vy, omega]:", [vx, vy, omega])
print("Wheel velocities [w1, w2, w3, w4]:", wheel_velocities)

print("\nExample 5: Simulating motion")
vx, vy, omega = 1, 0, 0.5  # Move forward while rotating
duration = 5  # seconds
dt = 0.1  # time step
positions = robot.simulate_motion(vx, vy, omega, duration, dt)
print(f"Initial position: {positions[0]}")
print(f"Final position: {positions[-1]}")
