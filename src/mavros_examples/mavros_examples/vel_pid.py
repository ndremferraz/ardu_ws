#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

class PID:
    def __init__(self, kp, ki, kd, output_limit, integral_limit):
        self.kp = np.array(kp, dtype=float)
        self.ki = np.array(ki, dtype=float)
        self.kd = np.array(kd, dtype=float)

        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.integral = np.zeros(3, dtype=float)
        self.prev_error = np.zeros(3, dtype=float)

    def reset(self):
        self.integral[:] = 0
        self.prev_error[:] = 0

    def update(self, error, dt):

        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        if dt > 1e-9:
            derivative = (error - self.prev_error) / dt
        else: 
            derivative = np.zeros_like(error)
            
        self.prev_error = error.copy()
        

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, -self.output_limit, self.output_limit)

        return output
    
class WaypointController:
    def __init__(self, kp_pos=1.75, max_speed=0.5): 

        self.kp_pos = kp_pos
        self.max_speed = max_speed

        self.vel_pid = PID(kp=1.75,ki=0.1,kd=0.1,output_limit=2.0, integral_limit=2.0)

        self.position_tolerance = 0.10
        self.velocity_tolerance = 0.10

    def clamp_vector(self, vec, max_mag):
        mag = np.linalg.norm(vec)
        if mag > max_mag and mag > 1e-9:
            vec = vec * (max_mag / mag)
        return vec
    
    def update(self, current_position, velocity, target_position, dt):

        position_error = target_position - current_position
        vel_des = self.kp_pos * position_error
        vel_des = self.clamp_vector(vel_des, self.max_speed)

        vel_error = vel_des - velocity
        command = self.vel_pid.update(vel_error, dt)

        reached = (
            np.linalg.norm(position_error) < self.position_tolerance and
            np.linalg.norm(velocity) < self.velocity_tolerance
        )

        return reached, command

def plot_position_3d(time, x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(x, y, z, color="0.7", linewidth=1.5)
    trajectory = ax.scatter(x, y, z, c=time, cmap="viridis", s=20)

    ax.scatter(x[0], y[0], z[0], color="green", s=60, label="start")
    ax.scatter(x[-1], y[-1], z[-1], color="red", s=60, label="end")

    ax.set_title("Object Position Over Time")
    ax.set_xlabel("X position")
    ax.set_ylabel("Y position")
    ax.set_zlabel("Z position")
    ax.legend()

    colorbar = fig.colorbar(trajectory, ax=ax, pad=0.1)
    colorbar.set_label("Time")

    plt.show()


def plot_position_vs_time(time, x, y, z):
    fig, ax = plt.subplots()

    ax.plot(time, x, label="x position")
    ax.plot(time, y, label="y position")
    ax.plot(time, z, label="z position")

    ax.set_title("Position Components Over Time")
    ax.set_xlabel("Time")
    ax.set_ylabel("Position")
    ax.grid(True)
    ax.legend()

    plt.show()

'''
def main():
    controller = WaypointController()

    t = np.linspace(0, 2*np.pi, 15)
    x = np.sin(t) * 2
    y = np.sin(2*t)

    waypoints_xy = np.vstack((x, y)).T
    z = np.full((waypoints_xy.shape[0], 1), 1.5)
    waypoints = np.concatenate((waypoints_xy, z), axis=1)

    velocity = np.array([0.0, 0.0, 0.0])
    position = np.array([0.0, 0.0, 0.0])
    dt = 0.1
    t = 0
    max_time = 60.0

    time_history = [t]
    position_history = [position.copy()]

    for point in waypoints:
        reached, cmd = controller.update(position, velocity, point, dt)
        while not reached and t < max_time:
            t += dt
            print(f"time {t}")
            
            velocity += cmd * dt
            position += velocity * dt

            time_history.append(t)
            position_history.append(position.copy())

            reached, cmd = controller.update(position, velocity, point, dt)

    position_history = np.array(position_history)
    time_history = np.array(time_history)

    x_pos = position_history[:, 0]
    y_pos = position_history[:, 1]
    z_pos = position_history[:, 2]

    plot_position_3d(time_history, x_pos, y_pos, z_pos)
    plot_position_vs_time(time_history, x_pos, y_pos, z_pos)

if __name__ == "__main__":
    main()
'''

