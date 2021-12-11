#!/usr/bin/env python3


import random
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
from IPython import embed


class Robot:
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return "[x=%.5f y=%.5f orient=%.5f]" % (self.x, self.y, self.orientation)


class PID:
    _kp = None
    _ki = None
    _kd = None
    _prev_error = None
    _p_error = None
    _i_error = None
    _d_error = None

    def __init__(self, kp, ki, kd):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self.reset()

    def reset(self):
        self._p_error = 0.0
        self._i_error = 0.0
        self._d_error = 0.0
        self._counter = 0

    def update(self, error):
        if self._prev_error is None:
            self._prev_error = error

        self._p_error = error

        self._i_error += error

        self._d_error = error - self._prev_error

        self._prev_error = error

    @property
    def error(self):
        return (
            self._kp * self._p_error
            + self._ki * self._i_error
            + self._kd * self._d_error
        )


def make_robot(x=0.0, y=1.0, orientation=0.0):
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    robot = Robot()
    robot.set(x, y, orientation)
    robot.set_steering_drift(10 / 180 * np.pi)
    return robot


def run(robot, params, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []

    robots = []
    pid = PID(*params)
    total_squared_error = 0.0
    robots = [deepcopy(robot)]
    for i in range(n):
        cte = robot.y
        pid.update(cte)
        steer = -pid.error
        robot.move(steer, speed)
        robots.append(deepcopy(robot))
        total_squared_error += cte ** 2
    total_squared_error /= n
    return robots, total_squared_error


def optimize(x, y, tol=0.1, speed=1.0, n=100):
    p = [0.0, 0.0, 0.0]
    dp = [3.0, 3.0, 3.0]
    robot = make_robot(x=x, y=y)
    robots, best_err = run(robot, p, speed=speed, n=n)

    while sum(dp) > tol:
        for i in range(len(p)):
            robot = make_robot(y=y)
            p[i] += dp[i]
            robots, err = run(robot, p, speed=speed, n=n)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                # In opposite direction
                p[i] -= 2 * dp[i]
                robot = make_robot(y=y)
                robots, err = run(robot, p, speed=speed, n=n)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
    return p


def plot_robots(robots):
    poses = np.asarray([[robot.x, robot.y, robot.orientation] for robot in robots])
    plt.plot(poses[:, 0], poses[:, 1], marker="o", alpha=0.5)


if __name__ == "__main__":
    # run - does a single control run
    n = 100
    x = 0.0
    y = 2.0
    orientation = 0.0
    speed = 1.0
    plt.figure(figsize=(8, 8))
    plt.ioff()
    plt.hlines(y=0.0, xmin=-10.0, xmax=100.0, color="r", label="Reference")

    robot = make_robot(x=x, y=y, orientation=orientation)

    # Simulate perfect execution
    for i in range(20):
        print(i)
        p = optimize(robot.x, robot.y, tol=0.1, speed=speed, n=n)
        robots, error = run(deepcopy(robot), p, n=n, speed=speed)
        plot_robots(robots)
        robot = robots[1]

    plt.axis("equal")
    plt.ion()
    plt.show()

    embed()
