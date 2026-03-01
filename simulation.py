import math
import matplotlib.pyplot as plt


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        return output


class Drone:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def update(self, vx, vy, dt):
        self.x += vx * dt
        self.y += vy * dt


class Simulation:
    def __init__(self):

        # drone başlangıç konumu
        self.drone = Drone(0, 0)

        # hedef konumu
        self.target = (10, 10)

        # PID controller
        self.pid_x = PID(1.2, 0.0, 0.3)
        self.pid_y = PID(1.2, 0.0, 0.3)

        # kayıt
        self.history = []

        # sim parametreleri
        self.dt = 0.1
        self.time = 0
        self.max_time = 20


    def run(self):

        while self.time < self.max_time:

            error_x = self.target[0] - self.drone.x
            error_y = self.target[1] - self.drone.y

            vx = self.pid_x.update(error_x, self.dt)
            vy = self.pid_y.update(error_y, self.dt)

            self.drone.update(vx, vy, self.dt)

            self.history.append((self.drone.x, self.drone.y))

            distance = math.sqrt(error_x**2 + error_y**2)

            if distance < 0.05:
                print("Target reached")
                break

            self.time += self.dt


    def plot(self):

        drone_x = [p[0] for p in self.history]
        drone_y = [p[1] for p in self.history]

        target_x = self.target[0]
        target_y = self.target[1]

        plt.figure()

        plt.scatter(target_x, target_y, color="red", label="Target")

        plt.plot(drone_x, drone_y, color="blue", label="Drone Path")

        plt.scatter(drone_x[-1], drone_y[-1], color="blue")

        plt.title("UAV PID Flight Simulation")
        plt.xlabel("X")
        plt.ylabel("Y")

        plt.legend()
        plt.grid()

        plt.show()