import numpy as np

class Drone2D:
    def __init__(self):
        self.position = np.array([0.0, 0.0])
        self.velocity = np.array([0.0, 0.0])

    def update(self, acceleration, dt):
        self.velocity += acceleration * dt
        self.position += self.velocity * dt