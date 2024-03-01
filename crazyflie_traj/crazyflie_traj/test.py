import numpy as np
import matplotlib.pyplot as plt

def generate_spiral_trajectory(radius, pitch, height, speed, num_points):
    theta = np.linspace(-np.pi, 10*np.pi, num_points)
    z = radius * np.cos(theta) + height
    y = radius * np.sin(theta)
    x = pitch * theta 
    time = np.linspace(0, 2*np.pi, num_points) / speed

    return x, y, z, time

# Define parameters
radius = 1  # Radius of the spiral
pitch = 1    # Pitch of the spiral
height = 5   # Starting height
speed = 1    # Speed of the drone along the spiral
num_points = 100  # Number of points to generate

# Generate spiral trajectory
x, y, z, time = generate_spiral_trajectory(radius, pitch, height, speed, num_points)

# Plot trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Spiral Trajectory')
plt.show()
