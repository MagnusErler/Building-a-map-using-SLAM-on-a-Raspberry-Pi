
import math


# Motor
encoderTickPerRevolution = 1200
wheelRadius = 0.04              # [m]
distancePerTick = (2 * math.pi * wheelRadius) / encoderTickPerRevolution    # [m]
distanceBetweenWheels = 0.24    # [m]
#motorGear = 1/10

# Odometry
# Start position
x = 0
y = 0
theta = 0