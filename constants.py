
import math


# Motor
encoderTickPerRevolution = 1200
wheelRadius = 0.04              # [m]
wheelCircumference = 2 * math.pi * wheelRadius  # [m]
distancePerRevolution = wheelCircumference  # [m]
distancePerTick = wheelCircumference / encoderTickPerRevolution    # [m]
distanceBetweenWheels = 0.24    # [m]
maxRPM = 600    # [RPM]
#motorGear = 1/10

# Odometry
# Start position
x = 0
y = 0
theta = 0