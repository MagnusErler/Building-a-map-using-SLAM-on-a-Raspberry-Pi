
import math


# Motor
encoderTickPerRevolution = 1200
wheelRadius = 0.04  # [m]
wheelCircumference = 2 * math.pi * wheelRadius  # 0.25132 [m]
distancePerRevolution = wheelCircumference      # 0.25132 [m]
distancePerTick = distancePerRevolution / encoderTickPerRevolution  # 0.000209 [m]
distanceBetweenWheels = 0.24    # [m]
maxRPM = 600        # [RPM]
#motorGear = 1/10

# Odometry
# Start position
x = 0       # [m]
y = 0       # [m]
theta = 0   #