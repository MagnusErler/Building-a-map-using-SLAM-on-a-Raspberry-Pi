
import math

# Motor
encoderTickPerRevolution = 1200
wheelRadius = 0.04              # [m]
wheelCircumference = 2 * math.pi * wheelRadius  # 0.25132 [m]
wheelSpeedOffset = 0.4    # [%]
distancePerRevolution = wheelCircumference      # 0.25132 [m]
distancePerTick = distancePerRevolution / encoderTickPerRevolution  # 0.000209 [m]
distanceBetweenWheels = 0.24    # [m]
maxRPM = 600                    # [RPM = rounds per minute]
maxPWM = 255
maxVelocity = (maxRPM/60)*distancePerRevolution  # 2.51327 [m/s]
#motorGear = 1/10

# Odometry
# Start position
x = 0       # [m]
y = 0       # [m]
theta = 0   # [rad]

# Start orientation
roll = 0    # [degrees]
pitch = 0   # [degrees]
yaw = 0     # [degrees]