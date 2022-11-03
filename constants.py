
import math

# Motor
encoderTickPerRevolution = 1200
wheelRadius = 0.04              # [m]
wheelCircumference = 2 * math.pi * wheelRadius  # 0.25132 [m]
distancePerRevolution = wheelCircumference      # 0.25132 [m]
distancePerTick = distancePerRevolution / encoderTickPerRevolution  # 0.000209 [m]
distanceBetweenWheels = 0.24    # [m]
maxRPM = 600                    # [RPM = rounds per minute]
maxPWM = 255
maxVelocity = (maxRPM/60)*distancePerRevolution  # 2.51327 [m/s]
wheelSpeedOffset = 0.4          # [% (eg. 70 means 70%)]
#motorGear = 1/10

# Odometry
# Start position
#startPosition_x = 0       # [m]
#startPosition_y = 0       # [m]

# Start orientation
#roll = 0    # [degrees]
#pitch = 0   # [degrees]
#yaw = 0     # [degrees]
#startOrientation_theta = 0   # [rad]