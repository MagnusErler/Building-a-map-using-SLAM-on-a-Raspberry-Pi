
import math

# Motor
encoderTickPerRevolution = 1200
wheelRadius = 0.04              # [m]
wheelCircumference = 2 * math.pi * wheelRadius  # 0.25132 [m]
distancePerRevolution = wheelCircumference      # 0.25132 [m]
distancePerTick = distancePerRevolution / encoderTickPerRevolution  # 0.000209 [m]
distanceBetweenWheels = 0.24    # [m]
maxRPM = 600                    # [RPM = rounds per minute]
maxPWM = 255                    # [PWM]
maxVelocity = (maxRPM/60)*distancePerRevolution # 2.51327 [m/s]
motorSpeedOffset = -1          # [% (eg. 70 means 70%)]
