
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from yaw_controller import YawController
from pid import PID


class TwistController(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        
        #vehicle parameters
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        
        # PID gain for throttle control
        self.kp = 10
        self.kd = 1
        self.ki = 0.1
        
        self.min_speed = 0.0
        
        self.pid_controller = PID(self.kp, self.ki, self.kd, 0.0, self.accel_limit)

        # Braking F = ma --> 10m/s**2 max a
        # self.nm_max = 10.0 * self.vehicle_mass * self.wheel_radius
        
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

    def control(self, velocity_cmd, current_velocity, acceleration, angular_velocity_cmd, dt, dbw_enabled):
        
        if dbw_enabled:
            steer = self.yaw_controller.get_steering(velocity_cmd, angular_velocity_cmd, current_velocity)
            # apply brakes if negative acceleration is above deadband, or if velocity_cmd is very small, else apply accelerator
            if (acceleration >= 0) or ( (self.brake_deadband*-1.0 < acceleration < 0) and velocity_cmd > 0.5):
                error = velocity_cmd - current_velocity
                throttle = self.pid_controller.step(error, dt)
                brake = 0.0
            else:
                error = current_velocity - velocity_cmd
                brake = abs(acceleration) * self.vehicle_mass * self.wheel_radius
                brake = max(brake, abs(self.decel_limit))
                throttle = 0.0
                self.pid_controller.reset()
        else:
            self.pid_controller.reset()
            throttle = 0.0
            brake = 0.0
            steer = 0.0

        return throttle, brake, steer
