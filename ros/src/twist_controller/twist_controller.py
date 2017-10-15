
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
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
        self.kp = 0.5
        self.kd = 0.01
        self.ki = 0.001
        
        self.min_speed = 0.0
        
        self.pid_controller = PID(self.kp, self.ki, self.kd, -1.0, 1.0)

        # Braking F = ma --> 10m/s**2 max a
        self.nm_max = 10.0 * self.vehicle_mass * self.wheel_radius
        
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

    def control(self, velocity_cmd, current_velocity, acceleration, angular_velocity_cmd, dt, dbw_enabled):
        
        if dbw_enabled:
            steer = self.yaw_controller.get_steering(velocity_cmd, angular_velocity_cmd, current_velocity)
            error = velocity_cmd - current_velocity
            sctl = self.pid_controller.step(error, dt)
            if velocity_cmd < 0.1 and error < 0.0 and abs(error) < 0.1:
                # if we're essentially stopped, stand on the brake instead of creeping
                throttle = 0.0
                brake = 0.05 * self.nm_max
            elif sctl > 0.0:
                throttle = sctl
                brake = 0.0
            else:
                throttle = 0.0
                brake = -1.0 * self.nm_max * sctl 
        else:
            self.pid_controller.reset()
            throttle = 0.0
            brake = 0.0
            steer = 0.0

        return throttle, brake, steer
