import time
import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    """
    def __init__(self, *args, **kwargs):
        pass
    """
    def __init__(self, vehicle_mass, brake_deadband, wheel_radius, decel_limit, wheel_base,
        steer_ratio, max_lat_accel, max_steer_angle, Kp, Ki, Kd):
        self.minimum_speed      = 1.0 * ONE_MPH
        self.throttle_pid       = PID(Kp, Ki, Kd)
        self.yaw_control        = YawController(wheel_base, steer_ratio, self.minimum_speed, max_lat_accel, max_steer_angle)

        self.brake_deadband     = brake_deadband
        self.vehicle_mass       = vehicle_mass
        self.wheel_radius       = wheel_radius
        self.deceleration_limit = decel_limit

        self.last_update_time      = None
        self.maximum_speed      = 0

    """
    def control(self, *args, **kwargs):
        return 1.0,0.0,0.0

    """
    def control(self, target_v, target_omega, current_v, dbw_enabled):

        if self.last_update_time is None or not dbw_enabled:
            self.last_update_time = time.time()
            return 0.0, 0.0, 0.0

        if target_v.x > self.maximum_speed:
            self.maximum_speed = target_v.x

        delta_time = time.time() - self.last_update_time

        # Maximum speed is in mph
        pid_error = min(target_v.x, self.maximum_speed) - current_v.x
        #rospy.logwarn("Error {}".format(error))

        throttle = self.throttle_pid.step(pid_error, delta_time)
        #rospy.logwarn("Throttle {}".format(throttle))

        # Max throttle is limited to 1.0
        throttle = max(0.0, min(1.0, throttle))

        if pid_error < 0: # Needs to decelerate

            deceleration = abs(pid_error) / delta_time

            if abs(deceleration) > abs(self.deceleration_limit)*500:
                deceleration = self.deceleration_limit * 500
            brake = self.vehicle_mass * deceleration * self.wheel_radius
            if brake < self.brake_deadband:
                brake = 0.0
            throttle = 0.0
        else:
            brake = 0.0


        # Steering control is using Yaw Control..
        steer = self.yaw_control.get_steering(target_v.x, target_omega.z, current_v.x)
        self.last_update_time = time.time()

        #rospy.logwarn("Throttle {}".format(throttle))
        #rospy.logwarn("Brake {}".format(brake))
        #rospy.logwarn("Steer {}".format(steer))
        return throttle, brake, steer
