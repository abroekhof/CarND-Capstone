#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math

from twist_controller import TwistController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
                                         
                                    

        # TODO: Create `TwistController` object
        self.controller = TwistController(vehicle_mass, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        self.current_velocity_sub = rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_callback)
        self.twist_cmd_sub = rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cmd_callback)
        self.dbw_enabled_sub = rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_enabled_callback)
        
        self.pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        self.waypoint_sub = rospy.Subscriber('final_waypoints', Lane, self.waypoint_callback)
        
        #set up class variables to store data from subscribers
        self.current_velocity = 0.0     
        self.velocity_cmd = 0.0
        self.angular_velocity_cmd = 0.0
        self.dbw_enabled = False
        self.car_position = [0, 0, 0]
        self.waypoint_position = [0, 0, 0]
        
        #set up timestamp for measuring actual cycle time
        self.time = rospy.get_time()
        
        self.loop()
        
    def current_velocity_callback(self, data):
        self.current_velocity = data.twist.linear.x
    
    
    def twist_cmd_callback(self, data):
        self.velocity_cmd = data.twist.linear.x
        self.angular_velocity_cmd = data.twist.angular.z
    
    
    def dbw_enabled_callback(self, data):
        rospy.logwarn("dbw_enabled:{}".format(data))
        self.dbw_enabled = data
        
    def pose_callback(self, data):
        self.car_position[0] = data.pose.position.x
        self.car_position[1] = data.pose.position.y
        self.car_position[2] = data.pose.position.z
        
    def waypoint_callback(self, data):
        #get position of first waypoint ahead of car
        self.waypoint_position[0] = data.waypoints[0].pose.pose.position.x
        self.waypoint_position[1] = data.waypoints[0].pose.pose.position.y
        self.waypoint_position[2] = data.waypoints[0].pose.pose.position.z

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            
            new_time = rospy.get_time()
            dt = new_time - self.time
            self.time = new_time
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            
            #calculate distance between car and next waypoint            
            distance = math.sqrt( (self.waypoint_position[0] - self.car_position[0])**2 + (self.waypoint_position[1] - self.car_position[1])**2 + (self.waypoint_position[2] - self.car_position[2])**2)
            
            #calculate desired acceleration using equation vf^2 = vi^2 + 2*a*d
            if distance == 0:
                acceleration = 0.0
            else:
                acceleration = (self.velocity_cmd**2 - self.current_velocity**2)/(2*distance)
                       
            throttle, brake, steering = self.controller.control(self.velocity_cmd, self.current_velocity, acceleration, self.angular_velocity_cmd, dt, self.dbw_enabled)
            if self.dbw_enabled:
                rospy.loginfo("throttle %f brake %f (vi=%f vf=%f)", throttle, brake, self.current_velocity, self.velocity_cmd)
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
