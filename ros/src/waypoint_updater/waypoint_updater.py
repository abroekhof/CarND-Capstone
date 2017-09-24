#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf
import angles

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.all_waypoints = []

        rospy.spin()

    def pose_cb(self, msg):
        calc_distance = lambda wp: math.sqrt(
            (wp.pose.pose.position.x-msg.pose.position.x)**2
            + (wp.pose.pose.position.y-msg.pose.position.y)**2
            + (wp.pose.pose.position.z-msg.pose.position.z)**2)

        # TODO(abroekhof): verify that this produces angles in the correct
        # reference frame.
        calc_angle = lambda wp: math.atan2(
            wp.pose.pose.position.y-msg.pose.position.y,
            wp.pose.pose.position.x-msg.pose.position.x)

        qtrn = msg.pose.orientation
        (_, _, car_yaw) = tf.transformations.euler_from_quaternion([qtrn.x, qtrn.y, qtrn.z, qtrn.w])

        min_dist = float("inf")
        start_idx = 0
        for idx, waypoint in enumerate(self.all_waypoints):
            wp_angle = calc_angle(waypoint)
            # Make sure that the car is generally pointed towards the waypoint.
            if abs(angles.shortest_angular_distance(car_yaw, wp_angle)) < math.pi/4:
                curr_dist = calc_distance(waypoint)
                if curr_dist < min_dist:
                    start_idx = idx
                    min_dist = curr_dist

        waypoints = []
        all_waypoints_len = len(self.all_waypoints)
        # TODO(abroekhof): Does final_waypoints need to wrap around, or does 
        # the car stop at the end?
        while len(waypoints) < LOOKAHEAD_WPS and start_idx < all_waypoints_len:
            waypoints.append(self.all_waypoints[start_idx])
            start_idx += 1

        lane = Lane()
        lane.header.frame_id = '/car'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, lane):
        self.all_waypoints = []
        for waypoint in lane.waypoints:
            self.all_waypoints.append(waypoint)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
