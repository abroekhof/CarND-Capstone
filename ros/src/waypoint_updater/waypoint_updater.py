#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf.transformations
import angles

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

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.pose = None
        self.base_lane = None
        self.traffic_wp = -1
        self.obstacle_wp = None
        self.seqnum = 0
        self.car_wp_q = -1
        self.plan_waypoints = []
        self.plan_wp_start = 0
        self.last_traffic_wp_planned = -1

        rospy.spin()

    def pose_cb(self, msg):
        #rospy.loginfo("pose_cb timestamp %s x=%d y=%d z=%d", msg.header.stamp, msg.pose.position.x,
        #              msg.pose.position.y, msg.pose.position.z)
        self.pose = msg.pose
        if self.base_lane != None:
            quaternion = (msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
            veh_yaw = yaw % (2.0 * math.pi)
            veh_x = msg.pose.position.x
            veh_y = msg.pose.position.y
            
            # the course hits an inflection point at x=2339 yaw=90+ at which point x starts to decrease
            # next inflection at x=155 yaw=270+ at which point x starts to increase
            # in front of the car = increasing x for yaw between 270 and 90 and decreasing x from 90 to 270

            #veh_fwd = True
            #if veh_yaw > (math.pi/2) and veh_yaw <= (3*math.pi/2):  # 90 to 270 degrees
            #    veh_fwd = False

            pub_waypoints = []
            wp_start = -1
            min_dist = 1e9
            dist_q = 1e9
            state = 0  # no progress finding waypoint
            if len(self.base_lane.waypoints) > 1:
                # find the first waypoint in front of the vehicle,
                #   then take a sequence of LOOKAHEAD_WPS waypoints
                for i in range(len(self.base_lane.waypoints)):
                    wp_idx = i
                    if self.car_wp_q >= 0:
                        wp_idx = (self.car_wp_q + i) % len(self.base_lane.waypoints)                    
                    wp = self.base_lane.waypoints[wp_idx]
                    wp_x = wp.pose.pose.position.x
                    wp_y = wp.pose.pose.position.y
                    veh_to_wp_dist = math.sqrt((wp_x - veh_x)**2 + (wp_y - veh_y)**2)
                    if veh_to_wp_dist < min_dist:
                        theta = math.atan2(wp_y - veh_y, wp_x - veh_x)
                        # make sure the waypoint is in front of the car
                        if abs(angles.shortest_angular_distance(theta, veh_yaw)) < math.pi/4.0:
                            state = 1  # found correctly oriented waypoint
                            min_dist = veh_to_wp_dist
                            wp_start = wp_idx
                            # we should be able to stop iterating because the next waypoint should be in front
                            #   of the previous waypoint...for now, starting where we left off should at least
                            #   eliminate a lot of atan2 calls
                    if veh_to_wp_dist > dist_q and state == 1:
                        state = 2  # wp getting farther away from car
                        break;
                    dist_q = veh_to_wp_dist

            if wp_start >= 0:
                self.car_wp_q = wp_start
                rospy.loginfo("Car waypoint is %d state %d plan_wp_start %d wp_start %d",
                              self.car_wp_q, state, self.plan_wp_start, wp_start)

                if len(self.plan_waypoints) > 0:
                    # update the plan with new waypoints
                    #  - remove passed waypoints
                    remove_cnt = (wp_start - self.plan_wp_start) % len(self.base_lane.waypoints)
                    for i in range(remove_cnt):
                        self.plan_waypoints.pop(0)
                    self.plan_wp_start = wp_start
                    wp_idx = wp_start + len(self.plan_waypoints) - 1
                    add_cnt = LOOKAHEAD_WPS - len(self.plan_waypoints)
                    for j in range(add_cnt):
                        idx = (wp_idx + j) % len(self.base_lane.waypoints)
                        wp = self.base_lane.waypoints[idx]
                        self.plan_waypoints.append(wp)
                else:
                    self.plan_wp_start = wp_start
                    for wp_idx in range(LOOKAHEAD_WPS):
                        idx = (wp_start + wp_idx) % len(self.base_lane.waypoints)
                        wp = self.base_lane.waypoints[idx]
                        self.plan_waypoints.append(wp)

                wp_last = wp_start + len(self.plan_waypoints) - 1
                
                # Add linear velocity to waypoints
                for i in range(len(self.plan_waypoints)):
                    self.set_waypoint_velocity(self.plan_waypoints, i, 20.0)

                # when the car enters scope of a traffic light, plan the velocity to decelerate
                #   only do this once per light unless the light state changes
                if self.traffic_wp >= 0 and self.traffic_wp != self.last_traffic_wp_planned:
                    rospy.loginfo("car is at wp %d car should stop at wp %d", self.car_wp_q, self.traffic_wp)
                    self.last_traffic_wp_planned = self.traffic_wp
                    
                l = Lane()
                l.header.seq = self.seqnum
                self.seqnum = self.seqnum + 1
                l.header.stamp = rospy.get_rostime()
                l.waypoints = self.plan_waypoints
                self.final_waypoints_pub.publish(l)
                            
            rospy.loginfo("wp_start %d min_dist %f wp.x %f pose.x %f veh_yaw %f",
                          wp_start, min_dist, self.base_lane.waypoints[wp_start].pose.pose.position.x,
                          msg.pose.position.x, veh_yaw)

        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_lane = waypoints
        pass

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        rospy.loginfo("traffic_cb wp = %d", msg.data)
        self.traffic_wp = msg.data
        if self.pose and self.traffic_wp >= 0:
            pose_x = self.pose.position.x
            pose_y = self.pose.position.y
            tl_x = self.base_lane.waypoints[self.traffic_wp].pose.pose.position.x
            tl_y = self.base_lane.waypoints[self.traffic_wp].pose.pose.position.y
            d = math.sqrt((pose_x - tl_x)**2 + (pose_y - tl_y)**2)
            rospy.loginfo("traffic control at waypoint %d distance %f", self.traffic_wp, d)
        pass

    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_wp = msg.data
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
