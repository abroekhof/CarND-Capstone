#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
import os
import csv

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.stop_lines = {}

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # reset the images index
        if os.path.isfile("light_classification/images.csv"):
            os.remove("light_classification/images.csv")

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        self.stop_lines = {}
        for stop_line in stop_line_positions:
            stop_line_pose = Pose()
            stop_line_pose.position.x = stop_line[0]
            stop_line_pose.position.y = stop_line[1]
            line_position = self.get_closest_waypoint(stop_line_pose)
            self.stop_lines[line_position] = stop_line_pose

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def distance_to_waypoint(self, pose1, pose2):
        veh_x = pose1.x
        veh_y = pose1.y
        wp_x = pose2.x
        wp_y = pose2.y
        dx = veh_x - wp_x
        dy = veh_y - wp_y
        return math.sqrt(dx * dx + dy * dy)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        nearest = None
        min_dist = float("inf")
        for index, waypoint in enumerate(self.waypoints.waypoints):
            dist = self.distance_to_waypoint(pose.position, waypoint.pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                nearest = index

        return nearest


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']
        cx = image_width / 2
        cy = image_height / 2

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/world", "/base_link", now, rospy.Duration(1.0))
            tl_point = PointStamped()
            tl_point.header.frame_id = "/world"
            tl_point.header.stamp = now
            tl_point.point.x = point_in_world.x
            tl_point.point.y = point_in_world.y
            tl_point.point.z = point_in_world.z

            car_point = self.listener.transformPoint("/base_link", tl_point)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
            return None

        camera_tilt = - 9. / 180. * math.pi
        if fx < 10 or fy < 10:
            fx = 2250
            fy = 2250

        # Car coordinate
        car_x = car_point.point.x
        car_y = car_point.point.y
        car_z = car_point.point.z

        # Camera coordinate (note camera is tilted slightly upward)
        cam_x = math.cos(camera_tilt) * car_x - math.sin(camera_tilt) * car_z
        cam_y = car_y
        cam_z = math.sin(camera_tilt) * car_x + math.cos(camera_tilt) * car_z

        # Image sensor coordinate
        img_x = -cam_y
        img_y = -cam_z
        img_z = cam_x

        # Pixel coordinate
        pix_x = int(fx * img_x / img_z + cx - 40)
        pix_y = int(fy * img_y / img_z + cy)

        # Calculate the bounding box size to encapsulate the traffic light
        traffic_light_size = 4
        bb_width = int(math.sqrt(fx*fy) * traffic_light_size / img_z)

        return pix_x, pix_y, bb_width

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO(Olala): return the real state
        # Fake light detection with ground truth
        if not light:
            return TrafficLight.UNKNOWN

        if light.state != TrafficLight.UNKNOWN:
            xy = self.project_to_image_plane(light.pose.pose.position)
            if xy is not None:
                x, y = xy
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                image_width = self.config['camera_info']['image_width']
                image_height = self.config['camera_info']['image_height']
                crop_size = 299
                half_crop = int(crop_size/2)
                if x > image_width - half_crop:
                    x_min = image_width - crop_size
                    x_max = image_width
                elif x < half_crop:
                    x_min = 0
                    x_max = crop_size
                else:
                    x_min = max(0, x-half_crop)
                    x_max = min(x_min+crop_size, image_width)

                if y > image_height - half_crop:
                    y_min = image_height - crop_size
                    y_max = image_height
                elif y < half_crop:
                    y_min = 0
                    y_max = crop_size
                else:
                    y_min = max(0, y-half_crop)
                    y_max = min(y_min+crop_size, image_height)
                cropped = cv_image[y_min:y_max, x_min:x_max]
                # cv2.rectangle(cv_image, (x-75, y-75), (x+75, y+75), (255, 0 , 0), 2)
                # cv2.putText(cv_image, 'x: %s, y %s' % (x, y), (100, 560), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)

                filename = os.path.abspath("light_classification/training_data/{}-{}.jpg".format(light.state, rospy.Time.now()))
                with open("light_classification/images.csv", "a") as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([filename, light.state])
                cv2.imwrite(filename, cropped)
                rospy.loginfo("light image loc: {}, {}".format(x, y))
            return light.state

        # End of fake light detection

        if(not self.has_image):
            self.prev_light_loc = None
            return False


        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.waypoints or not self.pose:
            return -1, TrafficLight.UNKNOWN

        light = None

         # car_fwd indicates whether car's moving the same direction as the waypoint index increase
        car_fwd = False
        closest_wp_index = None
        waypoints = self.waypoints.waypoints
        wp_length = len(waypoints)
        if(self.pose):
            car_orientation = self.pose.pose.orientation
            quaternion = (car_orientation.x, car_orientation.y,
                          car_orientation.z, car_orientation.w)
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

            car_pose = self.pose.pose
            closest_wp_index = self.get_closest_waypoint(car_pose)
            wp1 = waypoints[closest_wp_index]
            wp2 = waypoints[(closest_wp_index + 1) % wp_length]
            fwd_angle = math.atan2(wp2.pose.pose.position.y - wp1.pose.pose.position.y,
                                   wp2.pose.pose.position.x - wp1.pose.pose.position.x)

            if -math.pi / 2.0 < yaw - fwd_angle < math.pi / 2:
                car_fwd = True

        light, light_wp = None, None

        for i in range(wp_length):
            inc = 1 if car_fwd else -1
            index = (closest_wp_index + i * inc) % wp_length
            if index in self.stop_lines:
                light_wp = index
                stop_line_pose = self.stop_lines[index]
                min_dist = float("inf")
                # Find associated light
                for light_candidate in self.lights:
                    dist = self.distance_to_waypoint(light_candidate.pose.pose.position, stop_line_pose.position)
                    if dist < min_dist:
                        light = light_candidate
                        min_dist = dist
                break

        if light:
            state = TrafficLight.UNKNOWN
            if abs(light_wp - closest_wp_index) < 200:
                state = self.get_light_state(light)
                rospy.loginfo('light color ahead %s' % state)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
