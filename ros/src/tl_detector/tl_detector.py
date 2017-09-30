#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import tf.transformations
import angles

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

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

        # copied from sim_traffic_light_config.yaml for testing only
        self.intersection_coords = [[1148.56, 1184.65], [1559.2, 1158.43], [2122.14, 1526.79],
                                    [2175.237, 1795.71], [1493.29, 2947.67], [821.96, 2905.8],
                                    [161.76, 2303.82], [351.84, 1574.65]]
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        rospy.loginfo("tl got waypoints None = %s", waypoints == None)
        self.waypoints = waypoints

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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        if pose == None or self.waypoints == None:
            return -1
        
        # find the waypoint index
        q = (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(q)
                
        idx = -1
        min_dist = 1e9
        for i in range(len(self.waypoints.waypoints)):
            pose_x = pose.position.x
            pose_y = pose.position.y
            wp_x = self.waypoints.waypoints[i].pose.pose.position.x
            wp_y = self.waypoints.waypoints[i].pose.pose.position.y
            d = math.sqrt((pose_x - wp_x)**2 + (pose_y - wp_y)**2)
            if d < min_dist:
                # make sure the waypoint is in the direction of vehicle yaw
                theta = math.atan2(wp_y - pose_y, wp_x - pose_x)
                if abs(angles.shortest_angular_distance(theta, yaw)) < math.pi/4:
                    idx = i
                    min_dist = d
        return idx


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

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

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
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        if self.pose == None or self.waypoints == None:
            rospy.loginfo("pose or waypoints is None")
        else:
            for coord in self.intersection_coords:
                d = math.sqrt((coord[0] - self.pose.pose.position.x)**2 + (coord[1] - self.pose.pose.position.y)**2)
                theta = math.atan2((coord[1] - self.pose.pose.position.y), (coord[0] - self.pose.pose.position.x))
                q = (self.pose.pose.orientation.x, self.pose.pose.orientation.y,
                     self.pose.pose.orientation.z, self.pose.pose.orientation.w)
                _, _, yaw = tf.transformations.euler_from_quaternion(q)
                if d < 50.0 and abs(angles.shortest_angular_distance(theta, yaw)) < math.pi/4:
                    rospy.loginfo("pose.x %f pose.y %f approaching intersection .x %f .y %f distance %f",
                                  self.pose.pose.position.x, self.pose.pose.position.y,
                                  coord[0], coord[1], d)

                    # find the waypoint closest to the signal and with index after car
                    best_wp = -1
                    min_dist = 1e9
                    i = 0
                    decreasing = True
                    while decreasing:
                        idx = (car_position + i) % len(self.waypoints.waypoints)
                        wp_x = self.waypoints.waypoints[idx].pose.pose.position.x
                        wp_y = self.waypoints.waypoints[idx].pose.pose.position.y
                        d = math.sqrt((coord[0] - wp_x)**2 + (coord[1] - wp_y)**2)
                        if d < min_dist:
                            best_wp = idx
                            min_dist = d
                        else:
                            decreasing = False
                        i = i + 1
                        
                    if best_wp >= 0:
                        #rospy.loginfo("closest waypoint idx %d distance %f i=%d", best_wp, min_dist, i)
                        light = self.waypoints.waypoints[best_wp]
                        light_wp = best_wp


        if light:
            state = self.get_light_state(light)

            state = TrafficLight.RED
            
            return light_wp, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
