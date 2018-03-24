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
import numpy as np
import PyKDL

STATE_COUNT_THRESHOLD = 3
INSPECT_RADIUS = 50.0

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

        self.color_encoding = 'bgr8'

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        
    def waypoints_cb(self, waypoints):
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
            light_wp = light_wp if state in [TrafficLight.RED, TrafficLight.YELLOW] else -1
            #light_wp = light_wp if state in [TrafficLight.RED, TrafficLight.YELLOW] else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_nearest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_waypt_dist = np.Inf
        closest_waypt_idx = 0

        if self.waypoints is not None:
            waypts_list = self.waypoints.waypoints
        else:
            return None

        for waypt in range(len(waypts_list)):
            waypt_idx = waypts_list[waypt].pose.pose.position
            dist_between_waypts = math.sqrt((waypt_idx.x - pose.position.x)**2 + (waypt_idx.y - pose.position.y)**2 + (waypt_idx.z - pose.position.z)**2)
            if dist_between_waypts < closest_waypt_dist:
                closest_waypt_dist = dist_between_waypts
                closest_waypt_idx = waypt_idx

        #rospy.logwarn("Closest Waypoint Index {}".format(closest_waypt_idx))

        return closest_waypt_idx

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

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, self.color_encoding)
        #cv2.imshow("TL Image", cv_image)
        #cv2.waitKey(0)

        ##cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, self.color_encoding) #rgb8
        light_state = self.light_classifier.get_classification(cv_image)
        #Get classification
        return light_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_index = None
        car_position = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        stop_line_positions_holder = []

        #TODO find the closest visible traffic light (if one exists)
        if(self.pose):
            #rospy.logwarn("Self Pose {}".format(self.pose))
            for stop_line in stop_line_positions:
                tl = TrafficLight()
                tl.pose.pose.position.x = stop_line[0]
                tl.pose.pose.position.y = stop_line[1]
                tl.pose.pose.position.z = 0
                tl.pose.pose.orientation.x = self.pose.pose.orientation.x
                tl.pose.pose.orientation.y = self.pose.pose.orientation.y
                tl.pose.pose.orientation.z = self.pose.pose.orientation.z
                tl.pose.pose.orientation.w = self.pose.pose.orientation.w
                stop_line_positions_holder.append(tl)

            light_index  = self.get_closest_waypoint(self.pose.pose, self.lights)

        if light_index == None:
            return -1, TrafficLight.UNKNOWN

        stop_light_index = self.get_closest_waypoint(self.lights[light_index].pose.pose, stop_line_positions_holder)
        car_wrt_light_index = self.get_closest_waypoint(self.pose.pose, stop_line_positions_holder)

        #rospy.logwarn("Stop Light Index {}".format(stop_light_index))
        #rospy.logwarn("Car wrt Light Index{}".format(car_wrt_light_index))

        if car_wrt_light_index != stop_light_index:
            return -1, TrafficLight.UNKNOWN

        stop_waypt_index = self.get_closest_waypoint(stop_line_positions_holder[stop_light_index].pose.pose, self.waypoints.waypoints)

        state = self.get_light_state(self.lights[light_index])
        #rospy.logwarn("Light State {}".format(state))

        #self.waypoints = None
        return stop_waypt_index, state

    def get_closest_waypoint(self, pose, waypts):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            waypts (Waypoints)

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_distance = np.Inf
        closest_angle = None
        closest_idx = None

        if self.waypoints:
            for wp_idx, waypt in enumerate(waypts):
                distance = np.sqrt((waypt.pose.pose.position.x-pose.position.x)**2 +
                                    (waypt.pose.pose.position.y-pose.position.y)**2 +
                                    (waypt.pose.pose.position.z-pose.position.z)**2)
                if (distance < closest_distance) and (distance < INSPECT_RADIUS):
                    pose_quat = self.getQuaternion(pose)
                    waypt_vec = self.getWaypointVector(waypt, pose)
                    car_angle = self.getCarAngle(pose_quat, waypt_vec)

                    if car_angle < np.pi/2.0:
                        closest_distance = distance
                        #closest_angle = car_angle
                        closest_idx = wp_idx
                    if car_angle > np.pi/2.0:
                        closest_distance = distance
                        #closest_angle = car_angle
                        closest_idx = wp_idx

        return closest_idx#, closest_angle, closest_distance

        if not self.waypoints or not self.waypoints.waypoints:
            #rospy.logerr("Waypoints empty")
            return None

    def getQuaternion(self, pose):
        pose_orient_x = pose.orientation.x
        pose_orient_y = pose.orientation.y
        pose_orient_z = pose.orientation.z
        pose_orient_w = pose.orientation.w

        poseQuat = PyKDL.Rotation.Quaternion(pose_orient_x, pose_orient_y, pose_orient_z, pose_orient_w)
        return poseQuat

    def getWaypointVector(self, waypt, pose):
        waypt_vec = PyKDL.Vector(waypt.pose.pose.position.x - pose.position.x, waypt.pose.pose.position.y - pose.position.y, 0.0)
        return waypt_vec

    def getCarAngle(self, pose_quat, waypt_vec):
        '''
        roll = pitch = yaw = 0

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.z]
        (roll, pitch, yaw) = euler_from_quarternion(orientation_list)
        rospy.logwarn("Yaw Angle {}".format(yaw))
        return yaw
        '''
        '''
        my_pose = self.pose
        my_pose_x = my_pose.pose.position.x
        my_pose_y = my_pose.pose.position.y
        my_pose_z = my_pose.pose.position.z

        my_pose_orient_x = my_pose.pose.orientation.x
        my_pose_orient_x = my_pose.pose.orientation.y
        my_pose_orient_z = my_pose.pose.orientation.z
        my_pose_orient_w = my_pose.pose.orientation.w

        euler = tf.transformations.euler_from_quaternion([my_pose_orient_x,my_pose_orient_x, my_pose_orient_z, my_pose_orient_w])
        angle = euler[2]
        return angle
        '''
        #Credit: Mr. Eki PyKDL implementation
        car_orient = pose_quat * PyKDL.Vector(1.0,0.0,0.0)
        cos_thetha = PyKDL.dot(car_orient, waypt_vec)/car_orient.Norm()/waypt_vec.Norm()
        car_thetha = np.arccos(cos_thetha)
        return car_thetha

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
