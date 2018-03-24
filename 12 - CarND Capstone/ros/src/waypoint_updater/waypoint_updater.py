#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Member Variables
        self.waypoints = None
        self.final_waypoints = None
        self.closest_waypoint_idx = None
        self.closest_traffic_light_idx = -1
        self.red_light_in_front = False

        self.lookahead_wps = 0
        self.max_velocity = 1 #in mps
        self.decel_distance_point = 1 #Start reducing speed when car is lesser than this distance

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # Subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb)

        # Publisher(s)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        # Returns the closest waypoint to the car
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

        tmpWayPtsList = []

        if self.waypoints:
            for waypt in self.waypoints.waypoints:
                tmpWayPtsList.append(dl(waypt.pose.pose.position, msg.pose.position))
            self.closest_waypoint_idx = np.argmin(tmpWayPtsList)

            if self.closest_waypoint_idx + self.lookahead_wps + 1 > self.waypoints_size:
                list_end_idx = self.closest_waypoint_idx + self.lookahead_wps + 1 - self.waypoints_size
                self.final_waypoints = self.waypoints.waypoints[self.closest_waypoint_idx:] + self.waypoints.waypoints[:list_end_idx]
            else:
                self.final_waypoints = self.waypoints.waypoints[self.closest_waypoint_idx: self.closest_waypoint_idx + self.lookahead_wps + 1]

            self.waypoint_update()

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_size = np.shape(waypoints.waypoints)[0]
        self.lookahead_wps = min(LOOKAHEAD_WPS, self.waypoints_size//2)
        #rospy.logwarn("Total Waypoints {}".format(self.waypoints_size))
        self.max_velocity = self.get_waypoint_velocity(waypoints.waypoints[0]) #mps
        #rospy.logwarn("Maximum Velocity {}".format(self.maximum_velocity))
        self.decel_distance_point = 2.0 * self.max_velocity
        #rospy.logwarn("Decel distance point {}".format(self.decel_distance_point))

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        # Sets the closest traffic light index
        if(msg.data >= 0):
            self.closest_traffic_light_idx = msg.data
            self.red_light_in_front = True
        else:
            self.closest_traffic_light_idx = None
            self.red_light_in_front = False

    """
    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. We will implement it later
        pass
    """

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def waypoint_update(self):
        for waypt in range(self.lookahead_wps):
            velocity = self.max_velocity
            #rospy.logwarn("Red Light In Front {}".format(self.red_light_in_front))

            if self.red_light_in_front:
                waypt_distance = self.closest_traffic_light_idx - self.closest_waypoint_idx
                shouldDecel = (waypt < waypt_distance) & (waypt_distance > 1) & (waypt_distance < self.lookahead_wps)
                if shouldDecel:
                    tl_distance = self.get_distance(self.final_waypoints, waypt, waypt_distance+1)
                    if(tl_distance < self.decel_distance_point):
                        velocity = 0.0
                        #rospy.logwarn("TL ahead ... velocity Set to {}".format(velocity))

            self.set_waypoint_velocity(self.final_waypoints, waypt, velocity)
            #rospy.logwarn("Velocity {}".format(velocity))
        self.PublishLaneWaypoints()

    def PublishLaneWaypoints(self):
        ln = Lane()
        ln.header = self.waypoints.header
        ln.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(ln)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
