#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

import csv
import math

LOOKAHEAD_DISTANCE = 1.20
KP = 1.00

PI = 3.1415927

class SubscribeAndPublish:
    def __init__(self):
        rospy.init_node("pure_pursuit2")
        self.pub_1 = rospy.Publisher("/pp2", AckermannDriveStamped, queue_size=1000)
        self.pub_2 = rospy.Publisher("/env_viz2", Marker, queue_size=1000)
        self.pub_3 = rospy.Publisher("/dynamic_viz2", Marker, queue_size=1000)
        self.sub_ = rospy.Subscriber("/odom_red", Odometry, self.callback)

        self.xes = []
        self.yes = []
        self.headings = []
        self.current_indx = 0
        self.flag = False

        # Read in all the data
        with open("/home/rasmus/catkin_ws/src/f1tenth_simulator_two_agents/node/data_ims.csv") as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                x, y = map(float, row)
                self.xes.append(x)
                self.yes.append(y)
                #self.headings.append(heading)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time()
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.color.a = 1.0  # Don't forget to set the alpha!
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0

    def callback(self, odometry_info):
        x_current = odometry_info.pose.pose.position.x
        y_current = odometry_info.pose.pose.position.y
        siny_cosp = 2.0 * (odometry_info.pose.pose.orientation.w * odometry_info.pose.pose.orientation.z + odometry_info.pose.pose.orientation.x * odometry_info.pose.pose.orientation.y)
        cosy_cosp = 1.0 - 2.0 * (odometry_info.pose.pose.orientation.y * odometry_info.pose.pose.orientation.y + odometry_info.pose.pose.orientation.z * odometry_info.pose.pose.orientation.z)
        heading_current = math.atan2(siny_cosp, cosy_cosp)

        if not self.flag:
            shortest_distance = 100.0
            for i in range(len(self.xes)):
                if ((self.xes[i] - x_current) ** 2 + (self.yes[i] - y_current) ** 2) < shortest_distance:
                    shortest_distance = (self.xes[i] - x_current) ** 2 + (self.yes[i] - y_current) ** 2
                    self.current_indx = i
            self.flag = True

        while math.sqrt((self.xes[self.current_indx] - x_current) ** 2 + (self.yes[self.current_indx] - y_current) ** 2) < LOOKAHEAD_DISTANCE:
            self.current_indx += 1
            if self.current_indx > len(self.xes) - 1:
                self.current_indx = 0

        real_distance = math.sqrt((self.xes[self.current_indx] - x_current) ** 2 + (self.yes[self.current_indx] - y_current) ** 2)
        lookahead_angle = math.atan2(self.yes[self.current_indx] - y_current, self.xes[self.current_indx] - x_current)
        del_y = real_distance * math.sin(lookahead_angle - heading_current)
        angle = KP * 2.0 * del_y / (real_distance * real_distance)

        points = Point()
        marker_2 = Marker()
        points.x = self.xes[self.current_indx]
        points.y = self.yes[self.current_indx]
        points.z = 0.0
        marker_2.points.append(points)
        marker_2.header.frame_id = "map"
        marker_2.header.stamp = rospy.Time()
        marker_2.id = 0
        marker_2.type = Marker.POINTS
        marker_2.action = Marker.ADD
        marker_2.pose.position.x = 0.0
        marker_2.pose.position.y = 0.0
        marker_2.pose.position.z = 0.0
        marker_2.pose.orientation.x = 0.0
        marker_2.pose.orientation.y = 0.0
        marker_2.pose.orientation.z = 0.0
        marker_2.pose.orientation.w = 1.0
        marker_2.scale.x = 0.2
        marker_2.scale.y = 0.2
        marker_2.color.a = 1.0  # Don't forget to set the alpha!
        marker_2.color.r = 1.0
        marker_2.color.g = 0.0
        marker_2.color.b = 0.0

        self.reactive_control(angle)
        self.pub_2.publish(self.marker)
        self.pub_3.publish(marker_2)

    def reactive_control(self, angle):
        ackermann_drive_result = AckermannDriveStamped()
        ackermann_drive_result.drive.steering_angle = angle

        if abs(angle) > 20.0 / 180.0 * PI:
            ackermann_drive_result.drive.speed = 1
        elif abs(angle) > 10.0 / 180.0 * PI:
            ackermann_drive_result.drive.speed = 1
        else:
            ackermann_drive_result.drive.speed = 1

        self.pub_1.publish(ackermann_drive_result)

if __name__ == "__main__":
    SAPObject = SubscribeAndPublish()
    rospy.spin()

