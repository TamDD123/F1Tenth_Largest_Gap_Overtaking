#!/usr/bin/env python
 
# Import necessary libraries
import rospy	# The ROS Python library
from ackermann_msgs.msg import AckermannDriveStamped	# Ackermann steering control messages
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry	# Messages for robot pose
from sensor_msgs.msg import LaserScan	# Messages for LIDAR data
from visualization_msgs.msg import Marker	# Messages for visualizing points in RViz
from std_msgs.msg import Bool
from datetime import datetime
import numpy as np
import copy
 
 
 
import csv
import math

# Define constants
LOOKAHEAD_DISTANCE = 1.20	# Distance to look ahead for path planning
KP = 1.00	# Proportional control constant
PI = 3.1415927	# Pi constant
GAP_THRESHOLD = 2.0	# The minimu size of a gap for overtaking
SAFETY_RADIUS = 0.30	#	Safety distance from obstacles
 
class SubscribeAndPublish:
    def __init__(self):
        rospy.init_node("pure_pursuit")
	# Initialize publishers and subscribers
	# Publishers to control the car and visualize data
        self.pub_1 = rospy.Publisher("/pp", AckermannDriveStamped, queue_size=1000)
        self.pub_2 = rospy.Publisher("/env_viz", Marker, queue_size=1000)
        self.pub_3 = rospy.Publisher("/dynamic_viz", Marker, queue_size=1000)
	# Subscribers to get odometry and LIDAR data
        self.sub_1 = rospy.Subscriber("/odom_blue", Odometry, self.callback)
        self.sub_2 = rospy.Subscriber("/blue/scan", LaserScan, self.lidar_callback)
	# Initialization of various variables
        self.xes = []
        self.yes = []
        self.current_indx = 0
        self.flag = False
        self.logflag = False
        self.scanrows = []
        self.idx = 0
        self.holder = []
        self.detectflag = True
        self.indexes = []
        self.overtaking = False
	self.target_angle = 0.0
	self.distance = 100.0
	self.largest_gap = 0
	self.o = 0
	self.n = 0
	self.overtaking_cooldown = 0
	self.overtakes = 0
	self.goal_idx = None
	self.goal_flag = True
	self.lap = 0
	self.lap_flag = True
 
        # Read in all the data
        with open("/home/rasmus/catkin_ws/src/f1tenth_simulator_two_agents/node/data_ims.csv") as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                x, y = map(float, row)
                self.xes.append(x)
                self.yes.append(y)
 
 
        self.makefile = False #change if you want to log data
        if (self.makefile == True):
 
            self.temp_filename = datetime.now()
            self.filename = self.temp_filename.strftime("%d-%m-%Y %H:%M:%S")
            self.file_path = "/home/rasmus/catkin_ws/src/f1tenth_simulator_two_agents/media/" + self.filename + ".csv"
            self.csv_file = None
            self.writer = None
            self.previous_ranges = []
            self.previous_intensities = []
            self.csv_file = open(self.file_path, 'w')
            self.writer = csv.writer(self.csv_file) 
 
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
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
 
        self.logx = 0.
        self.logy = 0.
        self.prev_scan = []
        self.prev_odom = []
 
    # Filters and returns a segment of LIDAR data
    def filter_lidar(self, data):
        ranges = []
        gap = 109
        for i in range(gap):
            ranges.append(data.ranges[(len(data.ranges)/2)-54+i])
        return ranges

    # Finds and returns the largest and the angle at the center of the largest gap in the LIDAR data
    def find_largest_gap(self, data):
        ranges = np.array(data.ranges)

	# Create an array indicating whether each point is in a gap.
	in_gap = ranges > GAP_THRESHOLD
	gap_changes = np.diff(in_gap.astype(int))

	# Find indices of the gap starts/ends.
	gap_starts = np.where(gap_changes > 0)[0] + 1
	gap_ends = np.where(gap_changes < 0)[0]

	# Make sure we have the same number of gap starts and ends.
	if gap_starts[0] > gap_ends[0]:
	    gap_ends = gap_ends[1:]
	if gap_starts[-1] > gap_ends[-1]:
	    gap_starts = gap_starts[:-1]

	# Find the largest gap.
	gaps = gap_ends - gap_starts
	largest_gap_index = np.argmax(gaps)
	largest_gap_size = gaps[largest_gap_index]
	largest_gap_start = gap_starts[largest_gap_index]
	largest_gap_end = gap_ends[largest_gap_index]

	# Compute the angle at the center of the gap.
	largest_gap_center = (largest_gap_start + largest_gap_end) / 2
	largest_gap_angle = data.angle_min + data.angle_increment * largest_gap_center

	return largest_gap_size, largest_gap_angle
 
    # Logs robot's position when it has moved a certain distance
    def logging(self, pos):
        distance = abs(math.sqrt((pos.pose.pose.position.x-self.logx)**2+(pos.pose.pose.position.y-self.logy)**2))
        if distance >= 0.5:
            self.logx = pos.pose.pose.position.x
            self.logy = pos.pose.pose.position.y
            self.logflag = True
            self.prev_odom = [self.logx, self.logy]
        else:
            self.logflag = False
        return self.logflag, self.logx, self.logy
 
    # Combines LIDAR and odometry data for logging
    def combine_data(self):
        if self.logflag == True:
            result = []
            result = self.prev_scan
            for i in range(len(self.prev_odom)):
                result.append(self.prev_odom[i])
            self.writer.writerow(result)

    # Checks if an object is within the safety radius
    def is_object_in_safety_radius(self, ranges):
        for range_i in ranges:
            if range_i < SAFETY_RADIUS:
                return True
        return False

    # Callback function for LIDAR data
    # It detects gaps for overtaking and sets flags accordingly
    def lidar_callback(self, data):
	# If we're not currently overtaking, find the largest gap
        largest_gap, gap_angle = self.find_largest_gap(data)

	self.largest_gap = largest_gap

        if not self.overtaking:
            # Now we also check for the safety radius
            if largest_gap > (GAP_THRESHOLD + 2*SAFETY_RADIUS) and largest_gap < 170: # Change the last number depending on map - Big maps = 170 - Small maps = 120
                self.overtaking = True
		self.overtaking_cooldown = 5000
	elif self.overtaking_cooldown == 0:
	    self.overtaking = False
	if self.overtaking == True:
	    if self.is_object_in_safety_radius(data.ranges):
		self.target_angle = 0
	    else:
		self.target_angle = gap_angle
	    self.overtaking_cooldown -= 1
	if largest_gap < (GAP_THRESHOLD + 88.0):
	    self.overtaking = False

        self.prev_scan = self.filter_lidar(data)

    # Callback function for odometry data
    # Implements the pure pursuit path following algorithm
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
 
	if self.goal_flag == True:
	    self.goal_idx = self.current_indx - 1
	    self.goal_flag = False 

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
        marker_2.color.r = 0.0
        marker_2.color.g = 0.0
        marker_2.color.b = 1.0
 
        self.reactive_control(angle)
        self.pub_2.publish(self.marker)
        self.pub_3.publish(marker_2)
        self.logging(odometry_info)
        if (self.makefile == True):
            self.combine_data()

	if self.current_indx == self.goal_idx:
	    if self.lap == 10:
		self.sub_2.unregister()
		print("Finished")
	    elif self.lap_flag:
		self.lap += 1
		self.lap_flag = False
		print("Lap ", self.lap)
	if self.current_indx == (self.goal_idx + 1):
	    self.lap_flag = True
 
    # Sends steering and speed controls to the car based on computed angle
    def reactive_control(self, angle):
        ackermann_drive_result = AckermannDriveStamped()
        if self.overtaking and self.largest_gap > GAP_THRESHOLD + 2*SAFETY_RADIUS:
	    if self.o == 0:
	        print("Overtaking")
		self.overtakes += 1
		print(self.overtakes)
		self.o = 1
		self.n = 0
            ackermann_drive_result.drive.steering_angle = self.target_angle
        else:
	    if self.n == 0:
		print("No-overtaking")
		self.n = 1
		self.o = 0
            ackermann_drive_result.drive.steering_angle = angle
 
	if self.largest_gap < GAP_THRESHOLD + 88:
	    ackermann_drive_result.drive.speed = 0.05 
	elif abs(angle) > 45.0 / 180.0 * PI:
	    ackermann_drive_result.drive.speed = 0.5
        elif abs(angle) > 20.0 / 180.0 * PI:
            ackermann_drive_result.drive.speed = 1.0
        elif abs(angle) > 10.0 / 180.0 * PI:
            ackermann_drive_result.drive.speed = 2.0
        else:
            ackermann_drive_result.drive.speed = 3.0
 
        self.pub_1.publish(ackermann_drive_result)
 
if __name__ == "__main__":
    SAPObject = SubscribeAndPublish()
    rospy.spin()
