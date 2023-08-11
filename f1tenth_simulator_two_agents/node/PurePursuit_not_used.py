
import copy
 
 
 
import csv
import math
 
LOOKAHEAD_DISTANCE = 1.20
KP = 1.00
PI = 3.1415927
GAP_THRESHOLD = 2.0
SAFETY_RADIUS = 0.40
 
class SubscribeAndPublish:
    def __init__(self):
        rospy.init_node("pure_pursuit")
        self.pub_1 = rospy.Publisher("/pp", AckermannDriveStamped, queue_size=1000)
        self.pub_2 = rospy.Publisher("/env_viz", Marker, queue_size=1000)
        self.pub_3 = rospy.Publisher("/dynamic_viz", Marker, queue_size=1000)
        self.sub_1 = rospy.Subscriber("/odom_blue", Odometry, self.callback)
        self.sub_2 = rospy.Subscriber("/blue/scan", LaserScan, self.lidar_callback)
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
        with open("/home/rasmus/catkin_ws/src/f1tenth_simulator_two_agents/node/data_silverstone.csv") as csvfile:
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
 
    def sliding_window(self, n, x):
	sum_ = 0
	for i in range(n):
	    sum_ += np.mean(self.scanrows[i+x])
	result = sum_/n
	return result
 
    def detection(self, ranges):
	#Initialize empty list for storing indexes of beams
	self.indexes = []
	#Makes sure we stay within indexes of data
	if (self.detectflag == True and self.idx <= len(self.scanrows[self.idx])-(self.n-1)):
	    #Checks difference between scan and ground truth data window means
	    if (abs(np.mean(ranges) - self.sliding_window(self.n, self.idx)) > 0.11):
		#Checks difference between scan and ground truth data for each beam
		for i in range(len(self.scanrows[self.idx])):
		    if (abs(ranges[i] - self.scanrows[self.idx][i]) > 1.0 and i not in self.indexes):
			self.indexes.append(i)
		#Checks what beams are closes to middle beam, and returns that beam lenght
		if bool(self.indexes):
		    for i in range((len(self.indexes)-1)/2):
			if (len(ranges)-1)/2 in self.indexes:
			    self.follow_distance = ranges[(len(ranges)-1)/2]
			    return self.follow_distance
			elif (len(ranges)-1)/2+i in self.indexes:
			    self.follow_distance = ranges[(len(ranges)-1)/2+i]
			    return self.follow_distance
			elif (len(ranges)-1)/2-i in self.indexes:
			    self.follow_distance = ranges[(len(ranges)-1)/2-i]
			    return self.follow_distance
	    self.idx += 1
 
    def cluster_detect(self, ranges):
	cluster_idx = 2
	cluster_vals = []
	car = []
	indexes = []
	data = []
	for i in range(len(ranges)):
	    #First beam
	    if (i == 0):
		cluster_vals.append(cluster_idx)
	    #Last beam
	    elif (i == len(ranges)-1):
		cluster_vals.append(cluster_idx)
	    #Compare beam with neighbour beams
	    elif (abs(ranges[i-1]-ranges[i]) <= abs(ranges[i]-ranges[i+1]) and abs(ranges[i]-ranges[i+1]) > 0.3):
		cluster_idx -= 1
		cluster_vals.append(cluster_idx)
	    else:
		cluster_vals.append(cluster_idx)
	#An attempt to recognize all beams in with cluster_vals = 1 as a car
	for i in range(len(cluster_vals)):
	    if (cluster_vals[i] == 1):
		indexes.append(i)
		car.append(ranges[i])
	#Finding the middle point and angle of/to the car
	middle = (indexes[0]+indexes[len(indexes)-1])/2
	angle = middle * 0.004712
	#If amount of beams hitting the car are equal
	if (self.is_even(middle)):
	    data.append(ranges[middle])
	    data.append(ranges[middle+1])
	    distance = np.mean(data)
	#If amount of beams hitting the car aren't equal
	else:
	    middle = middle+0.5
	    middle = int(middle)
	    distance = ranges[middle]
	return angle, distance
 
    #An attempt at calculating the velocity to follow the car in front
    def dynamic_vel(self, distance):
	#Current timestamp
	timer = datetime.now()
	self.timestamp = timer
	#If first timestamp, set as previous timestamp and do nothing
	if (self.prev_timestamp == 0.):
	    self.prev_timestamp = self.timestamp
	    self.prev_distance = distance
	    return 0.
	#Calculate time difference
	delta_time = self.timestamp - self.prev_timestamp
	#Make sure the scan has updated
	if(delta_time.total_seconds() >= self.scan_time):
	    #Calculate differnce in distance and velocity
	    self.prev_timestamp = self.timestamp
	    delta_distance = abs(distance - self.prev_distance)
	    self.prev_distance = distance
	    velocity = delta_distance/delta_time.total_seconds()
	return velocity
 
    #Simple method for checking if an integer is even
    def is_even(self, num):
	return num % 2 == 0
 
    def filter_lidar(self, data):
        ranges = []
#	gap = 109
        gap = 361
        for i in range(gap):
            ranges.append(data.ranges[(len(data.ranges)/2)-54+i])
        return ranges
 
 
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
 
 
    def combine_data(self):
        if self.logflag == True:
            result = []
            result = self.prev_scan
            for i in range(len(self.prev_odom)):
                result.append(self.prev_odom[i])
            self.writer.writerow(result)
 
    def is_object_in_safety_radius(self, ranges):
        for range_i in ranges:
            if range_i < SAFETY_RADIUS:
                return True
        return False
 
    def lidar_callback(self, data):
	# If we're not currently overtaking, find the largest gap
        largest_gap, gap_angle = self.find_largest_gap(data)
 
	self.largest_gap = largest_gap
 
        if not self.overtaking:
            # Now we also check for the safety radius
            if largest_gap > (GAP_THRESHOLD + 2*SAFETY_RADIUS) and largest_gap < 170: # Change the last number depending on map - Big maps = 170 - Small maps = 120
                self.overtaking = True
		self.overtaking_cooldown = 2000
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
	    if self.lap == 50:
		self.sub_2.unregister()
		print("Finished")
	    elif self.lap_flag:
		self.lap += 1
		self.lap_flag = False
		print("Lap ", self.lap)
	if self.current_indx == (self.goal_idx + 1):
	    self.lap_flag = True
 
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
