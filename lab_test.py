#!/usr/bin/env python

""" lab_test.py

Command a warehousing robot to move autonomously along a route with a number of goal locations defined inside a pre-made map. The robot attempts to navigate to each location and it  keeps track of it's success rate, time elapsed, and total distance traveled.
      
"""

import rospy
import actionlib
import pandas as pd
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pow, sqrt, pi, radians
import tf
import PyKDL

# Assign spreadsheet file with the coordinates of the locations inside the map
file = '/home/juan/Documents/Archivos_para_pedidos/coordenadaspuntos.xlsx'

# Load spreadsheet
xl = pd.ExcelFile(file)


# Load the sheet into a DataFrame by name: df

df = pd.read_excel(xl,  'Coordenadas puntos')

#position=[df.ix[point, 'x'], df.ix[point, 'y']]

# Assign spreadsheet file with the route indications for the robot
route = '/home/juan/Documents/Archivos_para_pedidos/Ruta1.xlsx'

# Load the route spreadsheet
xlroute = pd.ExcelFile(route)

dfroute = pd.read_excel(xlroute,  'Ruta1')


# Save the positions from the Route that the robot must visit in a list
posiciones=[]
for i in range(len(dfroute)):
    if dfroute.ix[i, 'Accion']!=0:
        posiciones.append(dfroute.ix[i, 'Punto'])

#print(posiciones)


class LabTest():
    def __init__(self):
        rospy.init_node('lab_test', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)

        # How long in seconds should the robot pause at each location? 10s default
        self.rest_time = rospy.get_param("~rest_time", 10)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        locations=dict()

        #locations['computador'] = Pose(Point(1.288, 0.094, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
        for point in posiciones:
             locations[point]= Pose(Point(df.ix[point, 'x'], df.ix[point, 'y'], 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
            #locations[point]= [df.ix[point, 'x'], df.ix[point, 'y']]
        
      #  locations['computador'] = Pose(Point(1.288, 0.094, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
      #  locations['izquierda_arriba'] = Pose(Point(1.428, 3.828, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
      #  locations['medio_estantes'] = Pose(Point(-0.461, 2.063, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
      #  locations['izquierda_abajo'] = Pose(Point(-2.528, 3.825, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
      #  locations['puerta'] = Pose(Point(-2.531, -1.800, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
      #  locations['estante_afuera'] = Pose(Point(-8.570, -2.846, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
        
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(posiciones)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation")
        
        # Begin the main loop and run through the sequence of locations
        while not rospy.is_shutdown():
            # If we've gone through the current sequence,
            # start again with the sequence
            if i == n_locations:
                i = 0
                sequence = posiciones
                # Skip over first location if it is the same as
                # the last location
                if sequence[0] == last_location:
                    i = 1
            
            # Get the next location in the current sequence
            location = sequence[i]
                        
            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x - 
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y - 
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x - 
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y - 
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""
            
            # Store the last location for distance calculations
            last_location = location
            
           
            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going next
            rospy.loginfo("Going to location: " + str(location))
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 5 minutes to get there (300 seconds)
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
            
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal, restarting goal")
		
		rate = 20
		r = rospy.Rate(rate)	
		self.tf_listener = tf.TransformListener()
		
		self.odom_frame = '/odom'

		try:
			self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
			self.base_frame = '/base_footprint'
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			try:
				self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_link'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
				rospy.signal_shutdown("tf Exception")
        
		move_cmd = Twist()
		goal_angle = 1800
		angular_tolerance = 2.5 # Converts 2.5 degrees to radians and sets it as tolerance value
		angular_speed = 0.5
		(position, rotation) = self.get_odom()
		
		move_cmd.angular.z = angular_speed
		last_angle = rotation
		turn_angle = 0
		#
		while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
        		self.cmd_vel_pub.publish(move_cmd)
			r.sleep()
			(position, rotation) = self.get_odom()
			delta_angle = normalize_angle(rotation - last_angle)
			turn_angle += delta_angle
			rospy.loginfo(turn_angle)
			last_angle = rotation

		#self.move_base.send_goal(self.goal)
            else:
		 # Increment the counters
            	
                        
		state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
		    i += 1
            	    n_goals += 1
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
			rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
			self.move_base.cancel_goal()
	                rospy.loginfo("Timed out achieving goal, restarting goal")
		
			rate = 20
			r = rospy.Rate(rate)	
			self.tf_listener = tf.TransformListener()
			
			self.odom_frame = '/odom'
	
			try:
				self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_footprint'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				try:
					self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
					self.base_frame = '/base_link'
				except (tf.Exception, tf.ConnectivityException, tf.LookupException):
					rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
					rospy.signal_shutdown("tf Exception")
        
			move_cmd = Twist()
			goal_angle = 1800
			angular_tolerance = 2.5 # Converts 2.5 degrees to radians and sets it as tolerance value
			angular_speed = 0.5
			(position, rotation) = self.get_odom()
		
			move_cmd.angular.z = angular_speed
			last_angle = rotation
			turn_angle = 0
			#
			while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
        			self.cmd_vel_pub.publish(move_cmd)
				r.sleep()
				(position, rotation) = self.get_odom()
				delta_angle = normalize_angle(rotation - last_angle)
				turn_angle += delta_angle
				rospy.loginfo(turn_angle)
				last_angle = rotation

			#self.move_base.send_goal(self.goal)
           
            	# How long have we been running?
            	running_time = rospy.Time.now() - start_time
            	running_time = running_time.secs / 60.0
            
            	# Print a summary success/failure, distance traveled and time elapsed
            	rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            	rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            	rospy.sleep(self.rest_time)
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    def get_odom(self):
	try:
		(trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
	except	(tf.Exception, tf.ConnectivityException, tf.LookupException):
		rospy.loginfo("TF Exception")
		return
	return (Point(*trans), quat_to_angle(Quaternion(*rot)))

def quat_to_angle(quat):
	rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
	return rot.GetRPY()[2]
def normalize_angle(angle):
	res = angle
	while res > pi:
		res -= 2.0 * pi
	while res < pi:
		res += 2.0 * pi
	return res		  
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        LabTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation finished.")
