#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Quaternion, Pose, PoseArray, PoseStamped
import math
import numpy
import actionlib
import sys
import time
import random
import tf
#import tf2_ros
#import tf2_geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import message_filters

from copy import deepcopy

roll = pitch = yaw = 0.0
x = y = z = 0.0
target =0
e_prev = 0
t_prev = -100
Kp=0.5
Kd=0.5
Ki=0.5
I=0
flag = False
ranges=[]


valid_angles = []
valid_ranges = []
flagUpdatePosition = True
flagUpdateRotation = True
flagUpdateRange = True
flagUpdatePoseArray = True

Nodes=[]


self_poses=PoseArray()
other_poses=PoseArray()

class Node:
	def __init__(self, node_x, node_y):
		self.node_x = node_x
		self.node_y = node_y

def rotate():    
	global yaw
	
	#e = abs(yaw-target_rad)
	e = 10*math.pi/180
	command.angular.z = e
	pub.publish(command)

	#print("target={} current:{}", target, yaw_deg)

def get_rotation (msg):
	global roll, pitch, yaw
	global x, y, z, flagUpdatePosition, flagUpdateRotation
	position_q = msg.pose.pose.position
	orientation_q = msg.pose.pose.orientation
	position_list = [position_q.x, position_q.y, position_q.z]
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	
	if flagUpdatePosition:
		(x, y, z) = (position_list)
		flagUpdatePosition=False
		#print x,y,z

	if flagUpdateRotation:
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		flagUpdateRotation=False


def callback(msg):
	global ranges
	global flagUpdateRange
	len_ranges = len(msg.ranges)
	#print 'len_ranges='+str(len_ranges)
	# values at 0 degree
	#print msg.ranges[0]
	if flagUpdateRange:
		ranges=[]
		ranges=deepcopy(msg.ranges)
		flagUpdateRange = False
	# values at 90 degree
	#print msg.ranges[len_ranges/2]
	# values at 180 degree
	#print msg.ranges[len_ranges-1]



def subPosesCallback(msg):
	global other_poses
	global flagUpdatePoseArray
	len_ranges = len(msg.poses)
	if flagUpdatePoseArray:
		other_poses=PoseArray()
		other_poses=deepcopy(msg)
		flagUpdatePoseArray = False


def new_callback(sub_scan, sub):
	global ranges
	global flagUpdateRange
	global roll, pitch, yaw
	global x, y, z, flagUpdatePosition, flagUpdateRotation
	position_q = sub.pose.pose.position
	orientation_q = sub.pose.pose.orientation
	position_list = [position_q.x, position_q.y, position_q.z]
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

	if flagUpdatePosition:
		(x, y, z) = (position_list)
		flagUpdatePosition=False
		#print x,y,z

	if flagUpdateRotation:
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		flagUpdateRotation=False

	len_ranges = len(sub_scan.ranges)
	#print 'len_ranges='+str(len_ranges)
	# values at 0 degree
	#print msg.ranges[0]

	if flagUpdateRange:
		ranges=[]
		ranges=deepcopy(sub_scan.ranges)
		flagUpdateRange = False

	# values at 90 degree
	#print msg.ranges[len_ranges/2]
	# values at 180 degree
	#print msg.ranges[len_ranges-1]


def sliding_window(X, valid_angles, window_size):
	global Nodes
	global self_poses
	global x, y, z
	global flagUpdateRotation, flagUpdatePosition
	global roll, pitch, yaw
	global other_poses
	global flagUpdatePoseArray

	
	
	# flag_suseq_found=False
	last_idx=0
	last_mean=numpy.mean(X)
	window=[]
	strWindow=''
	print('len(X)',len(X))

	temp_indices = []

	flagUpdateRotation = True
	flagUpdatePosition = True
	flagUpdatePoseArray = True
	time.sleep(2)

	print("From sliding window: yaw="+str(yaw))

	other_poses_transformed = transformPoses(other_poses)

	for i in range(len(X)):
		
		if i==0:
			window=range(i-window_size, i)
			window.extend(range(i, i+window_size))
			strWindow = 'Window 1'
		elif i>=len(X)-window_size-1:
			window=range(i-window_size, i)
			window.extend(range(i,len(X)-1))
			window.extend(range(0, (i+window_size)%(len(X)-1)))
			strWindow = 'Window 2'
		else:
			#window=[i-1, i, i+1]
			window=range(i-window_size, i+window_size)
			strWindow = 'Window 3'

		
		try:
			mean = numpy.mean([X[x] for x in window])
		except:
			print("window: ", window)
			print(strWindow)
			print('len(X)',len(X))
			#sys.exit(0)
			#return -1
			print("Exception occurrend in one window")
			continue



		dist=X[i]
		if dist>=1:
			dist = dist-1
		else:
			dist = 0
		target=valid_angles[i]
		#print("current_x, current_y, heading_angle:={}, {}", x, y, target)
		#updated 1st june
		#flagUpdateRotation = True
		#flagUpdatePosition = True
		target_x=x+(dist*math.cos(math.radians(target)))#-yaw))
		target_y=y+(dist*math.sin(math.radians(target)))#-yaw))

		current_node = Node(target_x, target_y)
		print("target_x, target_y, target", target_x, target_y, target)
		print('current_node: ' + str(current_node.node_x) + ',' + str(current_node.node_y))


		if mean>1 and test_distances(current_node, 0.5):
			temp_indices.append(i)
		
		if mean>last_mean:
			last_mean=mean
			last_idx=i


	flagGoalReached = False
	temp_x = x
	temp_y = y
	temp_yaw = yaw
	
	print('temp_indices', temp_indices)
	    
	
	while(not flagGoalReached):
		
		if len(temp_indices)>0:
			random_num = random.choice(temp_indices)
			last_idx = random_num
		#else go to the last_idx, which is the maximum available distance...
		else: #if temp_indices is empty
			last_idx=-1
			break


		dist=X[last_idx]
		#dist=numpy.mean(X)
		if dist>1:
			dist = dist-1
		else:
			dist = 0
		target = valid_angles[last_idx]

		flagUpdateRotation = True
		flagUpdatePosition = True
		time.sleep(2)

		target_x=temp_x+(dist*math.cos(math.radians(target)))#-temp_yaw))
		target_y=temp_y+(dist*math.sin(math.radians(target)))#-temp_yaw))
		current_node = Node(target_x, target_y)

		#print("current_x, current_y, yaw", temp_x, temp_y, math.degrees(temp_yaw))
		#print("target_x, target_y, target", target_x, target_y, target)

	
		
		
	
	
		try:
			result = movebase_client(current_node.node_x, current_node.node_y)

			#Check if the goal distance is close to actual position
			if result:
				flagGoalReached = True
				#Nodes.append(current_node)
				#self_poses=PoseArray()
				current_pose = Pose()
				current_pose.position.x = current_node.node_x
				current_pose.position.y = current_node.node_y
				self_poses.header.stamp = rospy.Time.now()
				self_poses.header.frame_id = "robot1/odom"
				#self_poses.poses = []
				self_poses.poses.append(current_pose)
				pub_poses.publish(self_poses)
				print('pose array published........')
		except (rospy.ROSInterruptException, KeyboardInterrupt) as error:
			#rospy.loginfo("Navigation test finished.")
			#sys.exit(0)
			print("Exception", error)
			print("Exception occurred in move_base goal navigation or Keyboard Interrupt!!")
			flagGoalReached = True
			last_idx = -1

	
	#return random_num
	return last_idx



def movebase_client(target_x, target_y):
	global x, y, z
	# Create an action client called "move_base" with action definition file "MoveBaseAction"
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

	# Waits until the action server has started up and started listening for goals.
	client.wait_for_server()

	# Creates a new goal with the MoveBaseGoal constructor
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "robot1/odom"
	goal.target_pose.header.stamp = rospy.Time.now()
	# Move 0.5 meters forward along the x axis of the "map" coordinate frame 
	goal.target_pose.pose.position.x = target_x
	goal.target_pose.pose.position.y = target_y
	# No rotation of the mobile base frame w.r.t. map frame
	#goal.target_pose.pose.orientation.w = 1.0
	#goal.target_pose.pose.orientation.w = math.atan2((goal.target_pose.pose.position.y-y), (goal.target_pose.pose.position.x-x))

	angle_to_goal=math.atan2((goal.target_pose.pose.position.y-y),  (goal.target_pose.pose.position.x-x))

	#q_angle = quaternion_from_euler(0, 0, angle_to_goal, axes='sxyz')
	q_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')

	q = Quaternion(*q_angle.tolist())

	goal.target_pose.pose.orientation = q

	# Sends the goal to the action server.
	client.send_goal(goal)
	# Waits for the server to finish performing the action.
	wait = client.wait_for_result(timeout=rospy.Duration(20.0))

	#print('client.get_state()', client.get_state())
	
	# If the result doesn't arrive, assume the Server is not available

	if client.get_state() == GoalStatus.SUCCEEDED:
		result = client.get_result()
		print "Result: SUCCEEDED " 
		return True
	elif client.get_state() == GoalStatus.PREEMPTED:
		print "Action pre-empted"
		goal = MoveBaseGoal()
		client.send_goal(goal)
		return False
	else:
		print 'Action failed'
		goal = MoveBaseGoal()
		client.send_goal(goal)
		return False


	"""if not wait:
		client.cancel_goal()
		print('Goal timed out...')
		#rospy.logerr("Action server not available!")
		#rospy.signal_shutdown("Action server not available!")
	else:
		# Result of executing the action
		print('client.get_result()', client.get_result())"""
		
	#return client.get_result()




def test_distances(current_node, threshold):
	#global Nodes
	global self_poses
	
	


	if(len(self_poses.poses) == 0):
		return True
	
	distances=[0 for i in range(len(self_poses.poses))]
	#print("length of poses in last seq:"+str(self_poses.poses))
	idx = 0
	for i in range(len(self_poses.poses)):
		node=self_poses.poses[i]
		#distances[idx] = math.sqrt((node.node_y - current_node.node_y)**2 + (node.node_x - current_node.node_x)**2)
		distances[idx] = math.sqrt((node.position.y - current_node.node_y)**2 + (node.position.x - current_node.node_x)**2)
		print('registered self node: ' + str(node.position.x) + ',' + str(node.position.y))
		idx = idx + 1

	#print('distances: ', distances)




def test_distances_others(current_node, other_poses_transformed, threshold):

	#print('length of other_poses = '+str(len(other_poses_transformed.poses)))

	distances_others=[0 for i in range(len(other_poses_transformed.poses))]
	#print("length of poses in last seq:"+str(self_poses.poses))
	idx = 0
	for j in range(len(other_poses_transformed.poses)):
		node_other=other_poses_transformed.poses[j]
		#distances[idx] = math.sqrt((node.node_y - current_node.node_y)**2 + (node.node_x - current_node.node_x)**2)
		distances_others[idx] = math.sqrt((node_other.position.y - current_node.node_y)**2 + (node_other.position.x - current_node.node_x)**2)
		print('registered other node: ' + str(node_other.position.x) + ',' + str(node_other.position.y))
		idx = idx + 1

	if(all(distances_others) > threshold):
		return True
	else:
		return False




def transformPoses(other_poses):
	print('length of other_poses = '+str(len(other_poses.poses)))
	other_poses_transformed = PoseArray()
	other_poses_transformed.header.frame_id = "robot1/odom"
	
	listener = tf.TransformListener()

	distances_others=[0 for i in range(len(other_poses.poses))]
	#print("length of poses in last seq:"+str(self_poses.poses))

	for i in range(len(other_poses.poses)):
		node_other=other_poses.poses[i]

		pose_stamped = PoseStamped()
		pose_stamped.pose = node_other
		pose_stamped.header.frame_id = other_poses.header.frame_id
		pose_stamped.header.stamp = rospy.Time.now()

		try:
			output_pose_stamped = listener.transformPose('robot1/odom', pose_stamped)
			other_poses_transformed.poses.append(output_pose_stamped)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException, KeyboardInterrupt) as error:
			print("Exception from transformPoses method: ", error)
			continue

	return other_poses_transformed
		



#Define the Subscribed Topics

rospy.init_node('robot1_rotate_robot')
sub_scan = message_filters.Subscriber('/robot1/scan', LaserScan)
sub = message_filters.Subscriber('/robot1/odom', Odometry)
#sub_scan = rospy.Subscriber ('/scan', LaserScan, callback)
#sub = rospy.Subscriber ('/odom', Odometry, get_rotation)

ts = message_filters.ApproximateTimeSynchronizer([sub_scan, sub], queue_size = 5, slop = 0.5)
ts.registerCallback(new_callback)


#Define the Published Topics

pub = rospy.Publisher('/robot1/cmd_vel_mux/input/teleop', Twist, queue_size=1)
r = rospy.Rate(10)

pub_poses = rospy.Publisher('/robot1/poseArray', PoseArray, queue_size=1)
sub_poses = rospy.Subscriber('/robot2/poseArray', PoseArray, subPosesCallback)
command =Twist()


#rospy.spin()
flagUpdatePosition = True
flagUpdateRotation = True
flagUpdateRange = True
#while(len(ranges)==0):
#	r.sleep()

#Actual Algorithm Starts

for iter in range(1, 100):
	valid_angles=[]
	valid_ranges=[]

	phase = 'rotate'
	flagUpdatePosition = True
	flagUpdateRotation = True
	flagUpdateRange = True
	flag = False
	


	while True:
	    
	    #quat = quaternion_from_euler (roll, pitch,yaw)
	    #print quat

		try:
			if phase == 'rotate':	
				yaw_deg = yaw*180/math.pi
				if len(ranges)>0:
					#print("generating costmap")
					flagUpdateRotation = True
					flagUpdateRange = True
					flagUpdatePosition = True
					valid_angles.append(yaw_deg)
					valid_ranges.append(ranges[(len(ranges)-1)/2])
			
			

				if not flag:
					#print('I am here...')
					if yaw_deg<=0:
						target = yaw_deg - 1
				    		flag = True
					else:
						if yaw_deg>1:
							target = yaw_deg - 1
						else:
							target = -1 #-yaw_deg
				    		flag = True
				   
				target_rad = target*math.pi/180

				if flag:
					if abs(yaw_deg - target)<0.9:
						#print('Error: ', abs(yaw - target_rad))
						phase = 'forward'
						print("Breakig loop...")

						break
					else:
						rotate()

				print("target={} current:{}", target, yaw_deg)
				flagUpdateRotation = True
				flagUpdateRange = True
				flagUpdatePosition = True


		
				r.sleep()
		except KeyboardInterrupt:
			sys.exit(0)



	pub.publish(Twist())

	"""valid_angles=numpy.arange(-0.521567881107, 0.524276316166, 0.00163668883033)
	valid_angles=[x*180/math.pi for x in valid_angles]
	valid_ranges=deepcopy(ranges)
	#print(valid_ranges)"""

	print("Sliding Window!!")

	flagUpdateRotation = True
	flagUpdateRange = True
	flagUpdatePosition = True

	#find a good heading direction
	#print("Size of valid angles", len(valid_angles))

	#valid_ranges=[math.nan, 1, 1, 2, 3, 4, math.nan, math.nan, math.nan, 5, 6, 6, 7]
	max_range=numpy.nanmax(valid_ranges)

	new_ranges=[8 if math.isnan(x) or x>8 else x for x in valid_ranges]
	window_size=2


	print('length of new_ranges from main loop: ', str(len(new_ranges)))
	#if local costmap not estimated correctly, redo the costmap collection
	if len(new_ranges)<100:
		continue

	heading=sliding_window(new_ranges, valid_angles, window_size)
	#print("Moving Forward!!")
	if heading == -1:
		print("Robot did not move...iter: "+str(iter))		
		#break

	#print("Valid Heading Found")
	#print([valid_ranges[x] for x in window])
	# print(lcs_ranges(ranges, max_ranges))


	#phase = 'rotate'
	#heading direction found
	

	#current_node=Nodes[len(Nodes)-1]
	#print("target_x, target_y, distance:={}, {}", target_x, target_y, dist)

	"""try:
		result = movebase_client(current_node.node_x, current_node.node_y)
		if result:
			rospy.loginfo("Goal execution done!")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")
		sys.exit(0)"""



