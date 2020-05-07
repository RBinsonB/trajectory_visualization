#!/usr/bin/env python
import sys
import rospy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from geometry_msgs.msg import Point

class trajectory_path:

	#=====================================
	#         Class constructor
	#  Initializes node and subscribers
	#=====================================
	def __init__(self):
		rospy.init_node('pose_to_path')

		self.loadParams()
		self.trajectory_path_msg = Path()
		self.previous_pose_position = Point()

		#Setup trajectory path publisher
		self.trajectory_path_pub = rospy.Publisher('~trajectory_path', Path, queue_size=10, latch=True)
		#Setup subscriber to pose
		self.pose_sub = rospy.Subscriber('pose_topic', Pose, self.pose_callback)
		#Setup subscriber to pose stamped
		self.pose_stamped_sub = rospy.Subscriber('pose_stamped_topic', PoseStamped, self.pose_stamped_callback)
		#Setup subscriber to pose with covariance
		self.pose_cov_sub = rospy.Subscriber('pose_cov_topic', PoseWithCovariance, self.pose_cov_callback)
		#Setup subscriber to pose with covariance stamped
		self.pose_cov_stamped_sub = rospy.Subscriber('pose_cov_stamped_topic', PoseWithCovarianceStamped, self.pose_cov_stamped_callback)
		#Setup subscriber to odometry
		self.odom_sub = rospy.subscriber('odom_topic', Odometry, self.odom_callback)


	#=====================================
	#       function for loading
	#    from ROS parameter server
	#=====================================
	def loadParams(self):
		#Load maximum number of poses in actual path
		self.max_poses = rospy.get_param('~max_poses', 1000)
		#Load threshold for adding a pose to actual path
		self.threshold = rospy.get_param('~movement_threshold', 0.001)
		#Load parent frame id for the trajectory
		self.frame_id = rospy.get_param('~frame_id', 'map')


	#=====================================
	#          Callback function 
	#     when receiving pose message
	#=====================================
	def pose_callback(self, pose_msg):
		rospy.logdebug("received pose message with position: "+str(pose_msg.position))
		#Process message position and add it to path
		self.publish_trajectory_path(pose_msg.position)

		
	#=====================================
	#      Callback function when
	#  receiving stamped pose message
	#=====================================
	def pose_stamped_callback(self, pose_stamped_msg):
		rospy.logdebug("received pose message with position: "+str(pose_stamped_msg.pose.position))
		#Process message position and add it to path
		if pose_stamped_msg.header.frame_id == self.frame_id :
			self.publish_trajectory_path(pose_stamped_msg.pose.position)
		else:
			rospy.logerror("PoseStamped message frame:"+pose_stamped_msg.header.frame_id+" does not correspond to trajectory frame"+self.frame_id)


	#=====================================
	#      Callback function when
	#  receiving pose with cov message
	#=====================================
	def pose_cov_callback(self, pose_cov_msg):
		rospy.logdebug("received pose cov message with position: "+str(pose_cov_msg.pose.position))
		#Process message position and add it to path
		self.publish_trajectory_path(pose_cov_msg.pose.position)


	#=====================================
	#      Callback function when
	#  receiving stamped pose cov message
	#=====================================
	def pose_cov_stamped_callback(self, pose_cov_stamped_msg):
		rospy.logdebug("received pose message with position: "+str(pose_cov_stamped_msg.pose.pose.position))
		#Process message position and add it to path
		if pose_cov_stamped_msg.header.frame_id == self.frame_id :
			self.publish_trajectory_path(pose_cov_stamped_msg.pose.pose.position)
		else:
			rospy.logerror("PoseWithCovaranceStamped message frame:"+pose_cov_stamped_msg.header.frame_id+" does not correspond to trajectory frame"+self.frame_id)


	#=====================================
	#      Callback function when
	#  receiving odometry message
	#=====================================
	def odom_callback(self, odom_msg):
		rospy.logdebug("received odom message with position: "+str(odom_msg.pose.pose.position))
		#Process message position and add it to path
		if odom_msg.header.frame_id == self.frame_id :
			self.publish_trajectory_path(odom_msg.pose.pose.position)
		else:
			rospy.logerror("Odometry message frame:"+odom_msg.header.frame_id+" does not correspond to trajectory frame"+self.frame_id)


	#=====================================
	#        Add pose and publish 
	#      trajectory path message
	#=====================================
	def publish_trajectory_path(self, position):
	#If the pose has move more than a set threshold, add it to the path message and publish
		if ((abs(self.previous_pose_position.x - position.x) > self.threshold)
		 or (abs(self.previous_pose_position.y - position.x) > self.threshold)
		 or (abs(self.previous_pose_position.z - position.x) > self.threshold)):
			rospy.logdebug('Exceding threshold, adding pose to path')
			#Add current pose to path
			self.trajectory_path_msg.header.stamp = rospy.Time.now()
			self.trajectory_path_msg.header.frame_id = self.frame_id
			pose_stamped_msg = PoseStamped()
			pose_stamped_msg.header.stamp = rospy.Time.now()
			pose_stamped_msg.pose.position.x = position.x
			pose_stamped_msg.pose.position.y = position.y
			pose_stamped_msg.pose.position.z = position.z

			#If max number of poses in path has not been reach, just add pose to message
			if len(self.trajectory_path_msg.poses) < self.max_poses:
				self.trajectory_path_msg.poses.append(pose_stamped_msg)
			#Else rotate the list to dismiss oldest value and add newer value at the end
			else :
				rospy.logdebug('Max number of poses reached, erasing oldest pose')
				self.trajectory_path_msg.poses = self.trajectory_path_msg.poses[1:]
				self.trajectory_path_msg.poses.append(pose_stamped_msg)

				self.previous_pose_position = pose_stamped_msg.pose.position
				self.trajectory_path_pub.publish(self.trajectory_path_msg)


#=====================================
#               Main
#=====================================
if __name__ == "__main__":
	trajectoryPath = trajectory_path()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"