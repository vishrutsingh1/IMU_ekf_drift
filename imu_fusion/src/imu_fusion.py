#!/usr/bin/env python
import rospy
from robot_localization.srv import ToggleFilterProcessing , ToggleFilterProcessingRequest,SetPose,SetPoseRequest
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import PoseWithCovarianceStamped 


class IMU_Filter(object):

	def __init__(self):
		self.time_then = rospy.get_rostime()
		self.time_now = 0
		self.switch = 0
		self.sub_odom = rospy.Subscriber("odom",Odometry,self.odom_callback)
		self.sub_odom_filter = rospy.Subscriber("odometry/filtered",Odometry,self.odom_filter_callback)
		##self.pose_pub = rospy.Publisher("set_pose" ,PoseWithCovariance,queue_size = 5)
		rospy.wait_for_service("/ekf_se/toggle")
		rospy.wait_for_service("/set_pose")
		self.toggle_service = rospy.ServiceProxy("/ekf_se/toggle",ToggleFilterProcessing)
		self.pose_service =   rospy.ServiceProxy("/set_pose",SetPose)
		self.toggle_request = ToggleFilterProcessingRequest()
		self.pose = SetPoseRequest() 
		self.linear_vel = 0
		self.angular_vel = 0
		self.time = 0
		self.pos_time = 0
		self.pos_time_then = rospy.get_rostime()
		self.pos_time_now = 0
		self.timer = rospy.get_param('~timer' , 5.0)
		self.pose_correct_timer = rospy.get_param('~pose_correct_timer' , 10.0)
		rospy.loginfo(self.timer)
		rospy.loginfo(self.pose_correct_timer)




	#def halt_service(self):
		
	#	self.toggle_request.on = False
	#	self.toggle_service(self.toggle_request)
	#	rospy.loginfo("halt state")

	#def start_service(self):
		
	#	self.toggle_request.on = True
	#	self.toggle_service(self.toggle_request)
	#	rospy.loginfo("active state")









	def odom_callback(self,msg):
		self.linear_vel = abs(msg.twist.twist.linear.x)
		self.angular_vel = abs(msg.twist.twist.angular.z)
		self.pose_time_now = rospy.Time.now()
		self.pos_time = self.pose_time_now.secs - self.pos_time_then.secs
		

		



		if(self.linear_vel == 0.00  and self.angular_vel == 0.00):
			self.time_now = rospy.get_rostime()
			self.time =  self.time_now.secs - self.time_then.secs
			

			

		if(self.linear_vel != 0.00 or self.angular_vel != 0.00):
			self.time_then = rospy.Time.now()

		
		
		if(self. switch== 1 and (self.linear_vel != 0.00 or self.angular_vel != 0.00)):
			#self.start_service()
			self.pose.pose.header.stamp = rospy.Time.now()
			self.pose_service(self.pose)
			rospy.loginfo("pose corrected !")
			self.switch = 0
			self.time = 0
			print("start")

		if (self.time >= self.timer and self.switch == 0):
			#self.halt_service()
			self.pos_time_then = rospy.get_rostime()
			self.pos_time = 0
			self.switch = 1
			print("stop")

		if(self.switch == 1  and self.pos_time >= self.pose_correct_timer):
			self.pose.pose.header.stamp = rospy.Time.now()
			self.pose_service(self.pose)
			self.pos_time = 0
			self.pos_time_then = rospy.get_rostime()
			rospy.loginfo("pose corrected !")





	def odom_filter_callback(self,data):

		if(self.switch == 0 and (self.linear_vel == 0.00  and self.angular_vel == 0.00)):

			self.pose.pose.header.frame_id = "odom"
			self.pose.pose.pose.pose = data.pose.pose
			self.pose.pose.pose.covariance = data.pose.covariance  	
			





	


	
























if __name__ == '__main__':
	try:
		rospy.init_node("imu_filter_toggle")
		obj = IMU_Filter()
	except rospy.ROSInterruptException:
		pass
		
	rospy.spin()
