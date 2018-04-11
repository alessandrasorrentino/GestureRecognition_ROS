#!/usr/bin/env python
import time
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from tfpose_ros.msg import Persons, Person, BodyPartElm
from estimator import Human, BodyPart, TfPoseEstimator

from geometry_msgs.msg import Twist

class VideoFrames:
	"""
	Reference : ros-video-recorder
	https://github.com/ildoonet/ros-video-recorder/blob/master/scripts/recorder.py
	"""
	def __init__(self, image_topic):
		self.image_sub = rospy.Subscriber(image_topic, Image, self.callback_image, queue_size=1)
		self.bridge = CvBridge()
		self.frames = []

	def callback_image(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr('Converting Image Error. ' + str(e))
			return

		self.frames.append((data.header.stamp, cv_image))

	def get_latest(self, at_time, remove_older=True):
		fs = [x for x in self.frames if x[0] <= at_time]
		if len(fs) == 0:
			return None

		f = fs[-1]
		if remove_older:
			self.frames = self.frames[len(fs) - 1:]

		return f[1]
##ALE        
def saveposes(humans, image_h, image_w):
	estimations=np.zeros(18*2)
	human=humans[0]
	i=0
	while i <18:
		if i not in human.body_parts.keys():
			estimations[i]= -1  
			estimations[i+18]= -1 
		else:
			body_part = human.body_parts[i]
			estimations[i]=int(body_part.x * image_w + 0.5) 
			estimations[i+18]= int(body_part.y * image_h + 0.5)
		i=i+1
	return estimations

def recognizeGesture(estimations):
	max_tv = 0.5
	max_rv = 1.0#yaw_rate_
	speed = 0
	turn = 0
	
	if estimations[4]>=0 and estimations[2]>=0 and estimations[21]>=0 and estimations[22]>=0 and estimations[22]<estimations[21]:
		if estimations[4]<estimations[2]:
			print('RICONOSCIUTO GESTO AVANTI')
			max_tv = 1.5#walk_vel_
			speed = 2
			turn = 0
		else:
			print('RICONOSCIUTO GESTO INDIETRO')
			max_tv = 1.5#walk_vel_
			speed = -2
			turn = 0
	elif estimations[5]>=0 and estimations[6]>=0 and estimations[24]>=0 and estimations[25]>=0 and estimations[24]> estimations[25]:
		if estimations[7]< estimations[5]: #braccio interno
			print('RICONOSCIUTO GESTO DESTRA')
			max_rv = 2.0#yaw_rate_
			speed = 0
			turn = -2
		else:
			print('RICONOSCIUTO GESTO SINISTRA')
			max_rv = 2.0#yaw_rate_
			speed = 0
			turn = 2
	else:
		print ('RIPOSO')
		speed = 0
		turn = 0

	cmdvel_ = Twist()
	cmdvel_.linear.x = speed * max_tv
	cmdvel_.angular.z = turn * max_rv
	pub_vel.publish(cmdvel_);
	

def cb_pose(data):
	# get image with pose time
	t = data.header.stamp
	image = vf.get_latest(t, remove_older=True)
	if image is None:
		rospy.logwarn('No received images.')
		return

	h, w = image.shape[:2]
	if resize_ratio > 0:
		image = cv2.resize(image, (int(resize_ratio*w), int(resize_ratio*h)), interpolation=cv2.INTER_LINEAR)

	# ros topic to Person instance
	humans = []
	for p_idx, person in enumerate(data.persons):
		human = Human([])
		for body_part in person.body_part:
			part = BodyPart('', body_part.part_id, body_part.x, body_part.y, body_part.confidence)
			human.body_parts[body_part.part_id] = part

		humans.append(human)
	estimations=saveposes(humans, h, w)

	# draw
	image = TfPoseEstimator.draw_humans(image, humans, imgcopy=False)
	pub_img.publish(cv_bridge.cv2_to_imgmsg(image, 'bgr8'))

	recognizeGesture(estimations)

if __name__ == '__main__':
	rospy.loginfo('initialization+')
	rospy.init_node('TfPoseEstimatorROS-Visualization', anonymous=True)

	# topics params
	image_topic = rospy.get_param('~camera', '')
	pose_topic = rospy.get_param('~pose', '/pose_estimator/pose')

	resize_ratio = float(rospy.get_param('~resize_ratio', '-1'))

	# publishers
	pub_img = rospy.Publisher('~output', Image, queue_size=1)
	pub_vel = rospy.Publisher("joystick_cmd_vel",Twist, queue_size=1)

	# initialization
	cv_bridge = CvBridge()
	vf = VideoFrames(image_topic)
	rospy.wait_for_message(image_topic, Image, timeout=30)

	# subscribers
	rospy.Subscriber(pose_topic, Persons, cb_pose, queue_size=1)

	# run
	rospy.spin()
