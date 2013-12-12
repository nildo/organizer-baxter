import rospy
import baxter_interface
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from std_msgs.msg import Header
from baxter_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from sensor_msgs.msg import Image
from organizer.srv import ImageSrv, ImageSrvResponse
import numpy as np
import cv, cv2
from cv_bridge import CvBridge, CvBridgeError

class Baxter:
	def __init__(self):
		pass

	def enable(self):
		# An attribute of class RobotEnable to enable or disable Baxter.
		# It has the following methods:
		#	enable()	- enable all joints
		#	disable()	- disable all joints
		#	reset()	- reset all joints, reset all jrcp faults, disable the robot
		#	stop()		- stop the robot, similar to hitting the e-stop button
		self.robotEnable = baxter_interface.RobotEnable()
		self.robotEnable.enable()
		self.leftArm = baxter_interface.Limb('left')
		self.rightArm = baxter_interface.Limb('right')
		# Waits for the image service of right hand camera to become available.
		rospy.wait_for_service('last_image')
		self.rightHandCamera = rospy.ServiceProxy('last_image', ImageSrv)
		self.leftGripper = baxter_interface.Gripper('left')
		
		
	def disable(self):
		self.robotEnable.disable()
		
	def getLeftArmPosition(self):
		position = self.leftArm.endpoint_pose()
		x = position['position'].x
		y = position['position'].y
		z = position['position'].z
		return [x,y,z]

	def getLeftArmOrientation(self):
		orientation = self.leftArm.endpoint_pose()
		x = orientation['orientation'].x
		y = orientation['orientation'].y
		z = orientation['orientation'].z
		w = orientation['orientation'].w
		return [x,y,z,w]
		
	def getRightArmPosition(self):
		position = self.rightArm.endpoint_pose()
		x = position['position'].x
		y = position['position'].y
		z = position['position'].z
		return [x,y,z]

	def getRightArmOrientation(self):
		orientation = self.rightArm.endpoint_pose()
		x = orientation['orientation'].x
		y = orientation['orientation'].y
		z = orientation['orientation'].z
		w = orientation['orientation'].w
		return [x,y,z,w]
		
	def inverseKinematics(self, limb, point, orientation):
		ns = "/sdk/robot/limb/" + limb + "/solve_ik_position"
		rospy.wait_for_service(ns)
		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		pose = PoseStamped(
					header=hdr,
					pose=Pose(
					position=Point(
						x=point[0],
						y=point[1],
						z=point[2],
					),
					orientation=Quaternion(
						x=orientation[0],
						y=orientation[1],
						z=orientation[2],
						w=orientation[3],
					)
				)
			);
		ikreq.pose_stamp.append(pose)
		try:
			resp = iksvc(ikreq)
		except rospy.ServiceException,e :
			rospy.loginfo("Service call failed: %s" % (e,))
		if (resp.isValid[0]):
			#print("SUCCESS - Valid Joint Solution Found:")
			# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].names, resp.joints[0].angles))
			#print limb_joints
			return limb_joints
		else:
			print("INVALID POSE - No Valid Joint Solution Found.")
			return None
			
	def ik(self, point, orientation):
		angles = self.inverseKinematics('left', point, orientation)
		if not angles:
			return False
		else:
			return True
			
		
	def moveLeftArm(self, point, orientation):
		angles = self.inverseKinematics('left', point, orientation)
		if not angles:
			return None
		self.leftArm.move_to_joint_positions(angles) # 15 secs timeout default.
		
	def getImageFromRightHandCamera(self):
		"""
		This method returns a numpy array of the image that was captured from
		Baxter's right hand camera.
		
		"""
		request = self.rightHandCamera()
		imgmsg = request.last_image
		bridge = CvBridge()
		cvimage = bridge.imgmsg_to_cv(imgmsg,'bgr8')
		# Save the last image captured.
		self.last_cvimage = cvimage
		cvimage_array = np.asarray(cvimage, dtype=np.uint8)
		return cvimage_array
		
	def getLastCvImage(self):
		return self.last_cvimage
	
	def getLastArrayImage(self):
		cvimage_array = np.asarray(self.last_cvimage, dtype=np.uint8)
		return cvimage_array
	
	# Gripper Methods:
	
	def calibrateLeftGripper(self):
		self.leftGripper.calibrate()
		
	def closeLeftGripper(self):
		self.leftGripper.close()
		
	def openLeftGripper(self):
		self.leftGripper.open()
