#!/usr/bin/env python

import roslib; roslib.load_manifest('organizer')
import rospy
import baxter_interface
from sensor_msgs.msg import Image
from organizer.srv import ImageSrv, ImageSrvResponse

class ImgService:
	def imgReceived(self, message):
		"""
		Callback for when an image is received.

		"""
		self.lastImage = message

	def getLastImage(self, request):
		"""
		Return the last image

		"""
		return ImageSrvResponse(self.lastImage)

	def __init__(self):
		"""
		Construtor: initializes node, subscribe to Baxter's right hand camera
		topic and create the service.

		"""
		self.lastImage = None;
		rospy.init_node('right_hand_camera_srv')
		
		cameraController = baxter_interface.CameraController("right_hand_camera")
		cameraController.close()
		cameraController.resolution= (1280,800)
		cameraController.open()

		rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.imgReceived)
		rospy.Service('last_image', ImageSrv, self.getLastImage)

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	node = ImgService()
	node.run()

