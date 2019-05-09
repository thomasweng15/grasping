
import rospy
import cv2
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

global saved_depth, saved_color
saved_depth = False
saved_color = False

def depth_callback(data):
	global saved_depth, saved_color
	if saved_depth and saved_color:
		rospy.signal_shutdown("")
	elif saved_depth:
		return
	bridge = CvBridge()
	im = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
	cv2.imwrite(sys.argv[1] + "depth.png", im)
	saved_depth = True

def color_callback(data):
	global saved_depth, saved_color
	if saved_depth and saved_color:
		rospy.signal_shutdown("")
	elif saved_color:
		return
	bridge = CvBridge()
	im_bgr = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
	im_rgb = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2RGB)
	cv2.imwrite(sys.argv[1] + "color.png", im_rgb)
	saved_color = True

if __name__ == '__main__':
	rospy.init_node('listener')
	rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)
	rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
	rospy.spin()


