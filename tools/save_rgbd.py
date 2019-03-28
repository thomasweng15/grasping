import rospy
import sys
from sensor_msgs.msg import Image
import pprint
import cv2
from cv_bridge import CvBridge
import tf
import time

global saved_RGB
global saved_Depth
saved_RGB = False
saved_Depth = False

filename = None

def callbackWriteRGB(data):
    global saved_RGB, saved_Depth
    if(saved_RGB==True and saved_Depth==True): 
        rospy.signal_shutdown("")

    bridge = CvBridge()
    cv_image1 = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    cv_image2 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2RGB)

    if(saved_RGB==False):
            cv2.imwrite(filename + '.jpg', cv_image2)
            print("Image Saved")
            saved_RGB = True

def callbackWriteDepth(data):
    global saved_RGB, saved_Depth
    if (saved_RGB==True and saved_Depth==False):
        bridge = CvBridge()
        cv_image1 = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        cv2.imwrite(filename + '_depth.jpg', cv_image1)
        print("Depth Saved")
        saved_Depth = True

def main():
    global filename
    filename = sys.argv[1]
    print("filename: %s" % filename)

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callbackWriteDepth)
    rospy.Subscriber("/camera/color/image_raw", Image, callbackWriteRGB)
    rospy.spin()

if __name__ == "__main__":
	main()





