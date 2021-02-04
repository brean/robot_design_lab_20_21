#!/usr/bin/python
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

class ImageProcessing:
    def __init__(self):
        self.bridge = CvBridge()

    def listen(self):
        rospy.init_node('image_processing', anonymous=True)
        self.image_sub = rospy.Subscriber("camera/image", Image, self.callback)
        rospy.spin()

    def callback(self, raw_image):
        try:
            image = self.bridge.imgmsg_to_cv2(raw_image, desired_encoding="bgr8")
            image = self.apply_filter(image)
            cv2.imshow('image', image)
            cv2.waitKey(30)
        except CvBridgeError as e:
            rospy.logerr(e)

    def apply_filter(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        lower_green = np.array([60,10,60])
        upper_green = np.array([255,255,255])

        mask = cv2.inRange(hsv, lower_green, upper_green)
        res = cv2.bitwise_and(img, img, mask=mask)
        return res

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == '__main__':
    img = ImageProcessing()
    img.listen()
    img.cleanup()