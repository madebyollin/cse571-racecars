#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge 
from std_msgs.msg import Header
from darknet_ros_msgs.msg import BoundingBox
# from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Image
import numpy as np
br = CvBridge()
class tracker_visulizer(object):

    def __init__(self):
        self.image = None
        self.tracker = None
        self._pub = rospy.Publisher('personfollower/tv', Image, queue_size=10)

    def wait_data(self):
        rate = rospy.Rate(30)
        while True:
            if (self.image != None) and (self.tracker != None):
                print('begin visulizer')
                return True
            rate.sleep()

    def image_cb(self, data):
        self.image = data

    def tracker_cb(self, data):
        self.tracker = data

    def tracker_pub(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if (self.image != None) and (self.tracker != None):
                img = br.imgmsg_to_cv2(self.image)
                xmin, ymin, xmax, ymax = self.tracker.xmin, self.tracker.ymin, self.tracker.xmax,self.tracker.ymax
                cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0,255,0), 4)
                img = br.cv2_to_imgmsg(img, "rgb8")
                self._pub.publish(img)
            rate.sleep()
        rospy.spin()

def main():
    rospy.init_node("tracker_visulizer", anonymous=True)
    tv = tracker_visulizer()
    img_sub = rospy.Subscriber('/camera/color/image_raw', Image, tv.image_cb)
    tracker_sub = rospy.Subscriber('/personfollower/tracker_box', BoundingBox, tv.tracker_cb)

    # tv.wait_data()
    tv.tracker_pub()

if __name__ == "__main__":
    main()
