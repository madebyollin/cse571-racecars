#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from darknet_ros_msgs.msg import BoundingBoxes
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class PersonFollower():
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    SERVO_RANGE = 0.68
    def __init__(self):
        self.person_bounding_box = None
        rospy.init_node("person_follower", anonymous=True)
        sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.cb)
        pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        rate = rospy.Rate(50)
        seq = 0
        while not rospy.is_shutdown():
            seq += 1
            header = Header(seq=seq, stamp=rospy.Time.now())
            # TODO: fix this
            bb = self.person_bounding_box
            if bb is not None:
                boxCenter = (bb.xmin + bb.xmax)/2.0
		ratio = boxCenter/640.0
		angle = (-0.34 + ratio*0.64)*-1.0
		#print ratio
		
		
                
		print("angle is", angle)
                #pub.publish()
                #print "self.person_bounding_box is ", self.person_bounding_box
                pub.publish(AckermannDriveStamped(header, AckermannDrive(steering_angle=angle)))
            rate.sleep()
        rospy.spin()

    def cb(self, data):
        for bounding_box in data.bounding_boxes:
            if bounding_box.Class == "person":
                #print("Person detected! Confidence", bounding_box.probability)
                bb = bounding_box
                size = abs(bb.xmax - bb.xmin) * abs(bb.ymax - bb.ymin)
                if bounding_box.probability > 0.8 and size > 0.05 * 640 * 480:
                    self.person_bounding_box = bounding_box 
                break
        else:
            pass
            # print("No person found :( lonely robot is sad") 


if __name__ == "__main__":
    pf = PersonFollower()
