#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from darknet_ros_msgs.msg import BoundingBoxes
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Joy

class PersonFollower():
    def __init__(self):
        IMAGE_WIDTH = 640
        IMAGE_HEIGHT = 480
        SERVO_RANGE = 0.34
        MAX_PERCENT_OF_FRAME = 0.27
        MIN_PERCENT_OF_FRAME = 0.18
        
        
        self.last_box = None
        self.person_bounding_box = None
	self.right_bumper = False
        # node setup
        rospy.init_node("person_follower", anonymous=True)
        sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, lambda x: self.cb(x))
	joy_sub = rospy.Subscriber('/vesc/joy', Joy, self.right_bump)
	pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        rate = rospy.Rate(30)
        seq = 0
        while not rospy.is_shutdown():
            seq += 1
            header = Header(seq=seq, stamp=rospy.Time.now())
            bb = self.person_bounding_box
            if bb is not None:
                boxCenter = (bb.xmin + bb.xmax) / 2.0
		ratio = boxCenter / 640.0
		angle = -1 * (-SERVO_RANGE + ratio * 2 * SERVO_RANGE)
                percent_of_frame = (abs(bb.xmax - bb.xmin) * abs(bb.ymax - bb.ymin)) / (640.0 * 480.0)
                speed = 0
                if percent_of_frame > MAX_PERCENT_OF_FRAME:
                    speed = -0.5
                    angle = -angle
                elif percent_of_frame < MIN_PERCENT_OF_FRAME:
                    speed = 0.5
		#print("angle is", angle, "speed is", speed, "percent of frame is", percent_of_frame)
                if self.right_bumper:
                    pub.publish(AckermannDriveStamped(header, AckermannDrive(steering_angle=angle, speed=speed)))
            rate.sleep()
        rospy.spin()

    def right_bump(self, data):
	self.right_bumper = bool(data.buttons[5])
        
    def area(self, a, b):
        dx = min(a.xmax, b.xmax) - max(a.xmin, b.xmin)
        dy = min(a.ymax, b.ymax) - max(a.ymin, b.ymin)
        if (dx >= 0) and (dy >= 0):
            return dx*dys

    def cb(self, data):
        
        best_match = None
        best_intersect = 0.0 
        box_found = False

        for bounding_box in data.bounding_boxes:
            bb = bounding_box
            size = abs(bb.xmax - bb.xmin) * abs(bb.ymax - bb.ymin)
            if bounding_box.Class == "person" and bounding_box.probability > 0.8 and size > 0.03 * 640 * 480:
                #print("Person detected! Confidence", bounding_box.probability)
                
                
                if self.last_box == None:
                    best_intersect = 1
                    self.person_bounding_box = bounding_box
                    self.last_box = bounding_box
                    box_found = True
                    break
                else:
                    intersect = self.area(bb, self.last_box)
                    if intersect > best_intersect:
                        best_match = bb
                        best_intersect = intersect
                        box_found = True
            self.last_box = best_match
            self.person_bounding_box = best_match
        if box_found:
            p = self.last_box
            prev_area = (p.ymax - p.ymin) * (p.xmax - p.xmin)


            print "Intersect percentage:", best_intersect / (1.0 * prev_area )
            #print self.person_bounding_box.xmax
        if box_found == False:
            self.person_bounding_box = None
            # print("No person found :( lonely robot is sad") 


if __name__ == "__main__":
    pf = PersonFollower()
