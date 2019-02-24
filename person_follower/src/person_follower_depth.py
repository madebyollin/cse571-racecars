#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Header
from darknet_ros_msgs.msg import BoundingBoxes
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Joy, Image
import numpy as np

br = CvBridge()

CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_AREA = CAMERA_WIDTH * CAMERA_HEIGHT

class PersonTracker:
    def __init__(self):
        # Real data
        self.bounding_boxes = None
        self.depth = None

        # Predictions
        self.predicted_bounding_box = None
        self.prediction_confidence = 0
        self.predicted_depth = None

    def depth_cb(self, data):
        """Callback for messages from the depth camera. Relies on self.person_bounding_box and sets self.person_depth"""
        averaged_depth = 0.0
        image = br.imgmsg_to_cv2(data)
        bb = self.bounding_box
        if bb is not None:
            person_region = image[bb.xmin:bb.xmax, bb.ymin:bb.ymax]
            # averaged_depth = np.mean(person_region)
            x_avg = (bb.xmin + bb.xmax) / 2
            y_avg = (bb.ymin + bb.ymax) / 2
            # NOTE: this could be replaced with a smarter weighted average
            # (with depths in the center of the bounding box higher-weighted)
            # only update depth if we can actually see them
            if 0 <= x_avg < CAMERA_WIDTH and 0 < y_avg <= CAMERA_HEIGHT:
                averaged_depth = image[y_avg, x_avg]
                print "person at", x_avg, y_avg, "has depth", averaged_depth, "and size", person_region.size / float(CAMERA_AREA)
                self.depth = averaged_depth
        else:
            print "no person"

    def update_prediction(self):
        # 

    def bounding_box_cb(self, data):
        """Callback for messages from yolo. Sets self.person_bounding_box to be the largest / best matching bounding box"""
        best_match = None
        best_intersect = 0.0 
        person_bounding_boxes = [x for x in data.bounding_boxes if x.Class == "person" and x.probability > 0.6 and size_of_bounding_box(x) / float(CAMERA_AREA) > 0.05]
        if self.bounding_box == None and person_bounding_boxes:
            best_intersect = 1
            # find the biggest person bounding box and follow that
            best_match = max(person_bounding_boxes, key=size_of_bounding_box)
        else:
            # if there is a person we're tracking
            # find best-matching bounding box
            for new_bounding_box in person_bounding_boxes:
                size = size_of_bounding_box(new_bounding_box)
                intersect = area(new_bounding_box, self.bounding_box)
                if intersect > best_intersect:
                    best_match = new_bounding_box
                    best_intersect = intersect
        if best_match is not None:
            p = self.bounding_box
            prev_area = best_intersect # whatever
            if p is not None:
                prev_area = (p.ymax - p.ymin) * (p.xmax - p.xmin)
        else:
            # if there's no bounding box, we should update our prediction so that the prediction can be tracked instead
            self.update_prediction()
        self.bounding_box = best_match

class PersonFollower:
    def __init__(self):

        # state from subscribers
        self.tracker = PersonTracker()
        self.right_bumper = False
        # node setup
        rospy.init_node("person_follower", anonymous=True)
        # subscriber setup
        sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.tracker.bounding_box_cb)
        depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.tracker.depth_cb)
	joy_sub = rospy.Subscriber('/vesc/joy', Joy, self.right_bump)
        # publisher setup
	pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        seq = 0
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            seq += 1
            header = Header(seq=seq, stamp=rospy.Time.now())
            angle, speed = self.get_steering_direction()
            if self.right_bumper and angle and speed:
                pub.publish(AckermannDriveStamped(header, AckermannDrive(steering_angle=angle, speed=speed)))
            rate.sleep()
        rospy.spin()

    def get_steering_direction(self):
        # main planning / control code!
        # relies on having a bounding box in self.tracker.bounding_box
        SERVO_RANGE = 0.34
        MAX_DEPTH = 1750
        MIN_DEPTH = 1500

        angle = None
        speed = None
        bb = self.tracker.bounding_box
        if bb is not None:
            speed = 0
            boxCenter = (bb.xmin + bb.xmax) / 2.0
            ratio = boxCenter / float(CAMERA_WIDTH)
            angle = -1 * (-SERVO_RANGE + ratio * 2 * SERVO_RANGE)
            if 0 < self.tracker.depth < MIN_DEPTH:
                speed = -0.5
                angle = -angle
            elif self.tracker.depth > MAX_DEPTH:
                speed = 0.5
        return angle, speed

 
    def right_bump(self, data):
	self.right_bumper = bool(data.buttons[5])
        
def area(a, b):
    dx = min(a.xmax, b.xmax) - max(a.xmin, b.xmin)
    dy = min(a.ymax, b.ymax) - max(a.ymin, b.ymin)
    if (dx >= 0) and (dy >= 0):
        return dx*dy

def center_of_bounding_box(bounding_box):
    return np.array([(bounding_box.xmax + bounding_box.xmin)/2.0,
            (bounding_box.ymax + bounding_box.ymin)/2.0])

def size_of_bounding_box(bounding_box):
    return abs(bounding_box.xmax - bounding_box.xmin) * abs(bounding_box.ymax - bounding_box.ymin)

if __name__ == "__main__":
    pf = PersonFollower()
