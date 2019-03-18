#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Header
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Joy, Image
import numpy as np
import message_filters
import cv2

br = CvBridge()

CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_AREA = CAMERA_WIDTH * CAMERA_HEIGHT
CONFIDENCE_DECAY_TIME = 5.0 # time, in seconds, before confidence hits zero (after not receiving any new samples)

class PersonTracker:
    def __init__(self):
        # Real data
        self.bounding_boxes = [(rospy.get_time(), None)]
        self.depths = [(rospy.get_time(), None)]

        # Predictions
        self.predicted_bounding_box = None
        self.predicted_depth = None
        self.prediction_confidence = 0
        self.tracker_pub = rospy.Publisher('/personfollower/tracker_box', BoundingBox, queue_size=10)
        self.average_img_pub = rospy.Publisher('/personfollower/average_tracked', Image, queue_size=10)

        # Average of tracked images, for template matching
        self.average_tracked = None
    def img_cb(self, sub, img):
        image = br.imgmsg_to_cv2(img)
        # No average tracked box
        if self.average_tracked is None:
            if self.predicted_bounding_box:
                bb = self.predicted_bounding_box 
                person_region = image[bb.ymin:bb.ymax, bb.xmin:bb.xmax]
                self.average_tracked = cv2.resize(person_region, (64,128)) 
        else:
            if self.predicted_bounding_box:
                bb = self.predicted_bounding_box
                person_region = image[bb.ymin:bb.ymax, bb.xmin:bb.xmax]
                
                if person_region.size < 4:
                    return
                resized = cv2.resize(person_region, (64,128))
                
                similarity = (1 - np.mean(np.abs(self.average_tracked - resized))/255.0)
                print similarity
                
                if img.header.seq % 100 == 0:
                    cv2.imwrite('average.png', self.average_tracked) 
                self.average_tracked = self.average_tracked*0.9+0.1*resized


    def depth_cb(self, data):
        """Callback for messages from the depth camera. Relies on self.person_bounding_box and sets self.person_depth"""
        averaged_depth = 0.0
        image = br.imgmsg_to_cv2(data)
        timestamp, bb = self.bounding_boxes[-1]
        if bb is not None:
            person_region = image[bb.ymin:bb.ymax, bb.xmin:bb.xmax]
            # averaged_depth = np.mean(person_region)
            x_avg = (bb.xmin + bb.xmax) / 2
            y_avg = (bb.ymin + bb.ymax) / 2
            # NOTE: this could be replaced with a smarter weighted average
            # (with depths in the center of the bounding box higher-weighted)
            # only update depth if we can actually see them
            if 0 <= x_avg < CAMERA_WIDTH and 0 < y_avg <= CAMERA_HEIGHT:
                averaged_depth = image[y_avg, x_avg]
                #print "person at", x_avg, y_avg, "has depth", averaged_depth, "and size", person_region.size / float(CAMERA_AREA)
                # self.depth = averaged_depth
                self.depths.append((rospy.get_time(), averaged_depth))
                self.update_prediction()
                if len(self.depths) > 10:
                    self.depths.pop(0)
        else:
            print "no person"

    def compute_predicted_depth():
        first_sample = None
        last_sample = None
        # figure out first and last "real" depth samples
        for sample in self.depths:
            if sample[1] is not None:
                if first_sample is None:
                    first_sample = sample
                last_sample = sample
        if first_sample is None or last_sample is None:
            self.prediction_confidence = 0
        else:
            # figure out average depth change per unit time
            depth_vel = (last_sample[1] - first_sample[1]) / (last_sample[0] - first_sample[0])
            # add this to the most recent sample to get a prediction
            time_since_last_sample = rospy.get_time() - last_sample[0]
            self.predicted_depth = last_sample[1] + depth_vel * time_since_last_sample
            self.prediction_confidence = max(0, 1 - time_since_last_sample / CONFIDENCE_DECAY_TIME)

    def compute_predicted_bounding_box():
        first_sample = None
        last_sample = None
        # figure out first and last "real" bounding_box samples
        for sample in self.bounding_boxes:
            if sample[1] is not None:
                if first_sample is None:
                    first_sample = sample
                last_sample = sample
        if first_sample is None or last_sample is None:
            self.prediction_confidence = 0
        else:
            # figure out average bounding_box change per unit time
            bounding_box_vel = (last_sample[1] - first_sample[1]) / (last_sample[0] - first_sample[0])
            # add this to the most recent sample to get a prediction
            time_since_last_sample = rospy.get_time() - last_sample[0]
            self.predicted_bounding_box = last_sample[1] + depth_vel * time_since_last_sample
            self.prediction_confidence = max(0, 1 - time_since_last_sample / CONFIDENCE_DECAY_TIME)

    def update_prediction(self):
        # TODO: replace with something smarter, maybe?
        _, depth = self.depths[-1]
        #if depth is None:
        #    depth = self.compute_predicted_depth()
        self.predicted_depth = depth
        _, bb = self.bounding_boxes[-1]
        #if bb is None:
        #    bb = self.compute_predicted_bb()
        self.predicted_bounding_box = bb

    def bounding_box_cb(self, data):
        """Callback for messages from yolo. Sets self.person_bounding_box to be the largest / best matching bounding box"""
        best_match = None
        best_intersect = 0.0 
        person_bounding_boxes = [x for x in data.bounding_boxes if x.Class == "person" and x.probability > 0.7 and size_of_bounding_box(x) / float(CAMERA_AREA) > 0.05]
        if self.predicted_bounding_box == None and person_bounding_boxes:
            best_intersect = 1
            # find the biggest person bounding box and follow that
            best_match = max(person_bounding_boxes, key=size_of_bounding_box)
        else:
            # if there is a person we're tracking
            # find best-matching bounding box
            for new_bounding_box in person_bounding_boxes:
                size = size_of_bounding_box(new_bounding_box)
                intersect = area(new_bounding_box, self.predicted_bounding_box)
                if intersect > best_intersect:
                    best_match = new_bounding_box
                    best_intersect = intersect
        if best_match is not None:
            p = self.predicted_bounding_box
            prev_area = best_intersect # whatever
            if p is not None:
                prev_area = (p.ymax - p.ymin) * (p.xmax - p.xmin)
        self.bounding_boxes.append((rospy.get_time(), best_match))
        self.update_prediction()
        if len(self.bounding_boxes) > 10:
            self.bounding_boxes.pop(0)

class PersonFollower:
    def __init__(self):
        # node setup
        rospy.init_node("person_follower", anonymous=True)
        # state from subscribers
        self.tracker = PersonTracker()
        self.right_bumper = False

        self.last_bounding_box_timestamp = 0
        self.last_speed = 0
        self.last_angle = 0
        # subscriber setup
        sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
        depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.tracker.depth_cb)
	joy_sub = rospy.Subscriber('/vesc/joy', Joy, self.right_bump)
        
        img_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        ts = message_filters.ApproximateTimeSynchronizer([sub, img_sub], 10, 0.1)
        sub.registerCallback(self.tracker.bounding_box_cb)
        ts.registerCallback(self.tracker.img_cb)
        # publisher setup
	pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        seq = 0
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            seq += 1
            header = Header(seq=seq, stamp=rospy.Time.now())
            if self.tracker.predicted_bounding_box is not None:
                self.tracker.tracker_pub.publish(self.tracker.predicted_bounding_box)
            if self.tracker.average_tracked is not None:
                self.tracker.average_img_pub.publish(br.cv2_to_imgmsg(self.tracker.average_tracked))
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
        bb = self.tracker.predicted_bounding_box
        depth = self.tracker.predicted_depth
        if bb is None:
            time_elapsed = rospy.get_time() - self.last_bounding_box_timestamp 
            # up to 5 seconds
            if time_elapsed < CONFIDENCE_DECAY_TIME:
                angle = self.last_angle
                speed = self.last_speed
        else:
            self.last_bounding_box_timestamp = rospy.get_time()
            speed = 0
            boxCenter = (bb.xmin + bb.xmax) / 2.0
            ratio = boxCenter / float(CAMERA_WIDTH)
            angle = -1 * (-SERVO_RANGE + ratio * 2 * SERVO_RANGE)
            if 0 < depth < MIN_DEPTH:
                speed = -0.6
                angle = -angle
            elif depth > MAX_DEPTH:
                speed = 0.55
        self.last_angle = angle
        self.last_speed = speed
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
