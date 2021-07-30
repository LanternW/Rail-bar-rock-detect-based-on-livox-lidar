#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist
from object_detection_msgs.msg import BoundingBoxes


def bboxCallBack(msg):
    t = Twist()
    t.linear.x = msg.bounding_boxes[0].xmin - 100
    t.linear.y = msg.bounding_boxes[0].ymin - 100
    t.angular.x = msg.bounding_boxes[0].xmax + 100
    t.angular.y = msg.bounding_boxes[0].ymax + 100
    pub.publish(t)



if __name__ == "__main__":
    rospy.init_node("convert_bounding_box",anonymous=True)
    sub = rospy.Subscriber("/yolov5trt/bboxes_pub",BoundingBoxes, bboxCallBack)
    pub = rospy.Publisher("/image_pixel", Twist,queue_size=10)
    rospy.spin()

