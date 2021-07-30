#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist
import time


rospy.init_node("test_bounding_box",anonymous=True)

pub = rospy.Publisher("/image_pixel", Twist,queue_size=10)

t = Twist()
#t.linear.x = 768
#t.linear.y = 445
#t.angular.x = 934
#t.angular.y = 699

t.linear.x = 10
t.linear.y = 10
t.angular.x = 1910
t.angular.y = 1070

i = 0
while i < 20:
    pub.publish(t)
    time.sleep(0.2)
    i += 1

