#!/usr/bin/env python3
from genpy import Duration
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        m = geometry_msgs.msg.TransformStamped()
        m.header.stamp = rospy.Time.now()-rospy.Duration(10)
        m.header.frame_id = "parent"
        m.child_frame_id = "child"
        m.transform.translation.x = 1
        m.transform.rotation.w = 1
        tfBuffer.set_transform(m, 'default_authority')
        try:
            trans = tfBuffer.lookup_transform('child', 'parent', rospy.Time.now()-rospy.Duration(11))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            print('err')
            continue

        print(trans.transform)

        rate.sleep()