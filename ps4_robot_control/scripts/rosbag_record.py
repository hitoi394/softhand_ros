#!/usr/bin/env python3
import rospy
from rqt_bag.recorder import Recorder

if __name__ == '__main__':
    rospy.init_node("rosbag_recorder")
    if rospy.has_param('~filename'):
        FILENAME = str(rospy.get_param('~filename'))
        DIRECTORY="/home/hiromasa/rosbag/"
        rec_2 = Recorder(DIRECTORY + FILENAME + ".bag", 
                        topics=['/myrobot/left_camera/aligned_depth_to_color/image_raw', 
                                '/myrobot/left_camera/camera/color/image_raw', 
                                '/myrobot/side_camera/aligned_depth_to_color/image_raw', 
                                '/myrobot/side_camera/camera/color/image_raw', 
                                '/myrobot/front_camera/aligned_depth_to_color/image_raw', 
                                '/myrobot/front_camera/camera/color/image_raw', 
                                '/cmd_vel/left', 
                                '/hand_angle', 
                                '/hand_ref_pressure', 
                                '/joy', 
                                '/joint_states', 
                                '/tf', 
                                '/tf_static'], 
                        all=False)
        rec_2.start()
        input('Press any key to stop recording!')
        rec_2.stop()
    else:
        rospy.logwarn("no filename")