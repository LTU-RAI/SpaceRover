#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import random
import time

class PathPublisher:
    def __init__(self):
        rospy.init_node('path_publisher', anonymous=False)
        self.publisher_ = rospy.Publisher('/dsp/path', Path, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz

    def random_path(self):
        # Example path: straight forward dots
        path_straight_forward = [[0,0],[1,0],[2,0],[3,0]]
        return path_straight_forward

    def publish_path(self):
        while not rospy.is_shutdown():
            path_random = self.random_path()
            # Scale values (optional)
            path_random = np.multiply(path_random, 2).tolist()

            msg = Path()
            msg.header.frame_id = "robot/odom"
            msg.header.stamp = rospy.Time.now()

            for point in path_random:
                pose = PoseStamped()
                pose.header.frame_id = "robot/odom"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = 0.0
                msg.poses.append(pose)

            self.publisher_.publish(msg)
            rospy.loginfo(f"Published path with {len(path_random)} points")
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = PathPublisher()
        node.publish_path()
    except rospy.ROSInterruptException:
        pass
