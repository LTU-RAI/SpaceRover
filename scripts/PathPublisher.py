#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPublisher:
    def __init__(self):
        rospy.init_node('path_publisher', anonymous=False)
        self.publisher = rospy.Publisher(
            '/dsp/path',
            Path,
            queue_size=1,
            latch=True
        )

        rospy.sleep(0.5)
        self.publish_path()
        rospy.spin()

    def publish_path(self):
        path = [[2.0, 2.0]]

        msg = Path()
        msg.header.frame_id = "robot/odom"
        msg.header.stamp = rospy.Time.now()

        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = "robot/odom"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            msg.poses.append(pose)

        self.publisher.publish(msg)
        rospy.loginfo(f"Published path once: {path}")

if __name__ == '__main__':
    try:
        PathPublisher()
    except rospy.ROSInterruptException:
        pass
