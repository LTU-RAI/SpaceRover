#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import math

class PathSubscriber:
    def __init__(self):
        rospy.init_node('path_subscriber', anonymous=False)

        # Subscriber for path
        self.latest_path = []
        rospy.Subscriber('/dsp/path', Path, self.path_callback)

        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Robot state
        self.current_position = [0.0, 0.0]
        self.threshold = 0.10

        # Rate of control loop
        self.rate = rospy.Rate(10)  # 10 Hz

        # Start following path
        self.follow_path_loop()

    def compute_twist(self, curr, dest):
        dx = dest[0] - curr[0]
        dy = dest[1] - curr[1]
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)

        twist = Twist()
        twist.linear.x = min(0.2, distance)
        twist.angular.z = 0.0
        return twist, distance

    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
    def follow_path_loop(self):
        while not rospy.is_shutdown():
            if not self.latest_path:
                self.stop()
                self.rate.sleep()
                continue

            wp = self.latest_path[0]
            twist, distance = self.compute_twist(self.current_position, wp)

            if distance < self.threshold:
                self.latest_path.pop(0)
                twist = Twist()
                rospy.loginfo(f"Reached target waypoint: {wp}")

            self.cmd_pub.publish(twist)

            # Simulate movement (replace with odometry)
            self.current_position[0] += twist.linear.x * 0.1 * math.cos(twist.angular.z)
            self.current_position[1] += twist.linear.x * 0.1 * math.sin(twist.angular.z)

            self.rate.sleep()
            
            if not self.latest_path:
                self.stop()
                rospy.loginfo(f"Waypoint exhausted, Stopping robot")
                break

    def path_callback(self, msg):
        self.latest_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        rospy.loginfo(f"Received {len(self.latest_path)} waypoints")
        rospy.loginfo(f"{self.latest_path}")

if __name__ == "__main__":
    try:
        PathSubscriber()
    except rospy.ROSInterruptException:
        pass
