#!/usr/bin/env python3
"""
Zero-velocity pose hold for robot_localization.

While the robot is stationary, IMU bias makes the EKF pose estimate creep.
This node watches wheel odometry; once the robot has been still for
`~timer` seconds it latches the current filtered pose and keeps resetting
the EKF to it (every `~pose_correct_timer` seconds) via the set_pose
service. When the robot starts moving again it applies one final reset
and hands control back to the filter.

Subscribes:  odom (nav_msgs/Odometry)                - wheel odometry
             odometry/filtered (nav_msgs/Odometry)   - robot_localization output
Calls:       set_pose (robot_localization/SetPose)
"""
import rospy
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose, SetPoseRequest


class ZeroVelocityPoseHold(object):

    def __init__(self):
        self.still_timeout = rospy.get_param("~timer", 5.0)
        self.correct_period = rospy.get_param("~pose_correct_timer", 10.0)
        set_pose_srv = rospy.get_param("~set_pose_service", "/set_pose")

        self.holding = False                 # True while the pose is latched
        self.is_still = False
        self.last_motion_time = rospy.Time.now()
        self.last_correction_time = rospy.Time.now()
        self.latched = SetPoseRequest()

        rospy.loginfo("Waiting for %s ...", set_pose_srv)
        rospy.wait_for_service(set_pose_srv)
        self.set_pose = rospy.ServiceProxy(set_pose_srv, SetPose)

        rospy.Subscriber("odom", Odometry, self.odom_cb)
        rospy.Subscriber("odometry/filtered", Odometry, self.filtered_cb)
        rospy.loginfo("Pose hold ready: still_timeout=%.1fs, correct_period=%.1fs",
                      self.still_timeout, self.correct_period)

    def apply_correction(self):
        self.latched.pose.header.stamp = rospy.Time.now()
        self.set_pose(self.latched)
        self.last_correction_time = rospy.Time.now()
        rospy.loginfo("Pose corrected")

    def odom_cb(self, msg):
        now = rospy.Time.now()
        self.is_still = (msg.twist.twist.linear.x == 0.0 and msg.twist.twist.angular.z == 0.0)

        if not self.is_still:
            self.last_motion_time = now
            if self.holding:                 # robot started moving: final reset, release
                self.apply_correction()
                self.holding = False
                rospy.loginfo("Motion detected - pose hold released")
            return

        still_for = (now - self.last_motion_time).to_sec()
        if not self.holding and still_for >= self.still_timeout:
            self.holding = True              # latch the most recent filtered pose
            self.last_correction_time = now
            rospy.loginfo("Stationary for %.1fs - pose hold engaged", still_for)
        elif self.holding and (now - self.last_correction_time).to_sec() >= self.correct_period:
            self.apply_correction()

    def filtered_cb(self, msg):
        # Keep updating the candidate pose until the hold is engaged
        if not self.holding and self.is_still:
            self.latched.pose.header.frame_id = msg.header.frame_id or "odom"
            self.latched.pose.pose.pose = msg.pose.pose
            self.latched.pose.pose.covariance = msg.pose.covariance


if __name__ == "__main__":
    rospy.init_node("imu_filter_toggle")
    try:
        ZeroVelocityPoseHold()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
