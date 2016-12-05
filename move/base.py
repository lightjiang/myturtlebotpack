# coding=utf-8
"""
light
2016.12.02
turtlebot motion control
"""
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *


class MotionHandler(object):
    def __init__(self):
        rospy.init_node('motion_base_control', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'
        self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
        self.base_frame = '/base_footprint'

    def goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 1.0  # 3 meters
        goal.target_pose.pose.orientation.w = 1.0  # go forward
        self.move_base.send_goal(goal)

    def test(self):
        self.move(x=0.1, angle=0)#-math.pi/2)
        #self.goal()

    def move(self, x=0.0, angle=0.0, hz=10, speed=0.2, rotate=1.0):
        """
        :param x: 前进或后退距离
        :param angle: 旋转角度  正为左转
        :param hz: 指令发送频率
        :param speed: 行进速度
        :param rotate: 旋转速度
        :return:
        """
        move_cmd = Twist()
        r = rospy.Rate(hz)
        (position, rotation) = self.get_odom()
        last_x = position.x
        last_y = position.y
        last_angle = rotation
        if x:
            move_cmd.linear.x = speed * x / abs(x)
        if angle:
            move_cmd.angular.z = rotate * angle / abs(angle)
        stat = 1
        turn_angle = 0
        distance = 0
        while not rospy.is_shutdown() and stat:
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            (position, rotation) = self.get_odom()
            delta_angle = normalize_angle(rotation - last_angle)
            turn_angle += delta_angle
            last_angle = rotation
            distance += math.sqrt(pow((position.x - last_x), 2) + pow((position.y - last_y), 2))
            last_x = position.x
            last_y = position.y
            print(distance, turn_angle)
            if distance >= abs(x):
                move_cmd.linear.x = 0.0
            if abs(turn_angle) >= abs(angle):
                move_cmd.angular.z = 0.0
            if not (move_cmd.linear.x or move_cmd.angular.z):
                stat = 0

    def get_odom(self):
        (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        return Point(*trans), quat_to_angle(Quaternion(*rot))

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    handler = MotionHandler()
    handler.test()