#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool


def normalize_angle(angle):
    """归一化到 [-pi, pi] 区间"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def is_goal_reached(robot_x, robot_y, robot_yaw,
                    goal_x, goal_y, goal_yaw,
                    xy_tol=0.05, yaw_tol_deg=5.0):
    """
    到点判定：位置和姿态都在容差之内则认为到达
    """
    dx = goal_x - robot_x
    dy = goal_y - robot_y
    dist = math.hypot(dx, dy)

    yaw_tol = math.radians(yaw_tol_deg)
    dyaw = normalize_angle(goal_yaw - robot_yaw)

    return (dist < xy_tol) and (abs(dyaw) < yaw_tol)


class GoalStopper(object):
    """
    外层“到点就停”过滤节点（以 RViz 的 2D Nav Goal 为终点）

    - 机器人位姿：通过 TF 获取 base_frame 在 global_frame 下的姿态
    - 终点：RViz /move_base_simple/goal
    - 速度：输入 cmd_in_topic（例如 /raw_cmd_vel_mppi），输出 cmd_out_topic（例如 /raw_cmd_vel）
    """

    def __init__(self):
        rospy.init_node('goal_stopper')

        # ====== 参数 ======
        self.xy_tol = rospy.get_param('~xy_tol', 0.05)          # 位置容差 [m]
        self.yaw_tol_deg = rospy.get_param('~yaw_tol_deg', 5.0) # 姿态容差 [deg]

        # 坐标系：和 launch 里的 global_frame / base_frame 对应
        self.global_frame = rospy.get_param('~global_frame', '2d_map')
        self.base_frame   = rospy.get_param('~base_frame',   'base_footprint')

        # 2D Nav Goal & 速度话题
        self.goal_topic   = rospy.get_param('~goal_topic',   '/move_base_simple/goal')
        self.cmd_in_topic = rospy.get_param('~cmd_in_topic', '/raw_cmd_vel_mppi')
        self.cmd_out_topic= rospy.get_param('~cmd_out_topic','/raw_cmd_vel')

        # 定时器检测频率（Hz），<=0 则关闭定时器，仅依赖速度回调触发
        self.timer_rate_hz = rospy.get_param('~timer_rate_hz', 50.0)

        # ====== 状态变量 ======
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        self.goal_reached = False

        # TF 监听器：用来拿 base_frame 在 global_frame 下的姿态
        self.tf_listener = tf.TransformListener()

        # ====== 发布器 ======
        self.cmd_pub = rospy.Publisher(self.cmd_out_topic, Twist, queue_size=10)
        self.goal_reached_pub = rospy.Publisher('goal_reached', Bool,
                                                queue_size=1, latch=True)
        self.goal_reached_pub.publish(Bool(data=False))

        # ====== 订阅器 ======
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber(self.cmd_in_topic, Twist, self.cmd_in_cb, queue_size=10)

        # ====== 定时器（可选的高频检测） ======
        if self.timer_rate_hz > 0.0:
            period = 1.0 / self.timer_rate_hz
            self.timer = rospy.Timer(rospy.Duration(period), self.timer_cb)
        else:
            self.timer = None

        rospy.loginfo(
            '[goal_stopper] started\n'
            '  global_frame   = %s\n'
            '  base_frame     = %s\n'
            '  goal_topic     = %s\n'
            '  cmd_in_topic   = %s\n'
            '  cmd_out_topic  = %s\n'
            '  xy_tol=%.3f yaw_tol_deg=%.1f timer_rate=%.1fHz',
            self.global_frame, self.base_frame,
            self.goal_topic, self.cmd_in_topic, self.cmd_out_topic,
            self.xy_tol, self.yaw_tol_deg, self.timer_rate_hz
        )

    # ============= TF 获取 base_footprint 在 global_frame 下的位姿 =============

    def update_robot_pose_from_tf(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.global_frame, self.base_frame, rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

        self.robot_x = trans[0]
        self.robot_y = trans[1]

        _, _, yaw = tf.transformations.euler_from_quaternion(rot)
        self.robot_yaw = yaw

        return True

    # ==================== RViz 2D Nav Goal 回调 ====================

    def goal_cb(self, msg):
        # 如果 goal 的 frame 不是 global_frame，就转换过去
        if msg.header.frame_id != self.global_frame:
            try:
                msg = self.tf_listener.transformPose(self.global_frame, msg)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn('[goal_stopper] failed to transform goal from %s to %s: %s',
                              msg.header.frame_id, self.global_frame, str(e))
                return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        q = msg.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.goal_yaw = yaw

        # 新目标：重置状态
        self.goal_reached = False
        self.goal_reached_pub.publish(Bool(data=False))

        rospy.loginfo(
            '[goal_stopper] new 2D Nav Goal, goal=(%.3f, %.3f), yaw=%.1fdeg (frame=%s)',
            self.goal_x, self.goal_y, math.degrees(self.goal_yaw), self.global_frame
        )

    # ==================== MPPI 输出速度回调 ====================

    def cmd_in_cb(self, msg):
        # 每来一帧速度前先用 TF 更新一下 base_footprint 位姿，再检查是否到点
        if self.goal_x is not None:
            if self.update_robot_pose_from_tf():
                self.check_goal()

        if not self.goal_reached:
            self.cmd_pub.publish(msg)
        else:
            zero = Twist()
            self.cmd_pub.publish(zero)

    # ==================== 定时器：高频补检（可选） ====================

    def timer_cb(self, event):
        if self.goal_x is None:
            return
        if not self.update_robot_pose_from_tf():
            return
        self.check_goal()

    # ==================== 核心：base_footprint vs 2D Nav Goal ====================

    def check_goal(self):
        if self.goal_reached:
            return
        if self.goal_x is None:
            return

        if is_goal_reached(self.robot_x, self.robot_y, self.robot_yaw,
                           self.goal_x, self.goal_y, self.goal_yaw,
                           self.xy_tol, self.yaw_tol_deg):
            self.goal_reached = True

            rospy.loginfo(
                '[goal_stopper] GOAL 到达: (%.3f, %.3f), yaw=%.1fdeg',
                self.goal_x, self.goal_y, math.degrees(self.goal_yaw)
            )

            self.goal_reached_pub.publish(Bool(data=True))

            zero = Twist()
            self.cmd_pub.publish(zero)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = GoalStopper()
    node.spin()
