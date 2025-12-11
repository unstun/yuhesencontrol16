#!/usr/bin/env python
# -*- coding: utf-8 -*-

# FILENAME: ~/catkin_ws/src/sentry_kinematics_bridge/scripts/twist_to_yhs_ctrl.py
#
# 功能（carlike 版本）：
# 1. 订阅 /cmd_vel (geometry_msgs/Twist)
# 2. 假设：
#       linear.x  = 车体纵向速度 v   [m/s]
#       angular.z = 前轮转角 steering [rad] ！！！不是角速度！！！
# 3. 将速度和转角转换为 yhs_can_msgs/ctrl_cmd：
#       ctrl_cmd_velocity  [m/s]，取绝对值
#       ctrl_cmd_steering  [deg]，左转为正，右转为负（根据你原来习惯）
# 4. 根据速度符号设置挡位和刹车

import rospy
import math
from geometry_msgs.msg import Twist
from yhs_can_msgs.msg import ctrl_cmd  # 自定义底盘控制消息

# --- 全局变量 ---
pub = None  # Publisher
# -----------------


def cmd_callback(data):
    """
    /cmd_vel 话题回调：
    - linear.x  : 线速度 v (m/s)
    - angular.z : 转向角 steering (rad)，由 mpc_local_planner/simple_car 输出
    """
    global pub

    # 1. 从 /cmd_vel 获取速度和转角
    v = data.linear.x              # m/s
    steering_rad = data.angular.z  # rad，simple_car 的转角
    steering_deg = math.degrees(steering_rad)  # 转成角度，方便给底盘

    # --- 创建 yhs_can_msgs/ctrl_cmd 消息 ---
    msg = ctrl_cmd()

    # 2. 根据速度正负设置挡位、刹车和速度
    if v > 0.05:          # 前进
        msg.ctrl_cmd_gear = 4     # 你原来的前进档
        msg.ctrl_cmd_Brake = 0    # 松刹车
        msg.ctrl_cmd_velocity = v # 保持 m/s

        # 前进时，左转为正，右转为负（保持你原来的方向）
        msg.ctrl_cmd_steering = steering_deg

    elif v < -0.05:       # 倒车
        msg.ctrl_cmd_gear = 2     # 你原来的倒档
        msg.ctrl_cmd_Brake = 0
        msg.ctrl_cmd_velocity = -v  # 底盘一般用正值表示速度大小

        # 保持你原来的习惯：倒车时把转向角取反
        # 这样“给一个左转角 +δ”，前进和倒车时车子的运动方向更直观
        msg.ctrl_cmd_steering = -steering_deg

    else:                 # 近似静止
        msg.ctrl_cmd_gear = 4     # 可以保持前进档
        msg.ctrl_cmd_Brake = 1    # 拉住刹车
        msg.ctrl_cmd_velocity = 0.0
        msg.ctrl_cmd_steering = 0.0

    # 3. 发布到底盘控制话题
    if pub is not None:
        pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('twist_to_yhs_ctrl_bridge', anonymous=True)

        # 话题名可以通过参数覆盖，默认 /cmd_vel → /ctrl_cmd
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
        ctrl_cmd_topic = rospy.get_param('~ctrl_cmd_topic', '/ctrl_cmd')

        # 订阅来自 move_base（mpc_local_planner）的 carlike cmd_vel
        rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)

        # 发布给 YHS 底盘
        pub = rospy.Publisher(ctrl_cmd_topic, ctrl_cmd, queue_size=1)

        rospy.loginfo("[%s] 运动学桥接节点已启动（carlike 模式）。", rospy.get_name())
        rospy.loginfo(" > 订阅话题: %s", twist_cmd_topic)
        rospy.loginfo(" > 发布话题: %s", ctrl_cmd_topic)
        rospy.loginfo(" > 约定: cmd_vel.angular.z = 前轮转角 [rad]",)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
