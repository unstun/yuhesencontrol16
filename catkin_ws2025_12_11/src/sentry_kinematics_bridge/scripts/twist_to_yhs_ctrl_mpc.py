#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from yhs_can_msgs.msg import ctrl_cmd


class TwistToYhsCtrlMPC(object):
    def __init__(self):
        # 车辆几何参数
        self.wheelbase = rospy.get_param("~wheelbase", 0.6)  # 轴距 [m]
        self.max_speed = rospy.get_param("~max_speed", 1.0)  # 最大车速 [m/s]

        # 最大前轮转角 [deg] —— 注意这里是度，不是 rad
        self.max_steering_angle_deg = rospy.get_param("~max_steering_angle_deg", 30.0)

        # 速度死区，小于这个就当 0
        self.speed_deadband = rospy.get_param("~speed_deadband", 0.02)

        # 挡位 / 刹车编码（按你 Yuhesen 手册来，如果不一样就改 param，不用改代码）
        self.forward_gear = rospy.get_param("~forward_gear", 4)   # 前进挡
        self.reverse_gear = rospy.get_param("~reverse_gear", 2)   # 倒车挡
        self.neutral_gear = rospy.get_param("~neutral_gear", 0)   # 空挡
        self.brake_on = rospy.get_param("~brake_on", 1)
        self.brake_off = rospy.get_param("~brake_off", 0)

        # 话题名
        cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        ctrl_cmd_topic = rospy.get_param("~ctrl_cmd_topic", "/ctrl_cmd")

        self.pub_cmd = rospy.Publisher(ctrl_cmd_topic, ctrl_cmd, queue_size=1)
        self.sub_cmd = rospy.Subscriber(cmd_vel_topic, Twist,
                                        self.cmd_callback, queue_size=1)

        rospy.loginfo("TwistToYhsCtrlMPC started. Listen: %s  Publish: %s",
                      cmd_vel_topic, ctrl_cmd_topic)

    @staticmethod
    def _saturate(x, xmin, xmax):
        return max(xmin, min(x, xmax))

    def _yawrate_to_steering_deg(self, v_signed, omega):
        """
        根据 Ackermann 小车模型：
            omega = v / L * tan(delta)
            => delta = atan( L * omega / v )

        v_signed：带符号车速（前进>0，倒退<0）
        omega：偏航角速度 [rad/s]

        返回值：前轮转角 [deg]，满足底盘接口要求（deg）
        """
        if abs(v_signed) < 1e-3 or abs(omega) < 1e-3:
            return 0.0

        # 先算 rad
        delta_rad = math.atan(self.wheelbase * omega / v_signed)
        # 再转成 deg
        delta_deg = math.degrees(delta_rad)

        # 做限幅（deg）
        return self._saturate(delta_deg,
                              -self.max_steering_angle_deg,
                              self.max_steering_angle_deg)

    def cmd_callback(self, msg):
        # MPC 输出：标准 ROS Twist
        v = msg.linear.x          # [m/s]，前进为正，倒退为负
        omega = msg.angular.z     # [rad/s]，左转为正

        out = ctrl_cmd()

        # ---------------- 1. 判定档位 / 速度 ----------------
        if v > self.speed_deadband:
            # 前进
            gear = self.forward_gear
            brake = self.brake_off
            speed = self._saturate(v, 0.0, self.max_speed)

        elif v < -self.speed_deadband:
            # 倒车：速度给绝对值，方向用档位表示
            gear = self.reverse_gear
            brake = self.brake_off
            speed = self._saturate(-v, 0.0, self.max_speed)

        else:
            # 近似静止：空挡 + 刹车
            gear = self.neutral_gear
            brake = self.brake_on
            speed = 0.0
            omega = 0.0  # 不再给转向，避免原地抖

        # 带符号速度：后面算转角要用
        if gear == self.forward_gear:
            v_signed = speed
        elif gear == self.reverse_gear:
            v_signed = -speed
        else:
            v_signed = 0.0

        # ---------------- 2. (v,omega) -> 转角(度) ----------------
        steering_deg = self._yawrate_to_steering_deg(v_signed, omega)

        # ---------------- 3. 填 ctrl_cmd ----------------
        out.ctrl_cmd_gear = int(gear)
        out.ctrl_cmd_Brake = int(brake)
        # 速度：永远给非负数，底盘只看挡位区分前后
        out.ctrl_cmd_velocity = float(speed)
        # 转角：单位“度”，底盘那边会再 *100 发 CAN
        out.ctrl_cmd_steering = float(steering_deg)

        self.pub_cmd.publish(out)


if __name__ == "__main__":
    rospy.init_node("twist_to_yhs_ctrl_mpc")
    node = TwistToYhsCtrlMPC()
    rospy.spin()
