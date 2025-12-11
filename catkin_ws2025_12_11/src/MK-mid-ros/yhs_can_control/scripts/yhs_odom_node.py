#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from yhs_can_msgs.msg import odo_fb


class YhsOdomNode(object):
    def __init__(self):
        # ===== 参数 =====
        # world/odom 坐标系
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        # 车体坐标系
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # 输入输出 topic
        self.input_topic = rospy.get_param("~input_topic", "/odo_fb")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")

        # 是否发布 TF(odom_frame -> base_frame)
        self.publish_tf = rospy.get_param("~publish_tf", True)

        # 防止 CAN 重启/丢帧导致的跳变
        self.max_delta_s = rospy.get_param("~max_delta_s", 1.0)            # 单帧最大位移(m)
        self.max_delta_ang_deg = rospy.get_param("~max_delta_ang_deg", 45.0)  # 单帧最大角变化(deg)

        # ===== 状态量 =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # rad

        self.last_mileage = None     # 上一帧累计里程(m)
        self.last_angle_deg = None   # 上一帧累计角度(deg)
        self.last_time = None

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=50)
        self.br = tf.TransformBroadcaster() if self.publish_tf else None

        rospy.Subscriber(self.input_topic, odo_fb, self.odo_cb, queue_size=50)

        rospy.loginfo("yhs_odom_node started. input: %s, output: %s, odom_frame: %s, base_frame: %s",
                      self.input_topic, self.odom_topic,
                      self.odom_frame, self.base_frame)

    @staticmethod
    def normalize_angle(angle):
        """把角度规约到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odo_cb(self, msg):
        # 实车：记得 /use_sim_time=false，这样 now() 是系统时间，不是 0 秒
        now = rospy.Time.now()

        # 累计量：m & deg
        s = msg.odo_fb_accumulative_mileage
        ang_deg = msg.odo_fb_accumulative_angular

        # 第一帧只初始化
        if self.last_mileage is None:
            self.last_mileage = s
            self.last_angle_deg = ang_deg
            self.last_time = now
            return

        # 差分
        ds = s - self.last_mileage
        d_ang_deg = ang_deg - self.last_angle_deg

        # 角度回绕修正（比如 359°→1° 时，d_ang 应该是 -2° 而不是 -358°）
        if d_ang_deg > 180.0:
            d_ang_deg -= 360.0
        elif d_ang_deg < -180.0:
            d_ang_deg += 360.0

        dt = (now - self.last_time).to_sec()
        if dt <= 0.0:
            dt = 1e-3

        # 跳变保护
        if abs(ds) > self.max_delta_s or abs(d_ang_deg) > self.max_delta_ang_deg:
            rospy.logwarn_throttle(
                1.0,
                "yhs_odom_node: abnormal delta dropped: ds=%.3f m, d_ang=%.3f deg",
                ds, d_ang_deg
            )
            self.last_mileage = s
            self.last_angle_deg = ang_deg
            self.last_time = now
            return

        # deg -> rad
        dtheta = math.radians(d_ang_deg)

        # 更新 yaw
        self.yaw = self.normalize_angle(self.yaw + dtheta)

        # 在车体坐标下只向前走 ds（不考虑侧滑）
        dx_body = ds
        dy_body = 0.0

        # 转到 world/odom 坐标系
        dx = math.cos(self.yaw) * dx_body - math.sin(self.yaw) * dy_body
        dy = math.sin(self.yaw) * dx_body + math.cos(self.yaw) * dy_body

        self.x += dx
        self.y += dy

        vx = ds / dt
        vyaw = dtheta / dt

        # ===== 发布 nav_msgs/Odometry =====
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = vyaw

        self.odom_pub.publish(odom)

        # 可选 TF：odom_frame -> base_frame
        if self.br is not None:
            self.br.sendTransform(
                (self.x, self.y, 0.0),
                q,
                now,
                self.base_frame,
                self.odom_frame
            )

        # 更新前一帧
        self.last_mileage = s
        self.last_angle_deg = ang_deg
        self.last_time = now


if __name__ == "__main__":
    rospy.init_node("yhs_odom_node")
    node = YhsOdomNode()
    rospy.spin()
