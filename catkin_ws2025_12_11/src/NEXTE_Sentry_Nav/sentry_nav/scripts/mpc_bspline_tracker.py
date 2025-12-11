#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

import cvxpy as cp


class MPCBsplineTracker(object):
    def __init__(self):
        # --- 参数 ---
        self.L = rospy.get_param("~wheelbase", 1.6)   # 车辆轴距
        self.dt = rospy.get_param("~dt", 0.05)        # 控制周期
        self.N  = rospy.get_param("~horizon", 15)     # MPC 预测步数

        self.v_nom = rospy.get_param("~v_nom", 0.8)
        self.a_max = rospy.get_param("~a_max", 1.0)
        self.delta_max = rospy.get_param("~delta_max", 0.5)  # rad

        # 话题名
        path_topic = rospy.get_param("~path_topic", "/move_base_rmp/bspline_plan")
        odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        cmd_topic  = rospy.get_param("~cmd_topic",  "/cmd_vel")

        # 订阅
        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

        # 发布
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)

        # 状态缓存
        self.path_xy = None    # np.array shape (M, 2)
        self.path_yaw = None   # np.array shape (M,)
        self.path_s = None     # 弧长
        self.path_t = None     # 时间
        self.v_ref_profile = None

        self.x = None  # 当前 [x, y, yaw, v]

        # MPC 变量预分配（cvxpy）
        nx = 4
        nu = 2
        self.x_var = cp.Variable((nx, self.N+1))
        self.u_var = cp.Variable((nu, self.N))

        # 权重矩阵
        self.Q = np.diag([1,1,1,1])
        self.R = np.diag([1, 1])
        self.S = np.diag([0.1, 0.1])

        # 控制循环定时器
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)

        rospy.loginfo("MPCBsplineTracker initialized")

    # ------------ 回调函数 ------------

    def path_callback(self, msg: Path):
        n = len(msg.poses)
        if n < 4:
            rospy.logwarn("Path too short")
            return

        xs = np.array([p.pose.position.x for p in msg.poses])
        ys = np.array([p.pose.position.y for p in msg.poses])

        # 弧长
        ds = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
        s = np.hstack(([0.0], np.cumsum(ds)))

        # yaw
        yaw = np.arctan2(np.gradient(ys), np.gradient(xs))

        # 时间参数化（恒速版本）
        t = s / max(self.v_nom, 1e-3)
        v_ref = np.full_like(s, self.v_nom)

        self.path_xy = np.stack([xs, ys], axis=1)
        self.path_yaw = yaw
        self.path_s = s
        self.path_t = t
        self.v_ref_profile = v_ref

        rospy.loginfo("Received path with %d points, length=%.2f m", n, s[-1])

    def odom_callback(self, msg: Odometry):
        # 从里程计取当前姿态
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        vx = msg.twist.twist.linear.x

        self.x = np.array([x, y, yaw, vx])

    # ------------ MPC 主循环 ------------

    def control_loop(self, event):
        if self.path_xy is None or self.x is None:
            return

        # 1. 找到当前在路径上的最近点索引
        pos = self.x[0:2]
        d = np.linalg.norm(self.path_xy - pos, axis=1)
        idx0 = int(np.argmin(d))

        # 2. 构造未来 N 步的参考（如果快到终点，就重复终点）
        M = self.path_xy.shape[0]
        idxs = np.clip(idx0 + np.arange(self.N+1), 0, M-1)

        x_ref = self.path_xy[idxs, 0]
        y_ref = self.path_xy[idxs, 1]
        yaw_ref = self.path_yaw[idxs]
        v_ref = self.v_ref_profile[idxs]

        # 3. 线性化 + MPC QP 搭建
        #   为了简化：在这里用“误差状态” e = x - x_ref，构建线性模型 e_{k+1} = A e_k + B u_k
        #   示意：直接用一个简单的近似 A,B，你可以自己换成更严谨的离散化模型
        v0 = max(self.x[3], 0.1)
        A = np.eye(4)
        A[0,2] = -v0 * np.sin(self.x[2]) * self.dt
        A[0,3] =  np.cos(self.x[2]) * self.dt
        A[1,2] =  v0 * np.cos(self.x[2]) * self.dt
        A[1,3] =  np.sin(self.x[2]) * self.dt
        A[2,3] =  np.tan(0.0) / self.L * self.dt  # 这里偷懒了，你可以按参考转角线性化
        A[3,3] = 1.0

        B = np.zeros((4,2))
        # u = [a, delta]
        B[2,1] = v0 * self.dt / (self.L * np.cos(0.0)**2 + 1e-6)  # 对 delta 的线性化
        B[3,0] = self.dt  # 对加速度

        # 4. 搭建优化问题
        x_var = self.x_var
        u_var = self.u_var

        constraints = []
        cost_terms = []

        # 初始误差
        e0 = self.x - np.array([x_ref[0], y_ref[0], yaw_ref[0], v_ref[0]])
        constraints += [x_var[:,0] == e0]

        for k in range(self.N):
            # 动态约束 e_{k+1} = A e_k + B u_k
            constraints += [x_var[:,k+1] == A @ x_var[:,k] + B @ u_var[:,k]]

            # 控制约束
            constraints += [
                cp.abs(u_var[0,k]) <= self.a_max,
                cp.abs(u_var[1,k]) <= self.delta_max
            ]

            # 代价：误差 + 控制
            cost_terms.append(cp.quad_form(x_var[:,k], self.Q))
            cost_terms.append(cp.quad_form(u_var[:,k], self.R))

            if k < self.N-1:
                cost_terms.append(cp.quad_form(u_var[:,k+1] - u_var[:,k], self.S))

        # 终端误差
        cost_terms.append(cp.quad_form(x_var[:,self.N], self.Q))

        cost = cp.sum(cost_terms)

        prob = cp.Problem(cp.Minimize(cost), constraints)

        try:
            prob.solve(solver=cp.OSQP, warm_start=True, max_iter=50)
        except Exception as e:
            rospy.logwarn("MPC solve failed: %s", e)
            return

        if prob.status not in ["optimal", "optimal_inaccurate"]:
            rospy.logwarn("MPC status: %s", prob.status)
            return

        # 5. 取第一步控制量 u0 = [a0, delta0]
        a0 = u_var[0,0].value
        delta0 = u_var[1,0].value

        # 6. 转成 /cmd_vel （这里假设底层接收的是 v, omega）
        #   简单做法：v = v_ref[0] + e_v 校正，omega = v / L * tan(delta)
        v_cmd = float(np.clip(v_ref[0] + x_var[3,0].value, 0.0, self.v_nom))
        omega_cmd = v_cmd * np.tan(float(delta0)) / self.L

        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = omega_cmd

        self.cmd_pub.publish(twist)


def main():
    rospy.init_node("mpc_bspline_tracker")
    tracker = MPCBsplineTracker()
    rospy.spin()


if __name__ == "__main__":
    main()
