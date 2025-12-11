#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import os
import threading
import math
import datetime

import matplotlib.pyplot as plt
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import tf2_ros


class LivePlotter:
    """
    功能：
      1) 轨迹对比：
         - 实际轨迹（TF: target_frame -> source_frame）
         - A* / Hybrid A* 全局规划轨迹
         - MPC 局部规划 local_plan 轨迹
           - self.mpc_x/self.mpc_y：最新一条局部轨迹（用于实时显示）
           - self.mpc_hist_*：累积整个实验过程中的所有局部轨迹点（方案B）
      2) 速度 & 转角对比：
         - 规定速度/转角（Twist）
         - 实际速度/转角（Odometry + 简单单轨迹车模型近似）

    退出节点时自动保存：
      - 轨迹 CSV + PNG
      - 速度/转角 CSV + PNG
      - MPC 全历史轨迹 CSV（方案B）
    """

    def __init__(self):
        # 1. 初始化节点
        rospy.init_node('live_trajectory_plotter', anonymous=True)

        # 用于文件名的时间戳
        self.start_time = rospy.Time.now()
        self.start_time_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        # 2. 读取参数（可在 launch 里覆盖）
        self.global_plan_topic = rospy.get_param(
            "~global_plan_topic", "/move_base_rmp/HybridAStarPlanner/plan"
        )
        self.mpc_plan_topic = rospy.get_param(
            "~mpc_plan_topic", "/move_base_rmp/MpcLocalPlannerROS/local_plan"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")

        self.target_frame = rospy.get_param("~target_frame", "2d_map")
        self.source_frame = rospy.get_param("~source_frame", "base_footprint")
        self.wheelbase = rospy.get_param("~wheelbase", 1.6)

        # 保存目录
        self.save_dir = os.path.expanduser(
            rospy.get_param("~save_dir", "~/trajectory_data")
        )
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 3. 数据缓存（加锁防止多线程读写冲突）
        self.lock = threading.Lock()

        # 3.1 三条轨迹 + 时间
        self.actual_x = []
        self.actual_y = []
        self.actual_t = []   # 实际轨迹时间

        self.astar_x = []    # A* / Hybrid A* 全局轨迹
        self.astar_y = []
        self.astar_t = []    # A* 全局轨迹时间

        self.mpc_x = []      # 当前 MPC 局部轨迹（最新一条，用于实时显示）
        self.mpc_y = []
        self.mpc_t = []      # 当前 MPC 局部轨迹时间

        # 方案B：累积整个 MPC 历史
        self.mpc_hist_x = []   # 所有历史 MPC 轨迹点（x）
        self.mpc_hist_y = []   # 所有历史 MPC 轨迹点（y）
        self.mpc_hist_t = []   # 所有历史 MPC 轨迹点（时间）

        # 3.2 速度/转角
        self.time_cmd = []
        self.cmd_v = []
        self.cmd_delta = []

        self.time_actual = []
        self.actual_v = []
        self.actual_delta = []

        # 4. TF 监听器，用于实时获取实际位置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 5. Matplotlib 绘图初始化
        plt.ion()

        # 5.1 轨迹对比图
        self.fig_xy, self.ax_xy = plt.subplots(figsize=(8, 8))
        self.ax_xy.set_title("Trajectory: Actual vs A* Global vs MPC Local")
        self.ax_xy.set_xlabel("X [m]")
        self.ax_xy.set_ylabel("Y [m]")
        self.ax_xy.grid(True)
        self.ax_xy.axis('equal')

        # 实际轨迹
        self.line_actual, = self.ax_xy.plot(
            [], [], '-', label='Actual (TF)', linewidth=1.5
        )
        # A* / Hybrid A* 全局轨迹
        self.line_astar, = self.ax_xy.plot(
            [], [], '--', label='Global plan (A*/Hybrid A*)', linewidth=2, alpha=0.8
        )
        # MPC 局部轨迹（当前最新一条）
        self.line_mpc, = self.ax_xy.plot(
            [], [], '-.', label='MPC local plan (latest)', linewidth=1.5, alpha=0.8
        )
        self.ax_xy.legend(loc='upper right')

        # 5.2 速度 & 转角对比图
        self.fig_vs, (self.ax_v, self.ax_delta) = plt.subplots(
            2, 1, sharex=True, figsize=(10, 6)
        )
        # 速度
        self.ax_v.set_title("Speed comparison")
        self.ax_v.set_ylabel("v [m/s]")
        self.ax_v.grid(True)
        self.line_v_cmd, = self.ax_v.plot([], [], label='v_cmd')
        self.line_v_actual, = self.ax_v.plot([], [], label='v_actual')
        self.ax_v.legend(loc='upper right')

        # 转角
        self.ax_delta.set_title("Steering angle comparison")
        self.ax_delta.set_ylabel("delta [rad]")
        self.ax_delta.set_xlabel("t [s]")
        self.ax_delta.grid(True)
        self.line_delta_cmd, = self.ax_delta.plot([], [], label='delta_cmd')
        self.line_delta_actual, = self.ax_delta.plot([], [], label='delta_actual')
        self.ax_delta.legend(loc='upper right')

        # 6. 订阅话题
        rospy.Subscriber(self.global_plan_topic, Path, self.global_plan_callback)
        rospy.Subscriber(self.mpc_plan_topic, Path, self.mpc_plan_callback)
        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        rospy.loginfo("========================================")
        rospy.loginfo("live_trajectory_plotter 已启动")
        rospy.loginfo("  Global plan (A*/Hybrid A*): %s", self.global_plan_topic)
        rospy.loginfo("  MPC local plan           : %s", self.mpc_plan_topic)
        rospy.loginfo("  Cmd vel / steering       : %s", self.cmd_topic)
        rospy.loginfo("  Actual vel / steering    : %s", self.odom_topic)
        rospy.loginfo("  Frames                   : %s -> %s",
                      self.target_frame, self.source_frame)
        rospy.loginfo("  Wheelbase                : %.3f m", self.wheelbase)
        rospy.loginfo("  Save dir                 : %s", self.save_dir)
        rospy.loginfo("  File tag (timestamp)     : %s", self.start_time_str)
        rospy.loginfo("========================================")

        # 7. 退出时自动保存数据
        rospy.on_shutdown(self.save_data)

    # ---------- TF: 实际位置 ----------

    def get_current_location(self):
        """通过 TF 获取当前机器人在 target_frame 下的坐标"""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            return (trans.transform.translation.x,
                    trans.transform.translation.y)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None, None

    # ---------- 回调：A* / Hybrid A* 全局轨迹 ----------

    def global_plan_callback(self, msg: Path):
        """
        A* / Hybrid A* 全局规划回调：
        同一目标下，只保留“最长的一条完整路径”，
        并记录每个路径点的时间戳。
        """
        if not msg.poses:
            return

        new_x = []
        new_y = []
        new_t = []

        for p in msg.poses:
            new_x.append(p.pose.position.x)
            new_y.append(p.pose.position.y)

            stamp = p.header.stamp
            if stamp.to_sec() == 0.0:
                stamp = msg.header.stamp
            t = (stamp - self.start_time).to_sec()
            new_t.append(t)

        with self.lock:
            # 第一次：直接记录
            if not self.astar_x:
                self.astar_x = new_x
                self.astar_y = new_y
                self.astar_t = new_t
                return

            # 判断是否是新目标：比较末端点距离
            dx = new_x[-1] - self.astar_x[-1]
            dy = new_y[-1] - self.astar_y[-1]
            dist_end = (dx ** 2 + dy ** 2) ** 0.5

            # 超过 1m 认为目标变了，更新整条路径
            if dist_end > 1.0:
                rospy.loginfo("检测到新的导航目标 (A*/Hybrid A*)，更新全局路径。")
                self.astar_x = new_x
                self.astar_y = new_y
                self.astar_t = new_t
            else:
                # 目标不变，只在“新路径更长”的情况下覆盖，避免路径被剪短
                if len(new_x) > len(self.astar_x):
                    self.astar_x = new_x
                    self.astar_y = new_y
                    self.astar_t = new_t

    # ---------- 回调：MPC 局部轨迹 ----------

    def mpc_plan_callback(self, msg: Path):
        """
        MPC 局部规划 local_plan 回调：
        - self.mpc_x/self.mpc_y/self.mpc_t 只保留“最新一条局部轨迹”（用于实时显示）
        - self.mpc_hist_* 累积整个实验过程中的所有局部轨迹点（方案B）
        """
        if not msg.poses:
            return

        xs = []
        ys = []
        ts = []

        for p in msg.poses:
            xs.append(p.pose.position.x)
            ys.append(p.pose.position.y)

            stamp = p.header.stamp
            if stamp.to_sec() == 0.0:
                stamp = msg.header.stamp
            t = (stamp - self.start_time).to_sec()
            ts.append(t)

        with self.lock:
            # 最新一条
            self.mpc_x = xs
            self.mpc_y = ys
            self.mpc_t = ts

            # 历史累积（方案B）
            self.mpc_hist_x.extend(xs)
            self.mpc_hist_y.extend(ys)
            self.mpc_hist_t.extend(ts)

    # ---------- 回调：规定速度 / 规定转角 ----------

    def cmd_callback(self, msg: Twist):
        """
        规定速度和转角：
          - 线速度 v_cmd = cmd.linear.x
          - 转角   delta_cmd = cmd.angular.z（假定这里就直接存的是前轮转角）
        如果你实际用的是“角速度”，可以在这里自己改成其它计算方式。
        """
        t = (rospy.Time.now() - self.start_time).to_sec()
        v = msg.linear.x
        delta_cmd = msg.angular.z

        with self.lock:
            self.time_cmd.append(t)
            self.cmd_v.append(v)
            self.cmd_delta.append(delta_cmd)

    # ---------- 回调：实际速度 / 实际转角 ----------

    def odom_callback(self, msg: Odometry):
        """
        实际速度来自 /Odometry：
          - v_actual = twist.linear.x
          - yaw_rate = twist.angular.z
        用一个简单单轨迹车模型近似实际转角：
          delta_actual ≈ atan( L * yaw_rate / v )
        注意：在 v 非常小时这个近似会不稳定，这里做了简单保护。
        """
        t = (msg.header.stamp - self.start_time).to_sec()
        v = msg.twist.twist.linear.x
        yaw_rate = msg.twist.twist.angular.z

        if abs(v) < 1e-3:
            delta = 0.0
        else:
            delta = math.atan(self.wheelbase * yaw_rate / max(v, 1e-3))

        with self.lock:
            self.time_actual.append(t)
            self.actual_v.append(v)
            self.actual_delta.append(delta)

    # ---------- 主循环：更新两张图 ----------

    def update_plot(self):
        # 1) 实际位姿（TF）
        cur_x, cur_y = self.get_current_location()

        with self.lock:
            if cur_x is not None:
                t_now = (rospy.Time.now() - self.start_time).to_sec()
                self.actual_x.append(cur_x)
                self.actual_y.append(cur_y)
                self.actual_t.append(t_now)

            ax_data = list(self.actual_x)
            ay_data = list(self.actual_y)
            at_data = list(self.actual_t)

            gx_data = list(self.astar_x)
            gy_data = list(self.astar_y)
            gt_data = list(self.astar_t)

            lx_data = list(self.mpc_x)
            ly_data = list(self.mpc_y)
            lt_data = list(self.mpc_t)

            t_cmd = list(self.time_cmd)
            v_cmd = list(self.cmd_v)
            delta_cmd = list(self.cmd_delta)

            t_act = list(self.time_actual)
            v_act = list(self.actual_v)
            delta_act = list(self.actual_delta)

        # ---------- 1) XY 轨迹对比 ----------
        if ax_data:
            self.line_actual.set_data(ax_data, ay_data)
        if gx_data:
            self.line_astar.set_data(gx_data, gy_data)
        if lx_data:
            self.line_mpc.set_data(lx_data, ly_data)

        all_x = ax_data + gx_data + lx_data
        all_y = ay_data + gy_data + ly_data
        if all_x and all_y:
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            margin = 1.0
            self.ax_xy.set_xlim(min_x - margin, max_x + margin)
            self.ax_xy.set_ylim(min_y - margin, max_y + margin)

        self.fig_xy.canvas.draw()
        self.fig_xy.canvas.flush_events()

        # ---------- 2) 速度 & 转角对比 ----------
        if t_cmd or t_act:
            # 速度
            if t_cmd:
                self.line_v_cmd.set_data(t_cmd, v_cmd)
            if t_act:
                self.line_v_actual.set_data(t_act, v_act)

            # 转角
            if delta_cmd:
                self.line_delta_cmd.set_data(t_cmd, delta_cmd)
            if delta_act:
                self.line_delta_actual.set_data(t_act, delta_act)

            all_t = t_cmd + t_act
            if all_t:
                t_min, t_max = min(all_t), max(all_t)
                if t_max <= t_min:
                    t_max = t_min + 1.0
                self.ax_v.set_xlim(t_min, t_max)
                self.ax_delta.set_xlim(t_min, t_max)

            all_v = v_cmd + v_act
            if all_v:
                v_min, v_max = min(all_v), max(all_v)
                if abs(v_max - v_min) < 1e-3:
                    v_min -= 0.5
                    v_max += 0.5
                self.ax_v.set_ylim(v_min - 0.1, v_max + 0.1)

            all_delta = delta_cmd + delta_act
            if all_delta:
                d_min, d_max = min(all_delta), max(all_delta)
                if abs(d_max - d_min) < 1e-3:
                    d_min -= 0.1
                    d_max += 0.1
                self.ax_delta.set_ylim(d_min - 0.05, d_max + 0.05)

            self.fig_vs.canvas.draw()
            self.fig_vs.canvas.flush_events()

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                self.update_plot()
                rate.sleep()
            except Exception as e:
                rospy.logwarn("绘图循环异常: %s", str(e))
                break

    # ---------- 保存数据 ----------

    def save_data(self):
        """
        退出时保存：
          1) 轨迹对比 CSV + PNG（按索引，包含当前最新一条 MPC 轨迹）
          2) 速度/转角对比 CSV + PNG（按索引）
          3) MPC 全历史轨迹 CSV（方案B：所有局部路径点累积）
        """
        rospy.loginfo("开始保存轨迹 / 速度 / 转角数据...")

        # 文件名仍然用启动时的时间戳，方便一组数据对应一组文件
        ts = self.start_time_str
        traj_csv = os.path.join(self.save_dir, f"trajectory_log_{ts}.csv")
        traj_img = os.path.join(self.save_dir, f"trajectory_compare_{ts}.png")
        vs_csv   = os.path.join(self.save_dir, f"vel_steer_log_{ts}.csv")
        vs_img   = os.path.join(self.save_dir, f"vel_steer_compare_{ts}.png")
        mpc_hist_csv = os.path.join(self.save_dir, f"trajectory_mpc_hist_{ts}.csv")

        # 一次性拷贝缓存中的数据，防止写文件时被回调线程修改
        with self.lock:
            # 轨迹：实际 / A* / MPC（最新一条）
            ax_data = list(self.actual_x)
            ay_data = list(self.actual_y)

            gx_data = list(self.astar_x)
            gy_data = list(self.astar_y)

            lx_data = list(self.mpc_x)
            ly_data = list(self.mpc_y)

            # MPC 历史轨迹（方案B）
            mpc_hist_x = list(self.mpc_hist_x)
            mpc_hist_y = list(self.mpc_hist_y)
            mpc_hist_t = list(self.mpc_hist_t)

            # 速度 / 转角：指令和实际
            v_cmd      = list(self.cmd_v)
            delta_cmd  = list(self.cmd_delta)

            v_act      = list(self.actual_v)
            delta_act  = list(self.actual_delta)

        # ---------- 1) 轨迹 CSV：按索引保存 ----------
        try:
            with open(traj_csv, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "idx",
                    "actual_x", "actual_y",
                    "astar_x", "astar_y",
                    "mpc_x", "mpc_y",
                ])

                max_len = max(
                    len(ax_data), len(ay_data),
                    len(gx_data), len(gy_data),
                    len(lx_data), len(ly_data)
                )

                for i in range(max_len):
                    row = [i]
                    row.append(ax_data[i] if i < len(ax_data) else "")
                    row.append(ay_data[i] if i < len(ay_data) else "")
                    row.append(gx_data[i] if i < len(gx_data) else "")
                    row.append(gy_data[i] if i < len(gy_data) else "")
                    row.append(lx_data[i] if i < len(lx_data) else "")
                    row.append(ly_data[i] if i < len(ly_data) else "")
                    writer.writerow(row)

            rospy.loginfo("轨迹数据已保存到 %s", traj_csv)
        except Exception as e:
            rospy.logwarn("保存轨迹 CSV 失败: %s", str(e))

        # ---------- 2) 速度 / 转角 CSV：按索引保存 ----------
        try:
            with open(vs_csv, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "idx_cmd", "v_cmd", "delta_cmd",
                    "idx_act", "v_actual", "delta_actual",
                ])

                n_cmd = max(len(v_cmd), len(delta_cmd))
                n_act = max(len(v_act), len(delta_act))
                max_len = max(n_cmd, n_act)

                for i in range(max_len):
                    # 指令部分
                    idx_cmd = i if i < n_cmd else ""
                    v_c     = v_cmd[i] if i < len(v_cmd) else ""
                    d_c     = delta_cmd[i] if i < len(delta_cmd) else ""
                    # 实际部分
                    idx_a = i if i < n_act else ""
                    v_a   = v_act[i] if i < len(v_act) else ""
                    d_a   = delta_act[i] if i < len(delta_act) else ""

                    writer.writerow([idx_cmd, v_c, d_c, idx_a, v_a, d_a])

            rospy.loginfo("速度 / 转角数据已保存到 %s", vs_csv)
        except Exception as e:
            rospy.logwarn("保存速度/转角 CSV 失败: %s", str(e))

        # ---------- 3) MPC 全历史轨迹 CSV（方案B） ----------
        try:
            with open(mpc_hist_csv, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["idx", "mpc_x", "mpc_y", "time_from_start"])

                for i, (x, y, t) in enumerate(zip(mpc_hist_x, mpc_hist_y, mpc_hist_t)):
                    writer.writerow([i, x, y, t])

            rospy.loginfo("MPC 全历史轨迹已保存到 %s", mpc_hist_csv)
        except Exception as e:
            rospy.logwarn("保存 MPC 历史 CSV 失败: %s", str(e))

        # ---------- 4) 图片照旧保存 ----------
        try:
            self.fig_xy.savefig(traj_img)
            rospy.loginfo("轨迹对比图已保存到 %s", traj_img)
        except Exception as e:
            rospy.logwarn("保存轨迹图失败: %s", str(e))

        try:
            self.fig_vs.savefig(vs_img)
            rospy.loginfo("速度 / 转角对比图已保存到 %s", vs_img)
        except Exception as e:
            rospy.logwarn("保存速度/转角图失败: %s", str(e))


if __name__ == '__main__':
    try:
        plotter = LivePlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
