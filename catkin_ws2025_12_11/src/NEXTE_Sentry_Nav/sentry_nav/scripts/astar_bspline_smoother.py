#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
from scipy.interpolate import splprep, splev


class AStarBSplineSmoother(object):
    def __init__(self):
        # 参数：输入/输出路径话题
        input_topic = rospy.get_param(
            "~input_path_topic",
            "/move_base_rmp/PathPlanner/plan"  # A* 原始路径
        )
        output_topic = rospy.get_param(
            "~output_path_topic",
            "/move_base_rmp/bspline_plan"      # 平滑后的路径
        )

        # B 样条平滑参数
        self.smooth_factor = rospy.get_param("~smooth_factor", 0.0)  # 越大越平滑（但偏离原路）
        self.num_samples = rospy.get_param("~num_samples", 200)      # 平滑路径采样点数

        self.sub = rospy.Subscriber(
            input_topic, Path, self.path_callback,
            queue_size=1
        )
        self.pub = rospy.Publisher(
            output_topic, Path,
            queue_size=1, latch=True
        )

        rospy.loginfo(
            "AStarBSplineSmoother: listening on %s, publishing %s",
            input_topic, output_topic
        )

    def path_callback(self, path_msg: Path):
        n = len(path_msg.poses)
        if n < 4:
            # 点太少就直接原样转发
            self.pub.publish(path_msg)
            return

        xs = np.array([p.pose.position.x for p in path_msg.poses])
        ys = np.array([p.pose.position.y for p in path_msg.poses])

        # 以弧长做参数 t，避免点分布不均造成扭结
        ds = np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2)
        total_len = np.sum(ds)
        if total_len < 1e-6:
            self.pub.publish(path_msg)
            return

        t = np.hstack(([0.0], np.cumsum(ds)))
        t /= t[-1]  # 归一化到 [0, 1]

        try:
            # 拟合 B 样条曲线（k=3 三次 B 样条）
            tck, u = splprep(
                [xs, ys],
                u=t,
                s=self.smooth_factor,
                k=3
            )

            # 重新采样成更平滑的路径
            u_new = np.linspace(0.0, 1.0, self.num_samples)
            x_new, y_new = splev(u_new, tck)
        except Exception as e:
            rospy.logwarn("B-spline fitting failed: %s, publish original path.", e)
            self.pub.publish(path_msg)
            return

        smooth_path = Path()
        smooth_path.header = path_msg.header
        smooth_path.poses = []

        for xi, yi in zip(x_new, y_new):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(xi)
            pose.pose.position.y = float(yi)
            pose.pose.position.z = 0.0
            # 先不管朝向，后面把首尾朝向拷贝回来
            smooth_path.poses.append(pose)

        # 把起点、终点的姿态（尤其是 yaw）拷贝回来
        if len(path_msg.poses) >= 2 and len(smooth_path.poses) >= 2:
            smooth_path.poses[0].pose.orientation = path_msg.poses[0].pose.orientation
            smooth_path.poses[-1].pose.orientation = path_msg.poses[-1].pose.orientation

        self.pub.publish(smooth_path)
        rospy.logdebug("Published smoothed path with %d poses", len(smooth_path.poses))


def main():
    rospy.init_node("astar_bspline_smoother")
    smoother = AStarBSplineSmoother()
    rospy.spin()


if __name__ == "__main__":
    main()
