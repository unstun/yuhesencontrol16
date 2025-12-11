#!/usr/bin/env python3
import math

import rospy
import reeds_shepp
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header

# 辅助函数：yaw -> quaternion
def yaw_to_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q

# 辅助函数：从 PoseStamped 里取出 (x, y, yaw)
def pose_to_xyyaw(pose):
    x = pose.pose.position.x
    y = pose.pose.position.y

    q = pose.pose.orientation
    # 只考虑 yaw（z 轴旋转）
    # yaw = atan2(2*w*z, 1 - 2*z^2)
    yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
    return x, y, yaw


class ReedsSheppSmoother(object):
    def __init__(self):
        # 从参数服务器读取参数（可以在 launch 里改）
        self.input_topic = rospy.get_param("~input_path_topic",
                                           "/move_base/GlobalPlanner/plan")
        self.output_topic = rospy.get_param("~output_path_topic",
                                            "/rs_smoothed_path")
        self.turning_radius = rospy.get_param("~turning_radius", 1.6)
        self.step_size = rospy.get_param("~step_size", 0.1)

        rospy.loginfo("Reeds-Shepp smoother:")
        rospy.loginfo("  input_topic  = %s", self.input_topic)
        rospy.loginfo("  output_topic = %s", self.output_topic)
        rospy.loginfo("  turning_radius = %.3f m", self.turning_radius)
        rospy.loginfo("  step_size      = %.3f m", self.step_size)

        self.path_sub = rospy.Subscriber(self.input_topic, Path,
                                         self.path_callback,
                                         queue_size=1)

        self.path_pub = rospy.Publisher(self.output_topic, Path,
                                        queue_size=1)

    def path_callback(self, msg):
        poses = msg.poses
        if len(poses) < 2:
            rospy.logwarn("Received path too short: %d points", len(poses))
            return

        rospy.loginfo("Received path with %d points, smoothing...", len(poses))

        smoothed_path = Path()
        smoothed_path.header = Header()
        smoothed_path.header.stamp = rospy.Time.now()
        smoothed_path.header.frame_id = msg.header.frame_id

        # 为了防止点太多，这里可以对原始路径做个降采样
        # 比如每隔 N 个点取一个作为 key 点
        N = 3  # 每 3 个点取一个（你可以以后调）
        key_indices = list(range(0, len(poses), N))
        if key_indices[-1] != len(poses) - 1:
            key_indices.append(len(poses) - 1)

        # 存储之前添加的最后一个点，避免重复
        first_segment = True

        for i in range(len(key_indices) - 1):
            idx0 = key_indices[i]
            idx1 = key_indices[i + 1]
            p0 = poses[idx0]
            p1 = poses[idx1]

            x0, y0, yaw0 = pose_to_xyyaw(p0)
            x1, y1, yaw1 = pose_to_xyyaw(p1)

            q0 = (x0, y0, yaw0)
            q1 = (x1, y1, yaw1)

            try:
                rs_points = reeds_shepp.path_sample(
                    q0, q1, self.turning_radius, self.step_size)
            except Exception as e:
                rospy.logwarn("Reeds-Shepp calculation failed between index %d and %d: %s",
                              idx0, idx1, str(e))
                continue

            # 第一个 segment 要保留第一个点
            # 后面的 segment 去掉第一个点（避免重复）
        # 第一个 segment 要保留第一个点
        # 后面的 segment 去掉第一个点（避免重复）
        start_idx = 0 if first_segment else 1

        # 有些 reeds_shepp 实现返回的点可能是 (x, y, yaw, 其它信息...)
        # 这里我们只关心前三个元素
        for pt in rs_points[start_idx:]:
            x = pt[0]
            y = pt[1]
            yaw = pt[2]

            pose_stamped = PoseStamped()
            pose_stamped.header = smoothed_path.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = yaw_to_quat(yaw)
            smoothed_path.poses.append(pose_stamped)

        first_segment = False


        rospy.loginfo("Publish smoothed path with %d points",
                      len(smoothed_path.poses))
        self.path_pub.publish(smoothed_path)


if __name__ == "__main__":
    rospy.init_node("reeds_shepp_smoother")
    node = ReedsSheppSmoother()
    rospy.loginfo("Reeds-Shepp smoother node started.")
    rospy.spin()
