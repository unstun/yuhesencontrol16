#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
import tf.transformations as tft
from tf2_ros import TransformException

# 可用参数（可在 launch 里改）
PARAM_CAMERA = "~camera_frame"     # 默认 "camera_init"
PARAM_BODY   = "~body_frame"       # 默认 "body"
PARAM_TOPIC  = "~initialpose_topic"# 默认 "/initialpose"
PARAM_TIMEOUT= "~tf_timeout"       # 默认 1.0 (s)

class InitialPoseToStaticTF(object):
    def __init__(self):
        self.camera = rospy.get_param(PARAM_CAMERA, "camera_init")
        self.body   = rospy.get_param(PARAM_BODY,   "body")
        self.topic  = rospy.get_param(PARAM_TOPIC,  "/initialpose")
        self.timeout= rospy.get_param(PARAM_TIMEOUT, 1.0)
        self.done   = False

        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.lst = tf2_ros.TransformListener(self.buf)
        self.stb = tf2_ros.StaticTransformBroadcaster()

        rospy.Subscriber(self.topic, PoseWithCovarianceStamped, self.cb, queue_size=1)
        rospy.loginfo("initialpose_to_static_tf: waiting for message on %s", self.topic)
        rospy.loginfo("camera_frame=%s, body_frame=%s", self.camera, self.body)

    @staticmethod
    def pose_to_mat(pose):
        p = pose.position
        q = pose.orientation
        M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        M[0, 3], M[1, 3], M[2, 3] = p.x, p.y, p.z
        return M

    @staticmethod
    def mat_to_tr(M):
        trans = M[0:3, 3]
        quat  = tft.quaternion_from_matrix(M)
        return trans, quat

    def cb(self, msg):
        if self.done:
            return

        parent = msg.header.frame_id  # 一般 "map" 或 "2d_map"
        if not parent:
            rospy.logerr("initialpose header.frame_id 为空！请在 RViz 里把 Fixed Frame 设为 map/2d_map。")
            return

        # 1) RViz 点到的 T_parent_body
        T_parent_body = self.pose_to_mat(msg.pose.pose)

        # 2) 取当前 T_camera_body（来自 LIO 的动态 TF）
        try:
            tf_cam_body = self.buf.lookup_transform(
                target_frame=self.camera,    # 目标（上级）坐标系
                source_frame=self.body,      # 源（下级）坐标系
                time=rospy.Time(0),          # 最新
                timeout=rospy.Duration(self.timeout)
            )
        except TransformException as e:
            rospy.logerr("lookup %s -> %s 失败: %s", self.camera, self.body, str(e))
            return

        # 转成 4x4 矩阵
        t = tf_cam_body.transform.translation
        q = tf_cam_body.transform.rotation
        T_cam_body = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T_cam_body[0, 3], T_cam_body[1, 3], T_cam_body[2, 3] = t.x, t.y, t.z

        # 3) 计算 T_parent_camera = T_parent_body * inv(T_cam_body)
        T_parent_cam = T_parent_body.dot(tft.inverse_matrix(T_cam_body))
        trans, quat  = self.mat_to_tr(T_parent_cam)

        # 4) 发一次静态 TF（latched），锁定 camera_init
        out = TransformStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = parent
        out.child_frame_id  = self.camera
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = trans
        out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = quat
        self.stb.sendTransform(out)

        rospy.loginfo("LOCKED STATIC TF: [%s] -> [%s]", parent, self.camera)
        self.done = True

        # 可选：发布后直接退出（更干脆）
        rospy.signal_shutdown("static tf published once; shutting down")

if __name__ == "__main__":
    rospy.init_node("initialpose_to_static_tf")
    InitialPoseToStaticTF()
    rospy.spin()

