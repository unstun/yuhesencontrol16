#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import open3d as o3d
import rospy
import ros_numpy
import sensor_msgs.point_cloud2 as pc2  # 引入标准库以防ros_numpy失败
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def msg_to_array(pc_msg):
    """
    通用点云转数组函数，主要用于解析地图点云
    """
    try:
        pc_array = ros_numpy.numpify(pc_msg)
        pc = np.zeros([len(pc_array), 3])
        pc[:, 0] = pc_array['x']
        pc[:, 1] = pc_array['y']
        pc[:, 2] = pc_array['z']
        return pc
    except Exception as e:
        # 如果ros_numpy失败，使用标准库兜底
        gen = pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
        return np.array(list(gen))


def registration_at_scale(pc_scan, pc_map, initial, scale):
    try:
        # 兼容 Open3D 不同版本的 API
        if hasattr(o3d.pipelines.registration, 'registration_icp'):
            reg_func = o3d.pipelines.registration.registration_icp
            pt2pt = o3d.pipelines.registration.TransformationEstimationPointToPoint()
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        else:
            reg_func = o3d.registration.registration_icp
            pt2pt = o3d.registration.TransformationEstimationPointToPoint()
            criteria = o3d.registration.ICPConvergenceCriteria(max_iteration=20)
            
        result_icp = reg_func(
            voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), 
            voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
            1.0 * scale, initial,
            pt2pt,
            criteria
        )
        return result_icp.transformation, result_icp.fitness
    except Exception as e:
        rospy.logerr("ICP Error: {}".format(e))
        return initial, 0.0


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    # Check if input pc has intensity channel (Nx4) or just xyz (Nx3)
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom, initialized
    # 用icp配准
    rospy.loginfo('Global localization by scan-to-map matching......')

    if cur_scan is None or cur_odom is None:
        rospy.logwarn("Waiting for scan or odom...")
        return False

    # 注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    rospy.loginfo('Time cost: {:.3f}s, Fitness: {:.4f}'.format(toc - tic, fitness))

    # ================= 核心逻辑：阈值判断与防跳变 =================
    if fitness > LOCALIZATION_TH:
        
        # [防跳变检测]：如果已经初始化过了，检查这次修正的距离是否过大
        if initialized:
            # 获取旧的平移向量 (xyz)
            old_trans = T_map_to_odom[:3, 3]
            # 获取新的平移向量 (xyz)
            new_trans = transformation[:3, 3]
            
            # 计算两者的欧氏距离
            diff_dist = np.linalg.norm(old_trans - new_trans)
            
            # 如果瞬移超过设定距离，拒绝更新
            if diff_dist > MAX_JUMP_DIST:
                rospy.logerr('--------------------------------------------------')
                rospy.logerr('!!! REJECTED LARGE JUMP !!!')
                rospy.logerr('Detected Jump Distance: {:.3f} m (Limit: {:.3f} m)'.format(diff_dist, MAX_JUMP_DIST))
                rospy.logerr('Fitness was good ({:.4f}), but move was too sudden.'.format(fitness))
                rospy.logerr('Keeping old pose to prevent jitter.')
                rospy.logerr('--------------------------------------------------')
                return False 
        
        # 更新位姿
        T_map_to_odom = transformation

        # 发布map_to_odom
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        
        rospy.loginfo('>>> Localization Updated Successfully.')
        return True
    else:
        rospy.logwarn('Not match (Low Fitness)!!!!')
        rospy.logwarn('Fitness score: {:.4f} < Threshold: {:.2f}'.format(fitness, LOCALIZATION_TH))
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map

    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    rospy.loginfo('Global map received.')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    """
    Robust scan callback handling both custom FAST-LIO fields and standard PointCloud2.
    """
    global cur_scan
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    try:
        pub_pc_in_map.publish(pc_msg)
    except:
        pass

    pc_points = None
    
    # 【方案 A】尝试还原 FAST-LIO 专用字段映射 (速度快)
    # 仅当字段数量看起来像 FAST-LIO 的格式时尝试
    try:
        if len(pc_msg.fields) >= 8:
            # 必须使用副本修改，避免影响其他订阅者
            msg_copy = copy.deepcopy(pc_msg)
            # 重排字段以匹配ros_numpy预期
            msg_copy.fields = [
                msg_copy.fields[0], msg_copy.fields[1], msg_copy.fields[2], # x, y, z
                msg_copy.fields[4], msg_copy.fields[5], msg_copy.fields[6],
                msg_copy.fields[3], msg_copy.fields[7]
            ]
            pc_array = ros_numpy.numpify(msg_copy)
            
            pc_points = np.zeros([len(pc_array), 3])
            pc_points[:, 0] = pc_array['x']
            pc_points[:, 1] = pc_array['y']
            pc_points[:, 2] = pc_array['z']
    except Exception:
        pc_points = None

    # 【方案 B】标准保底方案 (通用性强，防崩溃)
    # 如果 ros_numpy 失败，使用 sensor_msgs.point_cloud2.read_points
    if pc_points is None:
        try:
            # 仅读取 xyz
            gen = pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
            pc_points = np.array(list(gen))
        except Exception as e:
            rospy.logerr_throttle(2.0, "Failed to parse point cloud: {}".format(e))
            return

    # 转为 Open3D 格式
    if pc_points is not None and len(pc_points) > 0:
        cur_scan = o3d.geometry.PointCloud()
        cur_scan.points = o3d.utility.Vector3dVector(pc_points)
    else:
        rospy.logwarn_throttle(2.0, "Received empty or invalid point cloud")


def thread_localization():
    global T_map_to_odom
    while True:
        # 每隔一段时间进行全局定位
        rospy.sleep(1 / FREQ_LOCALIZATION)
        # 只有当已经初始化后，才在后台持续修正
        # 且使用最新的 map_to_odom 作为先验
        if initialized:
            global_localization(T_map_to_odom)


if __name__ == '__main__':
    # ================== 参数配置区域 ==================
    MAP_VOXEL_SIZE = 0.4
    SCAN_VOXEL_SIZE = 0.1

    # Global localization frequency (HZ)
    # 建议维持在 0.5Hz - 1.0Hz，太快会抢占CPU导致里程计卡顿
    FREQ_LOCALIZATION = 0.01 

    # [优化] 提高阈值，过滤掉质量不好的匹配 (原值通常是0.9)
    LOCALIZATION_TH = 0.93 

    # [新增] 最大允许跳变距离 (米)
    # 如果ICP计算出的新位置相比上一时刻突变超过这个距离，则认为是误匹配，直接丢弃
    MAX_JUMP_DIST = 0.5

    # FOV(rad), modify this according to your LiDAR type
    FOV = 6.28

    # The farthest distance(meters) within FOV
    FOV_FAR = 30
    # =================================================

    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    # 初始化
    while not initialized:
        rospy.logwarn('Waiting for initial pose....')

        # 等待初始位姿
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        initial_pose = pose_to_mat(pose_msg)
        if cur_scan:
            # 第一次初始化时允许大范围修正（不检查跳变）
            if global_localization(initial_pose):
                initialized = True
        else:
            rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    # 开始定期全局定位
    _thread.start_new_thread(thread_localization, ())

    rospy.spin()