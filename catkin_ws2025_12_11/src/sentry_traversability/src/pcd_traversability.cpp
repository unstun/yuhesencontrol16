#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

// -------- 常量 --------
const double PI = 3.14159265358979323846;

// -------- 全局参数 --------
double map_resolution;          // 栅格分辨率
double vaf_radius;              // VAF 邻域半径
double max_slope_deg;           // 最大爬坡角（度）
double relaxation_factor;       // VAF 松弛因子
double voxel_leaf_size;         // 体素滤波尺寸

double sensor_pitch_deg;        // 传感器向下安装的俯仰角(绝对值)，用来把点云向上旋转
double pre_filter_z_max;        // VAF 前高度预滤波阈值（旋转后 z > 此值 的点剔除）

std::string file_directory;
std::string file_name;
std::string map_topic_name;
std::string frame_id;

// 新增：可通行 / 不可通行点云话题名称
std::string traversable_cloud_topic;
std::string non_traversable_cloud_topic;

// -------- 载入 PCD --------
bool loadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::string full_path = file_directory;
  if (!full_path.empty() && full_path.back() != '/')
    full_path += "/";
  full_path += file_name + ".pcd";

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(full_path, *cloud) == -1) {
    ROS_ERROR("无法加载 PCD 文件: %s", full_path.c_str());
    return false;
  }
  ROS_INFO("成功加载 PCD: %s, 点数: %zu", full_path.c_str(), cloud->points.size());
  return true;
}

// -------- 绕 Y 轴向上旋转点云（纠正雷达向下安装角度）--------
// 右手系，x 前，y 左，z 上；正角度为绕 y 轴右手旋转
void rotatePointCloudPitch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                           double pitch_deg) {
  if (!cloud_in) return;

  cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());
  cloud_out->points.resize(cloud_in->points.size());
  cloud_out->width  = cloud_in->width;
  cloud_out->height = cloud_in->height;
  cloud_out->is_dense = cloud_in->is_dense;

  double theta = pitch_deg * PI / 180.0;
  double c = std::cos(theta);
  double s = std::sin(theta);

  for (size_t i = 0; i < cloud_in->points.size(); ++i) {
    const auto &p = cloud_in->points[i];
    pcl::PointXYZ q;

    // 绕 y 轴旋转:
    // x' =  c * x + s * z
    // y' =  y
    // z' = -s * x + c * z
    q.x = static_cast<float>(c * p.x + s * p.z);
    q.y = p.y;
    q.z = static_cast<float>(-s * p.x + c * p.z);

    cloud_out->points[i] = q;
  }

  ROS_INFO("已对点云绕 Y 轴向上旋转 %.2f 度，点数: %zu",
           pitch_deg, cloud_out->points.size());
}

// -------- VAF + 旋转后高度预滤波 + 体素降采样 + 栅格生成 + 点云分割 --------
void buildTraversabilityMapVAF(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_rotated,
    nav_msgs::OccupancyGrid &msg,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &trav_cloud_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &nontrav_cloud_out) {

  // 确保输出指针至少是有效的空点云，避免主函数崩溃
  trav_cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());
  nontrav_cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());

  if (!cloud_rotated || cloud_rotated->points.empty()) {
    ROS_WARN("输入点云为空，无法生成栅格地图");
    return;
  }

  const int N_input = static_cast<int>(cloud_rotated->points.size());

  // 0. 在 VAF 前做一次高度预滤波：旋转后的点云，z > pre_filter_z_max 的点全部剔除
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());
  cloud_raw->points.reserve(cloud_rotated->points.size());

  int removed_pre_height = 0;
  for (int i = 0; i < N_input; ++i) {
    const auto &p = cloud_rotated->points[i];
    if (static_cast<double>(p.z) > pre_filter_z_max) {
      removed_pre_height++;
      continue;
    }
    cloud_raw->points.push_back(p);
  }
  cloud_raw->width  = cloud_raw->points.size();
  cloud_raw->height = 1;
  cloud_raw->is_dense = true;

  if (cloud_raw->points.empty()) {
    ROS_ERROR("高度预滤波后点云为空，请检查 pre_filter_z_max=%.2f 参数和安装角度",
              pre_filter_z_max);
    return;
  }

  const int N_raw = static_cast<int>(cloud_raw->points.size());
  ROS_INFO("VAF 前高度预滤波: 输入点=%d, 保留=%d, 剔除(z>%.2f)=%d",
           N_input, N_raw, pre_filter_z_max, removed_pre_height);

  // 1. 体素降采样，减少 VAF 运算量
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud_raw);
  voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel.filter(*cloud_ds);

  if (cloud_ds->points.empty()) {
    ROS_ERROR("体素降采样后点云为空，请检查 voxel_leaf_size 参数");
    return;
  }

  const int N_ds = static_cast<int>(cloud_ds->points.size());
  ROS_INFO("体素降采样: 原始点数=%d, 降采样后=%d, leaf_size=%.3f",
           N_raw, N_ds, voxel_leaf_size);

  // 2. 在降采样点云上构建 KDTree 并做 VAF
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_ds);

  std::vector<char> is_traversable_ds(N_ds, 0);  // 仅 VAF 判定结果（不再做第二次高度过滤）

  double max_angle_rad = max_slope_deg * PI / 180.0;

  std::vector<int>   neighbor_indices;
  std::vector<float> neighbor_sqr_dists;

  ROS_INFO("开始 VAF 分割: radius=%.3f, max_angle=%.1f deg, relaxation_factor=%.3f",
           vaf_radius, max_slope_deg, relaxation_factor);

  int step = std::max(1, N_ds / 100);  // 用于打印进度

  for (int i = 0; i < N_ds; ++i) {
    const auto &pt = cloud_ds->points[i];

    neighbor_indices.clear();
    neighbor_sqr_dists.clear();

    int found = kdtree.radiusSearch(pt, vaf_radius, neighbor_indices, neighbor_sqr_dists);
    if (found <= 1) {
      // 附近没有足够邻居，无法判断坡度，保持为不可通行(0)
      if (i % step == 0) {
        double percent = 100.0 * static_cast<double>(i) / static_cast<double>(N_ds);
        ROS_INFO("VAF 进度: %.1f%%", percent);
      }
      continue;
    }

    int angle_count = 0;
    int total_neighbors = found - 1;

    for (int k = 0; k < found; ++k) {
      int idx = neighbor_indices[k];
      if (idx == i) continue;

      const auto &nbr = cloud_ds->points[idx];
      double dx = static_cast<double>(pt.x) - static_cast<double>(nbr.x);
      double dy = static_cast<double>(pt.y) - static_cast<double>(nbr.y);
      double horizontal_dist = std::sqrt(dx * dx + dy * dy);
      if (horizontal_dist < 1e-6) continue;

      double dz = std::fabs(static_cast<double>(pt.z) - static_cast<double>(nbr.z));
      double angle = std::atan2(dz, horizontal_dist);

      if (angle > max_angle_rad) {
        angle_count++;
      }
    }

    double exceed_ratio = (total_neighbors > 0)
                          ? static_cast<double>(angle_count) / static_cast<double>(total_neighbors)
                          : 0.0;

    // exceed_ratio <= relaxation_factor -> 认为该点为可通行点
    if (exceed_ratio <= relaxation_factor) {
      is_traversable_ds[i] = 1;
    }

    if (i % step == 0) {
      double percent = 100.0 * static_cast<double>(i) / static_cast<double>(N_ds);
      ROS_INFO("VAF 进度: %.1f%%", percent);
    }
  }

  int vaf_trav_count = 0;
  for (int i = 0; i < N_ds; ++i)
    if (is_traversable_ds[i]) vaf_trav_count++;

  ROS_INFO("VAF 分割结束：可通行点数 = %d (%.2f %%)",
           vaf_trav_count,
           100.0 * static_cast<double>(vaf_trav_count) / static_cast<double>(N_ds));

  // 2.1 根据 VAF 结果构造可通行 & 不可通行点云（基于降采样后的 cloud_ds）
  trav_cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());
  nontrav_cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());
  trav_cloud_out->points.reserve(N_ds);
  nontrav_cloud_out->points.reserve(N_ds);

  for (int i = 0; i < N_ds; ++i) {
    if (is_traversable_ds[i]) {
      trav_cloud_out->points.push_back(cloud_ds->points[i]);
    } else {
      nontrav_cloud_out->points.push_back(cloud_ds->points[i]);
    }
  }

  trav_cloud_out->width  = trav_cloud_out->points.size();
  trav_cloud_out->height = 1;
  trav_cloud_out->is_dense = true;

  nontrav_cloud_out->width  = nontrav_cloud_out->points.size();
  nontrav_cloud_out->height = 1;
  nontrav_cloud_out->is_dense = true;

  ROS_INFO("VAF 点云导出: traversable=%zu, non_traversable=%zu",
           trav_cloud_out->points.size(),
           nontrav_cloud_out->points.size());

  // 3. 用“旋转+预滤波后的原始点云”计算 XY 边界
  double x_min = cloud_raw->points[0].x;
  double x_max = cloud_raw->points[0].x;
  double y_min = cloud_raw->points[0].y;
  double y_max = cloud_raw->points[0].y;

  for (int i = 1; i < N_raw; ++i) {
    const auto &p = cloud_raw->points[i];
    if (p.x < x_min) x_min = p.x;
    if (p.x > x_max) x_max = p.x;
    if (p.y < y_min) y_min = p.y;
    if (p.y > y_max) y_max = p.y;
  }

  // 4. 填 OccupancyGrid 元信息
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = map_resolution;

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width  = static_cast<unsigned int>((x_max - x_min) / map_resolution) + 1;
  msg.info.height = static_cast<unsigned int>((y_max - y_min) / map_resolution) + 1;

  size_t grid_size = static_cast<size_t>(msg.info.width) * msg.info.height;
  msg.data.assign(grid_size, -1);  // 默认 unknown

  std::vector<char> has_raw(grid_size, 0);
  std::vector<char> has_trav(grid_size, 0);

  // 5. 预滤波后原始点云：标记 has_raw （表示“这里有感知到东西”）
  for (int i = 0; i < N_raw; ++i) {
    const auto &p = cloud_raw->points[i];
    int ix = static_cast<int>((p.x - x_min) / map_resolution);
    int iy = static_cast<int>((p.y - y_min) / map_resolution);

    if (ix < 0 || iy < 0 ||
        ix >= static_cast<int>(msg.info.width) ||
        iy >= static_cast<int>(msg.info.height))
      continue;

    size_t idx = static_cast<size_t>(ix) + static_cast<size_t>(iy) * msg.info.width;
    has_raw[idx] = 1;
  }

  // 6. 降采样 + VAF 可通行点：标记 has_trav
  for (int i = 0; i < N_ds; ++i) {
    if (!is_traversable_ds[i]) continue;

    const auto &p = cloud_ds->points[i];
    int ix = static_cast<int>((p.x - x_min) / map_resolution);
    int iy = static_cast<int>((p.y - y_min) / map_resolution);

    if (ix < 0 || iy < 0 ||
        ix >= static_cast<int>(msg.info.width) ||
        iy >= static_cast<int>(msg.info.height))
      continue;

    size_t idx = static_cast<size_t>(ix) + static_cast<size_t>(iy) * msg.info.width;
    has_trav[idx] = 1;
  }

  // 7. 生成最终栅格：
  //    - has_trav == 1             -> 0 (free)
  //    - has_raw == 1 & !has_trav  -> 100 (obstacle)
  //    - 否则                      -> -1 (unknown)
  int free_cells = 0, occ_cells = 0, unknown_cells = 0;
  for (size_t idx = 0; idx < grid_size; ++idx) {
    if (has_trav[idx]) {
      msg.data[idx] = 0;
      free_cells++;
    } else if (has_raw[idx]) {
      msg.data[idx] = 100;
      occ_cells++;
    } else {
      msg.data[idx] = -1;
      unknown_cells++;
    }
  }

  ROS_INFO("生成 VAF 可通行栅格: width=%u, height=%u, res=%.3f",
           msg.info.width, msg.info.height, msg.info.resolution);
  ROS_INFO("栅格统计: free=%d, occ=%d, unknown=%d",
           free_cells, occ_cells, unknown_cells);
}

// -------- main --------
int main(int argc, char **argv) {
  ros::init(argc, argv, "pcd_traversability");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("file_directory", file_directory,
            std::string("/home/robot/catkin_ws/src/fast_lio/PCD"));
  pnh.param("file_name", file_name, std::string("scans"));

  pnh.param("map_resolution", map_resolution, 0.05);
  pnh.param("vaf_radius", vaf_radius, 0.5);
  pnh.param("max_slope_deg", max_slope_deg, 40.0);
  pnh.param("relaxation_factor", relaxation_factor, 0.15);
  pnh.param("voxel_leaf_size", voxel_leaf_size, 0.2);

  // 雷达俯仰角 & VAF 前高度预滤波阈值
  pnh.param("sensor_pitch_deg", sensor_pitch_deg, 30.0);   // 雷达向下 30 度 => 点云向上旋转 30 度
  pnh.param("pre_filter_z_max", pre_filter_z_max, 1.0);    // 旋转后 z > 1.0m 的点剔除

  pnh.param("map_topic_name", map_topic_name, std::string("traversability_map"));
  pnh.param("frame_id",       frame_id,       std::string("2d_map"));

  // 新增：点云话题名称
  pnh.param("traversable_cloud_topic",
            traversable_cloud_topic,
            std::string("traversable_points"));
  pnh.param("non_traversable_cloud_topic",
            non_traversable_cloud_topic,
            std::string("non_traversable_points"));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_in(new pcl::PointCloud<pcl::PointXYZ>());
  if (!loadPointCloud(cloud_raw_in)) {
    return -1;
  }

  // 先对原始点云做绕 Y 轴的向上旋转，纠正雷达向下安装的俯仰角
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZ>());
  rotatePointCloudPitch(cloud_raw_in, cloud_rotated, sensor_pitch_deg);

  nav_msgs::OccupancyGrid grid;

  // 用于接收 VAF 分割结果的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr trav_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr nontrav_cloud;

  buildTraversabilityMapVAF(cloud_rotated, grid, trav_cloud, nontrav_cloud);

  ros::Publisher map_pub =
      nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1, true);
  ros::Publisher trav_pub =
      nh.advertise<sensor_msgs::PointCloud2>(traversable_cloud_topic, 1, true);
  ros::Publisher nontrav_pub =
      nh.advertise<sensor_msgs::PointCloud2>(non_traversable_cloud_topic, 1, true);

  // 发布栅格
  map_pub.publish(grid);

  // 发布两朵点云
  if (trav_cloud) {
    sensor_msgs::PointCloud2 trav_msg;
    pcl::toROSMsg(*trav_cloud, trav_msg);
    trav_msg.header.frame_id = frame_id;
    trav_msg.header.stamp = ros::Time::now();
    trav_pub.publish(trav_msg);
  } else {
    ROS_WARN("可通行点云为空，未发布");
  }

  if (nontrav_cloud) {
    sensor_msgs::PointCloud2 nontrav_msg;
    pcl::toROSMsg(*nontrav_cloud, nontrav_msg);
    nontrav_msg.header.frame_id = frame_id;
    nontrav_msg.header.stamp = ros::Time::now();
    nontrav_pub.publish(nontrav_msg);
  } else {
    ROS_WARN("不可通行点云为空，未发布");
  }

  ROS_INFO("已发布可通行栅格到 topic: %s (latched)", map_topic_name.c_str());
  ROS_INFO("已发布可通行点云: %s, 不可通行点云: %s (latched)",
           traversable_cloud_topic.c_str(),
           non_traversable_cloud_topic.c_str());

  ros::spin();
  return 0;
}
