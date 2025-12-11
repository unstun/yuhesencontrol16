#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/**
 * 功能：
 *   从 3D TF 树里读出机器人运动中心（例如 base_footprint_3d）
 *   在 2D 地图坐标系下发布一个“压扁”的 2D base_footprint：
 *     - 位置：只保留 x / y，z 直接设为 0
 *     - 姿态：roll = pitch = 0，只保留 yaw
 *
 * 用到的私有参数（~ 命名空间）：
 *   ~global_frame  : 2D 地图坐标系，默认 "2d_map"
 *   ~source_frame  : 3D 运动中心坐标系，默认 "base_footprint_3d"
 *   ~target_frame  : 2D 运动中心坐标系，默认 "base_footprint"
 *   ~publish_rate  : 发布频率，默认 50.0 Hz
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Trans_TF_2d");
  ros::NodeHandle nh("~");

  // 读取参数
  std::string global_frame;   // 2D 地图坐标系（代价地图用的那个）
  std::string source_frame;   // 3D 里的运动中心（原来的 location / body 等）
  std::string target_frame;   // 2D 里的运动中心（给 move_base 用）

  nh.param<std::string>("global_frame", global_frame, std::string("2d_map"));
  nh.param<std::string>("source_frame", source_frame, std::string("base_footprint_3d"));
  nh.param<std::string>("target_frame", target_frame, std::string("base_footprint"));

  double publish_rate;
  nh.param("publish_rate", publish_rate, 50.0);

  ROS_INFO_STREAM("[Trans_TF_2d] global_frame = " << global_frame
                  << ", source_frame = " << source_frame
                  << ", target_frame = " << target_frame
                  << ", publish_rate = " << publish_rate << " Hz");

  tf::TransformListener  listener;
  tf::TransformBroadcaster broadcaster;
  tf::Transform          tf_out;

  // 给 TF 一点时间初始化
  ros::Duration(1.0).sleep();

  ros::Rate rate(publish_rate);
  while (ros::ok())
  {
    tf::StampedTransform tf_in;

    try
    {
      // 从 2D 地图坐标系看 3D 运动中心
      // 注意：这里假定 global_frame -> ... -> source_frame 在 TF 里是连通的
      listener.lookupTransform(global_frame, source_frame,
                               ros::Time(0), tf_in);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[Trans_TF_2d] lookupTransform(%s -> %s) failed: %s",
                        global_frame.c_str(),
                        source_frame.c_str(),
                        ex.what());
      rate.sleep();
      continue;
    }

    // 提取位姿
    const tf::Vector3& p_in = tf_in.getOrigin();
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_in.getRotation()).getRPY(roll, pitch, yaw);

    // 构造 2D 版本：只保留 x/y/yaw
    tf_out.setOrigin(tf::Vector3(p_in.x(), p_in.y(), 0.0));
    tf::Quaternion q_out;
    q_out.setRPY(0.0, 0.0, yaw);
    tf_out.setRotation(q_out);

    // 这里用 now() 就行，如果你很在意时间同步，也可以用 tf_in.stamp_
    ros::Time stamp = ros::Time::now();
    broadcaster.sendTransform(
        tf::StampedTransform(tf_out, stamp, global_frame, target_frame));

    rate.sleep();
  }

  return 0;
}
