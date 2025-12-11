#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "Trans_TF_2d");

  ros::NodeHandle node;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  
  // 等待 listener 缓存一些变换数据
  ros::Duration(1.0).sleep();

  ros::Rate rate(100.0); // 100Hz 足够了
  while (node.ok()){
    tf::StampedTransform transform_listener;
    
    try{
      // 1. 等待从 2d_map 到 body 的变换变为可用
      //    (这会同时等待 fast_lio 和 static_transform_publisher)
      listener.waitForTransform("2d_map", "body", ros::Time(0), ros::Duration(1.0));

      // 2. 查询从 "2d_map" (2D 世界) 到 "body" (3D 机器人) 的变换
      listener.lookupTransform("2d_map", "body",  
                               ros::Time(0), transform_listener);
    }
    catch (tf::TransformException &ex){ // 注意：使用 &ex
      ROS_WARN("Trans_TF_2d: 无法获取 '2d_map' -> 'body' 的变换: %s", ex.what());
      ros::Duration(0.1).sleep();
      continue; // 关键：跳过本次循环，直接重试
    }
    
    // 3. 获取 'body' 相对于 '2d_map' 的 3D 位置
    double robot_pose_x = transform_listener.getOrigin().x();
    double robot_pose_y = transform_listener.getOrigin().y();
    // 我们将 Z 设为 0, 所以不需要读取它

    // 4. 获取 3D 姿态 (四元数)
    tf::Quaternion q_3d = transform_listener.getRotation();
    
    // 5. 从 3D 四元数中提取 2D 偏航角 (Yaw)
    double roll, pitch, yaw;
    tf::Matrix3x3(q_3d).getRPY(roll, pitch, yaw);
    
    // 6. 创建一个新的、纯 2D 的变换
    tf::Transform transform_broadcaster;
    transform_broadcaster.setOrigin( tf::Vector3(robot_pose_x, robot_pose_y, 0.0) );
    
    // 7. 创建一个纯 2D 的四元数 (roll=0, pitch=0, yaw=yaw)
    tf::Quaternion q_2d;
    q_2d.setRPY(0, 0, yaw);
    transform_broadcaster.setRotation(q_2d);

    // 8. 广播这个新的 2D 变换: "2d_map" -> "body_2d"
    broadcaster.sendTransform(tf::StampedTransform(transform_broadcaster, ros::Time::now(), "2d_map", "body_2d"));
    
    rate.sleep();
  }
  return 0;
};