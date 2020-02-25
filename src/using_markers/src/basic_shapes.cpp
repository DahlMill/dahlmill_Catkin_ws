#include <ros/ros.h>
#include <visualization_msgs/Marker.h> //可视化

void DrawMarker(double x, double y, ros::Publisher &marker_pub, visualization_msgs::Marker &marker)
{
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
  //实例化一个Marker
  
  // 设置frame ID 和 时间戳
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  // 为这个marker设置一个独一无二的ID，一个marker接收到相同ns和id就会用新的信息代替旧的
  marker.ns = "basic_shapes";
  marker.id = 0;
  // 设置marker类型，初始化是立方体
  marker.type = shape;
  // 添加marker
  marker.action = visualization_msgs::Marker::ADD;
  // 设置marker的位置
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  // 设置marker的大小
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 0.5;
  // Set the color -- be sure to set alpha to something non-zero!
  // 设置marker的颜色
  // marker.color.r = 1.0f;
  // marker.color.g = 1.0f;
  // marker.color.b = 1.0f;
  // marker.color.a = 1.0;

  //取消自动删除
  marker.lifetime = ros::Duration();

  // Publish the marker
  // 必须有订阅者才会发布消息
  while (marker_pub.getNumSubscribers() < 1)
  {
    // if (!ros::ok())
    // {
    //   return 0;
    // }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);

  // Cycle between different shapes
  // 连续改变形状
  // switch (shape)
  // {
  // case visualization_msgs::Marker::CUBE:
  //   shape = visualization_msgs::Marker::SPHERE;
  //   break;
  // case visualization_msgs::Marker::SPHERE:
  //   shape = visualization_msgs::Marker::ARROW;
  //   break;
  // case visualization_msgs::Marker::ARROW:
  //   shape = visualization_msgs::Marker::CYLINDER;
  //   break;
  // case visualization_msgs::Marker::CYLINDER:
  //   shape = visualization_msgs::Marker::CUBE;
  //   break;
  // }
}

int main(int argc, char **argv)
{
  //初始化ROS，幷且创建一个ROS::Publisher 在话题visualization_marker上面
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher slamMarkerPub = n.advertise<visualization_msgs::Marker>("SLAM_Marker", 1);
  visualization_msgs::Marker slamMarker;
  slamMarker.color.r = 1.0f;
  slamMarker.color.g = 0.0f;
  slamMarker.color.b = 0.0f;
  slamMarker.color.a = 1.0;

  ros::Publisher chassisMarkerPub = n.advertise<visualization_msgs::Marker>("Chassis_Marker", 1);
  visualization_msgs::Marker chassisMarker;
  chassisMarker.color.r = 0.0f;
  chassisMarker.color.g = 1.0f;
  chassisMarker.color.b = 0.0f;
  chassisMarker.color.a = 1.0;

  // 初始化形状为圆柱体

  int x = 0;
  int y = 0;

  while (ros::ok())
  {
    x++;
    y++;


    DrawMarker(x, y, slamMarkerPub, slamMarker);
    DrawMarker(x * (-1), y * (-1), chassisMarkerPub, chassisMarker);
    r.sleep();
  }
}
