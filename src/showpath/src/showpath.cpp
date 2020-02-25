#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

void DrawPath(double x, double y, double th, ros::Publisher &path_pub, nav_msgs::Path &path)
{

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "map";
    path.poses.push_back(this_pose_stamped);

    path_pub.publish(path);
}

main(int argc, char **argv)
{
    ros::init(argc, argv, "showpath");

    ros::NodeHandle ph;
    ros::Publisher slamPathPub = ph.advertise<nav_msgs::Path>("SLAM_Path", 1, true);

    nav_msgs::Path slamPath;
    slamPath.header.stamp = ros::Time::now();
    slamPath.header.frame_id = "map";

    ros::Publisher chassisPathPub = ph.advertise<nav_msgs::Path>("Chassis_Path", 1, true);

    nav_msgs::Path chassisPath;
    chassisPath.header.stamp = ros::Time::now();
    chassisPath.header.frame_id = "map";

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        x += 0.1;
        y += 0.1;
        DrawPath(x, y, th, slamPathPub, slamPath);
        DrawPath(x*(-1), y*(-1), th, chassisPathPub, chassisPath);
        ros::spinOnce(); // check for incoming messages

        loop_rate.sleep();
    }

    return 0;
}