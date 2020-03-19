#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h> //可视化
#include <tf/tf.h>
#include <ConvertCP.h>

// Uart 接口头文件引用
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// TCP 接口头文件引用
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>

#define TCP_PC 0

#define UART_PATH "/dev/ttyACM0"

#define SOCKET_IP "192.168.16.115"

using namespace std;

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

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
    marker.scale.x = 15.0;
    marker.scale.y = 15.0;
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

main(int argc, char **argv)
{
#if TCP_PC
    // socket 构建
    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd == -1)
    {
        cout << "socket 创建失败： " << endl;
        exit(1);
    }
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(SOCKET_IP);
    addr.sin_port = htons(7777);

    int res = bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr));
    if (res == -1)
    {
        cout << "bind创建失败： " << endl;
        exit(-1);
    }
    cout << "bind ok 等待客户端的连接" << endl;
    //参数二：进程上限，一般小于30
    listen(socket_fd, 5);

    struct sockaddr_in client;
    socklen_t len = sizeof(client);

    // 等待连接成功
    int fd = accept(socket_fd, (struct sockaddr *)&client, &len);
    if (fd == -1)
    {
        cout << "accept错误\n"
             << endl;
        exit(-1);
    }
    //使用返回的socket描述符，进行读写通信。
    char *ip = inet_ntoa(client.sin_addr);
    cout << "客户端： [" << ip << "]连接成功" << endl;
#else
    int fd;
    char *portname = UART_PATH;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
#endif

    ros::init(argc, argv, "slamPath");

    ros::NodeHandle n;

    ros::Publisher slamPathPub = n.advertise<nav_msgs::Path>("SLAM_Path", 1, true);
    nav_msgs::Path slamPath;
    slamPath.header.stamp = ros::Time::now();
    slamPath.header.frame_id = "map";

    ros::Publisher chassisPathPub = n.advertise<nav_msgs::Path>("Chassis_Path", 1, true);
    nav_msgs::Path chassisPath;
    chassisPath.header.stamp = ros::Time::now();
    chassisPath.header.frame_id = "map";

    double xSLAM = 0.0;
    double ySLAM = 0.0;
    double xChassis = 0.0;
    double yChassis = 0.0;
    double th = 0.0;

    ros::Publisher slamMarkerPub = n.advertise<visualization_msgs::Marker>("SLAM_Marker", 1);
    visualization_msgs::Marker slamMarker;
    slamMarker.color.r = 1.0f;
    slamMarker.color.g = 0.0f;
    slamMarker.color.b = 1.0f;
    slamMarker.color.a = 1.0;

    ros::Publisher chassisMarkerPub = n.advertise<visualization_msgs::Marker>("Chassis_Marker", 1);
    visualization_msgs::Marker chassisMarker;
    chassisMarker.color.r = 0.0f;
    chassisMarker.color.g = 1.0f;
    chassisMarker.color.b = 1.0f;
    chassisMarker.color.a = 1.0;

    ros::Rate loop_rate(1000);

    CP_DATA getData;
    memset(&getData, 0, sizeof(CP_DATA));
    unsigned long lConunt = 0;
    while (ros::ok())
    {
        unsigned char buffer[64] = {};
        #if TCP_PC
        int size = read(fd, buffer, sizeof(buffer));
        #else
        int size = read(fd, buffer, sizeof(buffer) - 1);
        #endif

        cout << "step" << lConunt << endl;
        cout << "接收到字节数为： " << size << endl;
        cout << "内容： " << buffer << endl;

        if (Arr2PosData(getData, buffer))
        {
            cout << "Check pass" << endl;
            ShowPos(getData);
        }
        else
        {
            // cout << "check error" << endl;
        }

        xSLAM = getData.SLAM.x;
        ySLAM = getData.SLAM.y;

        xChassis = getData.Chassis.x;
        yChassis = getData.Chassis.y;

        DrawPath(xSLAM, ySLAM, th, slamPathPub, slamPath);
        DrawPath(xChassis, yChassis, th, chassisPathPub, chassisPath);

        DrawMarker(xSLAM, ySLAM, slamMarkerPub, slamMarker);
        DrawMarker(xChassis, yChassis, chassisMarkerPub, chassisMarker);

        ros::spinOnce(); // check for incoming messages

        loop_rate.sleep();

        lConunt++;
    }

    return 0;
}