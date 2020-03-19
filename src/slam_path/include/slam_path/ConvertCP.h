/*
 * @Author: your name
 * @Date: 2020-02-22 16:39:03
 * @LastEditTime: 2020-02-26 16:41:03
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/slam_path/include/slam_path/ConvertCP.h
 */
#ifndef CONVERT_CP_H
#define CONVERT_CP_H
#endif

struct POS
{
    int x;
    int y;
    int z;
    int yaw;
    unsigned char status;
};

struct CP_DATA
{
    POS SLAM;
    POS Chassis;
};

void ShowPos(CP_DATA get);
void ShowHexArr(unsigned char arr[], int len);

void PosData2Arr(CP_DATA &post, unsigned char *arr);
bool Arr2PosData(CP_DATA &get, unsigned char *arr);