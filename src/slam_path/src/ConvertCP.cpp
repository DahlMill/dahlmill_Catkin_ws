/*
 * @Author: your name
 * @Date: 2020-02-22 16:39:03
 * @LastEditTime: 2020-02-25 17:44:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/slam_path/src/ConvertCP.cpp
 */
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include "ConvertCP.h"

using namespace std;

/**
 * @description: 将一个int数据分解到两个字节中
 * @param {int} 需要分解的数据 {unsigend char} 需要存储的字节指针 
 * @return: 
 */
void CovInt2UChar(int data, unsigned char &hBit, unsigned char &lBit)
{
    hBit = data >> 8;
    lBit = data;
}

/**
 * @description: 两个字节的拼成一个有效数据
 * @param {unsigned char} 需要拼接的数据
 * @return: 拼接结果
 */
short CovUChar2Int(unsigned char hBit, unsigned char lBit)
{
    short data = (hBit << 8 | lBit);
    // data = (data < 32768) ? data : (data - 65536);
    return data;
}

/**
 * @description: 将位姿数据转换为数组 
 * @param {CP_DATA} SLAM 和底盘的位姿 {unsigned char} 需要存储的字节数组指针
 * @return: 
 */
void PosData2Arr(CP_DATA &post, unsigned char (&arr)[24])
{
    // int slamX = post.SLAM.x;
    // int slamY = post.SLAM.y;
    // int slamZ = post.SLAM.z;
    // int slamYaw = post.SLAM.yaw;
    // unsigned char slamStatus = post.SLAM.status;

    // int chassisX = post.Chassis.x;
    // int chassisY = post.Chassis.y;
    // int chassisYaw = post.Chassis.yaw;
    // unsigned char chassisStatus = post.Chassis.status;
    
    arr[0] = 0x55;
    arr[1] = 0x55;
    arr[2] = 0xAA;
    arr[3] = 0xAA;

    CovInt2UChar(post.SLAM.x, arr[4], arr[5]);
    CovInt2UChar(post.SLAM.y, arr[6], arr[7]);
    CovInt2UChar(post.SLAM.z, arr[8], arr[9]);
    CovInt2UChar(post.SLAM.yaw, arr[10], arr[11]);

    CovInt2UChar(post.Chassis.x, arr[12], arr[13]);
    CovInt2UChar(post.Chassis.y, arr[14], arr[15]);
    CovInt2UChar(post.Chassis.yaw, arr[16], arr[17]);

    // arr[4] = slamX >> 8;
    // arr[5] = slamX;

    // arr[6] = slamY >> 8;
    // arr[7] = slamY;

    // arr[8] = slamZ >> 8;
    // arr[9] = slamZ;

    // arr[10] = slamYaw >> 8;
    // arr[11] = slamYaw;

    // arr[12] = chassisX >> 8;
    // arr[13] = chassisX;

    // arr[14] = chassisY >> 8;
    // arr[15] = chassisY;

    // arr[16] = chassisYaw >> 8;
    // arr[17] = chassisYaw;

    arr[18] = post.SLAM.status;
    arr[19] = post.Chassis.status;

    int checkSum = 0;
    for(int i =0; i < (19 - 4) + 1; i++)
    {
        checkSum += arr[i + 4];
    }

    arr[20] = checkSum >> 8;
    arr[21] = checkSum;

    arr[22] = 0x55;
    arr[23] = 0xBB;

    // cout << hex << slamX << ", ";
    // cout << hex << (int)arr[4] << ", ";
    // cout << hex << (int)arr[5] << ", ";
    // cout << hex << ((int)arr[4] << 8) + arr[5] << ", ";
    // cout << dec << endl;

}

/**
 * @description: 将数组转换为位姿数据
 * @param {CP_DATA} SLAM 和底盘的位姿 {unsigned char} 输入的字节数组
 * @return: 数据较验是否成功
 */
bool Arr2PosData(CP_DATA &get, unsigned char (&arr)[36])
{
    if(arr[0] != 0x55)
        return false;
    if(arr[1] != 0x55)
        return false;
    if(arr[2] != 0xAA)
        return false;
    if(arr[3] != 0xAA)
        return false;

    if(arr[22] != 0x55)
        return false;
    if(arr[23] != 0xBB)
        return false;
    
    int checkSum = 0;
    for(int i =0; i < (19 - 4) + 1; i++)
    {
        checkSum += arr[i + 4];
    }

    int checkSum2Arr = arr[20] << 8 | arr[21];

    if(checkSum != checkSum2Arr)
    {
        cout << "checkSum:" << checkSum << "!=" << checkSum2Arr << endl;
        return false;
    }
    
    // cout << "check Pass" << endl;
    get.SLAM.x = CovUChar2Int(arr[4], arr[5]);
    get.SLAM.y = CovUChar2Int(arr[6], arr[7]);
    get.SLAM.z = CovUChar2Int(arr[8], arr[9]);
    get.SLAM.yaw = CovUChar2Int(arr[10], arr[11]);

    get.Chassis.x = CovUChar2Int(arr[12], arr[13]);
    get.Chassis.y = CovUChar2Int(arr[14], arr[15]);
    get.Chassis.yaw = CovUChar2Int(arr[16], arr[17]);

    get.SLAM.status = arr[18];
    get.Chassis.status = arr[19];
    

    return true;
}

/**
 * @description: 显示位姿数据
 * @param {CP_DATA} SLAM 和底盘的位姿
 * @return: 
 */
void ShowPos(CP_DATA get)
{
    cout << "SLAM X : " << get.SLAM.x << endl;
    cout << "SLAM Y : " << get.SLAM.y << endl;
    cout << "SLAM Z : " << get.SLAM.z << endl;
    cout << "SLAM Yaw : " << get.SLAM.yaw << endl;

    cout << "Chassis X : " << get.Chassis.x << endl;
    cout << "Chassis Y : " << get.Chassis.y << endl;
    cout << "Chassis Z : " << get.Chassis.z << endl;
    cout << "Chassis Yaw : " << get.Chassis.yaw << endl;

    cout << hex << "SLAM Status : " << (int)get.SLAM.status << endl;
    cout << hex << "Chassis Status : " << (int)get.Chassis.status << endl;

    cout << dec << endl;

}

/**
 * @description: 显示数组hex形态
 * @param {unsigned char} 需要显示的数组 {int}数组长度 
 * @return: 
 */
void ShowHexArr(unsigned char arr[], int len)
{
    // int len = sizeof(arr) / sizeof(int);

    for(int i = 0; i < len; i ++)
    {
        cout << hex << (int)arr[i] << ", ";
        // printf("%02X, ", arr[i]);
    }

    cout << dec << endl;
    cout << "len is " << len << endl;
}