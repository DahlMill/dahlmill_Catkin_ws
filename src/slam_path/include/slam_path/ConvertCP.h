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

void PosData2Arr(CP_DATA &post, unsigned char (&arr)[24]);
bool Arr2PosData(CP_DATA &get, unsigned char (&arr)[36]);