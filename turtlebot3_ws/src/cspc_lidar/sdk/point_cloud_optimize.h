#ifndef POINT_CLOUD_OPTIMIZE_H
#define POINT_CLOUD_OPTIMIZE_H

//#include "node_lidar.h"
#include "lidar_data_processing.h"
#include <vector>
#include "lidar_information.h"

using namespace std;
class Point_cloud_optimize
{
private:
    int cnt=0;
    int cnt_all=0;
    int cnt_judge=0;
    int cnt_record=0;
    bool cumulation = false;
    short Lidar_Blocked[800];
    
public:

    /*Point cloud filtering*/
    void PointCloudFilter(LaserScan *Scan);

    /*Distance of current point minus the distance from lidar to sweeper edge at the same angle*/
    int UltrasonicSimRanging(LaserPoint &pScan);

    /*Get the angle information to be removed*/
    void getLidarCoverAngle(char *charbuf);

    /*Judgment of lidar occlusion*/
    void lidar_blocked_judge(int count);

    /*Lidar occlusion counting*/
    void lidar_blocked_count(LaserPoint &pScan,int count_lidar);

    /*Point cloud gap elimination*/
    void lidar_cover_cut(float &,float &);

    /*Data reset*/
    void datas_clear();

    void lidar_blocked_init();
    
};

#endif