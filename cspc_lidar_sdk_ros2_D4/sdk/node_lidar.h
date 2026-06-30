#ifndef NODE_LIDAR_H
#define NODE_LIDAR_H

#include <stdint.h>
#include <vector>
#include <string>
#include <atomic>
#include "point_cloud_optimize.h"
#include "lidar_data_processing.h"
#include "serial_port.h"
#include "locker.h"
#include "timer.h"
#include <memory>
#include <iostream>


//#include <ros/ros.h>
//#include <sensor_msgs/LaserScan.h>

#include <iostream>

using namespace std;

/*Lidar basic information*/
typedef struct{
  int version = 4; //Lidar version
  string port = "/dev/ttyUSB0";     //Serial port name
  int m_SerialBaudrate = 230400;   //Baud rate
  bool m_intensities = false;
  uint64_t m_PointTime = 1e9/3800; 
  uint32_t trans_delay = 0;
  string frame_id = "laser_link";
  uint16_t frequency_max = 103;
  uint16_t frequency_min = 97;
}lidar_general_info_t;

/*Lidar occlusion judgment structure*/
typedef struct{
  uint16_t point_check = 0;
  uint16_t point_check_part = 0;
  uint16_t point_check_all = 0;
  int blocked_size = 0;
  int lidar_zero_count = 0;
  bool blocked_judge = true;
}lidar_block_t;

/*Lidar raw data packet*/
typedef struct{
  node_package package;
  node_packages packages;
  node_package_coin package_coin;
}lidar_package_t;

/*Lidar time*/
typedef struct{
  uint32_t scan_time_t = 0;      
  uint64_t scan_time_current = 0;
  uint64_t scan_time_record = 0;
  uint64_t system_start_time = 0;
  uint64_t last_encry_time = 0;
  uint64_t tim_scan_start = 0;
  uint64_t tim_scan_end = 0;
  uint64_t scan_start_time = 0;
  uint64_t scan_end_time = 0;
  uint64_t lidar_frequence_abnormal_time = 0;
}lidar_time_t;

/*Robot structure information*/
typedef struct{
  float install_to_zero = 90.0;
  int ROBOT_DIAMETER_mm;               //Sweeper diameter
  int LIDAR_ROBOT_CENTER_DISTANCE_mm;  //Distance from sweeper lidar installation position to center
  int LidarCoverBarNumber = 0;//Lidar gap elimination value
  vector<LidarCoverAngleStr> LidarCoverAngle;
}lidar_robot_info_t;

/*Lidar node operating status*/
typedef struct{
  bool FilterEnable = true;           //Whether filtering is needed
  bool GyroCompensateEnable = false; //Whether to do rotation angle compensation
  bool isConnected = false;          //Serial port connection status
  bool slam_user = false;
  bool encry_lidar = false;
  bool optimize_enable = true;
  bool lidar_cover = false;
  bool disable_encry = false;
  bool able_exposure = false;
  bool low_exposure = false;
  bool lidar_ready = false;
  bool lidar_last_status =false;
  bool close_lidar = true;
  bool lidar_trap_restart = false;    //Lidar stuck restart status

  bool lidar_restart_try = false;


  uint8_t lidar_abnormal_state = 0; 
}lidar_status_t;

struct node_lidar_t
{

  node_info *scan_node_buf;      //Store laser lidar point information
  uint8_t *globalRecvBuffer;     //Store received lidar data
  lidar_block_t lidar_block;     //Lidar occlusion judgment

  lidar_package_t scan_packages; //Lidar data packets of different versions

  lidar_time_t lidar_time;       //Lidar time

  lidar_general_info_t lidar_general_info; //Lidar basic information

  lidar_robot_info_t lidar_robot_info;     //Information about lidar position relative to robot installation

  lidar_status_t lidar_status;             //Lidar operating status

  size_t scan_node_count = 0;	//Number of laser points
  
  uint8_t recv_encryp[8];
  uint8_t lidar_version[8];

  Point_cloud_optimize optimize_lidar;
  Lidar_Data_Processing lidar_data_processing;
  //Serial_Port *serial_port;
  std::shared_ptr<Serial_Port> serial_port;
  Event _dataEvent;
  Locker _lock;

  bool data_calibration = false;
  float range_max;

  ~node_lidar_t();
};

extern node_lidar_t node_lidar;

bool data_handling(LaserScan &outscan);
bool initialize(); 
int node_start();
void send_lidar_data(LaserScan &outscan);
void flushSerial();
result_t grabScanData(uint32_t timeout = DEFAULT_TIMEOUT);
result_t connect(const char *port_path, uint32_t baudrate,node_lidar_t &lidar_arg);


#endif
