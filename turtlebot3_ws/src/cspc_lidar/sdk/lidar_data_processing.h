#ifndef LIDAR_DATA_PROCESSING
#define LIDAR_DATA_PROCESSING

#include <stdint.h>
#include "lidar_information.h"

class Lidar_Data_Processing
{

private:
  uint16_t CheckSumCal;
  uint16_t CheckSum;		         //Checksum
  uint16_t SampleNumlAndCTCal;
  uint16_t LastSampleAngleCal;
  uint16_t Valu8Tou16;
  uint16_t package_Sample_Index; //Package sample point index
  uint16_t FirstSampleAngle;     //Starting sample angle
  uint16_t LastSampleAngle;      //Ending sample angle
  uint8_t scan_frequence;	       //Lidar rotation speed in protocol
  bool CheckSumResult;
  bool has_package_error;
  float IntervalSampleAngle;
  float IntervalSampleAngle_LastPackage;
  int package_index;

  float start_t =0;
  float stop_t = 0;
  float angle_new = 0;
  float angle_bak = 0;
  size_t recvNodeCount;

  uint64_t m_node_time_ns;			 //< time stamp
  uint64_t m_node_last_time_ns;  //< time stamp
  uint32_t m_pointTime;				   //< two laser point time intervals
  size_t buffer_size = 0;

public:

  Lidar_Data_Processing();
  ~Lidar_Data_Processing();

  int PackageSampleBytes;   //Number of laser points contained in one package

  /*Send command to laser lidar*/
  //result_t sendCommand(uint8_t cmd,const void *payload = NULL,size_t payloadsize = 0);
  result_t sendCommand(uint8_t cmd);
  /*Write data to laser lidar*/
  result_t sendData(const uint8_t *data, size_t size);

  /*Receive message from laser lidar*/
  result_t waitResponseHeader(uint8_t cmd,uint64_t timeout = DEFAULT_TIMEOUT);

  /*Wait for laser lidar speed adjustment to complete*/
  result_t waitSpeedRight(uint8_t cmd,uint64_t timeout = DEFAULT_TIMEOUT);

  /*Receive lidar data uploaded from serial port*/
  result_t waitScanData(node_info *nodebuffer, size_t &count,uint32_t timeout = DEFAULT_TIMEOUT);

  /*Parse received lidar data points (v1 and v2 versions)*/
  result_t waitPackage(node_info *node,uint32_t timeout = DEFAULT_TIMEOUT);

  /*Parse received lidar data points (M1CT_coin_plus)*/
  result_t waitPackage_coin(node_info *node,uint32_t timeout = DEFAULT_TIMEOUT);

  bool wait_start_reply(uint64_t timeout = DEFAULT_TIMEOUT);
};

#endif
