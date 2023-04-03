#ifndef WARRIOR_HARDWARE__SERIALPORT_HARDWARE_INTERFACE_HPP_
#define WARRIOR_HARDWARE__SERIALPORT_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "warrior_hardware/crc.hpp"
#include "warrior_hardware/visibility_control.h"
#include "warrior_hardware/can_driver.hpp"
#include "warrior_hardware/hardware_singleton.hpp"

using hardware_interface::return_type;

namespace warrior_hardware
{

#define ANGLE_NUM 3
#define RM_IMU_USE
//#define T_IMU_USE

class ImuHardwareInterface
: public hardware_interface::BaseInterface<hardware_interface::SensorInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ImuHardwareInterface);

  WARRIOR_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  WARRIOR_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  WARRIOR_HARDWARE_PUBLIC
  return_type start() override;

  WARRIOR_HARDWARE_PUBLIC
  return_type stop() override;

  WARRIOR_HARDWARE_PUBLIC
  return_type read() override;

private:
  struct imu_data
  {
    double pitch = 0.0;
    double yaw   = 0.0;
    double roll  = 0.0;
  };

  typedef struct
  {
      uint8_t quat_euler:1;
      uint8_t gyro_rangle:3;
      uint8_t accel_rangle:2;
      uint8_t imu_sensor_rotation:5;
      uint8_t ahrs_rotation_sequence:3;
      int16_t quat[4];
      float quat_fp32[4];
      int16_t euler_angle[3];
      float euler_angle_fp32[3];
      int16_t gyro_int16[3];
      int16_t accel_int16[3];
      int16_t mag_int16[3];
      float gyro_fp32[3];
      float accel_fp32[3];
      uint16_t sensor_time;
      uint16_t sensor_temperature;
      int16_t sensor_control_temperature;
      float gyro_sen;
      float accel_sen;
      uint8_t data_ready_flag;
  }rm_imu_data_t;
  /*imu*/
  rm_imu_data_t rm_imu_data;
  imu_data RM_imu_date_;
  imu_data T_imu_date_;
  std::string imu_ID_String_;
  int16_t imu_ID_;
  /*usb -can*/
  int can_device_num_;
  VCI_BOARD_INFO pInfo1_[50];
  VCI_BOARD_INFO pInfo_;//用来获取设备信息。
  int reclen_;
  int ind_;
  int count_;
  VCI_CAN_OBJ rec_[3000];//接收缓存，设为3000为佳。

};

}  // namespace warrior_hardware
#endif  // WARRIOR_HARDWARE__TEST_HARDWARE_INTERFACE_HPP_
