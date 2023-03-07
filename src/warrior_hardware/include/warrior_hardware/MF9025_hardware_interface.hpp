#ifndef WARRIOR_HARDWARE__IMU_HARDWARE_INTERFACE_HPP_
#define WARRIOR_HARDWARE__IMU_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "warrior_hardware/visibility_control.h"
#include "warrior_hardware/can_driver.hpp"
#include "warrior_hardware/bsp_LK_MF9025.hpp"

#define ANGLE_NUM       9
#define LK_COMMOND_NUM  3
#define LK_STATE_NUM    3
#define RM_IMU_USE
#define LKMF9025_MAX_POSITION 65534
#define LKMF9025_MIN_POSITION 0

//#define T_IMU_USE

using hardware_interface::return_type;
namespace warrior_hardware
{

class MF9025HardwareInterface
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MF9025HardwareInterface);

  WARRIOR_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  WARRIOR_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  WARRIOR_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  WARRIOR_HARDWARE_PUBLIC
  return_type start() override;

  WARRIOR_HARDWARE_PUBLIC
  return_type stop() override;

  WARRIOR_HARDWARE_PUBLIC
  return_type read() override;

  WARRIOR_HARDWARE_PUBLIC
  return_type write() override;

private:
  struct imu_data
  {
    double pitch = 0.0;
    double yaw   = 0.0;
    double roll  = 0.0;
    double wx  = 0.0;
    double wy  = 0.0;
    double wz  = 0.0;
    double ax  = 0.0;
    double ay  = 0.0;
    double az  = 0.0;
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
  /*MF_9025*/
  std::vector<double> LK_commands_positions_;/*0 left 1 right*/
  std::vector<double> LK_commands_velocities_;
  std::vector<double> LK_commands_torque_;
  std::vector<double> LK_positions_;
  std::vector<double> LK_velocities_;
  std::vector<double> LK_torque_;
  uint16_t MF9025_left_id_;
  uint16_t MF9025_right_id_;
  char test_id_[3];
  /*usb -can*/
  int can_device_num_;
  VCI_BOARD_INFO pInfo1_[50];
  VCI_BOARD_INFO pInfo_;//用来获取设备信息。
  int reclen_;
  int ind_;
  int count_;
  VCI_CAN_OBJ rec_[3000];//接收缓存，设为3000为佳。
  VCI_CAN_OBJ send_9025_[2];

  int32_t speedControl_LK_L_;
  int32_t speedControl_LK_R_;
  double  speedControl_LK_L_d;

  std::shared_ptr<MF9025DataProcess> MF9025_data_process_;

};

}  // namespace warrior_hardware
#endif  // WARRIOR_HARDWARE__TEST_HARDWARE_INTERFACE_HPP_
