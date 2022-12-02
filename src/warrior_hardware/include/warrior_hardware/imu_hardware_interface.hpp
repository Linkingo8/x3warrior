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
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "warrior_hardware/visibility_control.h"

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

  imu_data RM_imu_date_;
  imu_data T_imu_date_;
};

}  // namespace warrior_hardware
#endif  // WARRIOR_HARDWARE__TEST_HARDWARE_INTERFACE_HPP_
