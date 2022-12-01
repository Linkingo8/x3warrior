
#include "warrior_hardware/imu_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace warrior_hardware
{
return_type ImuHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info_)
{
    if (configure_default(info_) != return_type::OK)
    {
      return return_type::ERROR;
    }
    const auto & state_interfaces = info_.sensors[0].state_interfaces;
    if (state_interfaces.size() != ANGLE_NUM)
    {
      return return_type::ERROR;
    }
    for (const auto & imu_key : {"pitch", "yaw", "roll"})
    {
      if (
        std::find_if(
          state_interfaces.begin(), state_interfaces.end(), [&imu_key](const auto & interface_info) {
            return interface_info.name == imu_key;
          }) == state_interfaces.end())
      {
        return return_type::ERROR;
      }
    }
    fprintf(stderr, "ImuHardwareInterface configured successfully.\n");
    // initialize the angle value
    return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
ImuHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    const auto & sensor_name = info_.sensors[0].name;

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "pitch", &T_imu_date_.pitch));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "yaw", &T_imu_date_.yaw));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "roll", &T_imu_date_.roll));

    return state_interfaces;
}

return_type ImuHardwareInterface::start()
{
  RCLCPP_INFO(
  rclcpp::get_logger("ImuHardwareInterface"), "Starting... please wait...");

  // Set some default values
  if (std::isnan(T_imu_date_.pitch))
  {
    T_imu_date_.pitch = 0;
  }
  if (std::isnan(T_imu_date_.yaw))
  {
    T_imu_date_.yaw = 0;
  }
  if (std::isnan(T_imu_date_.roll))
  {
    T_imu_date_.roll = 0;
  }
  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("ImuHardwareInterface"), "System successfully started!");
  return return_type::OK;
}

return_type ImuHardwareInterface::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("TestHardwareInterface"), "Stopping... please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("TestHardwareInterface"), "System successfully stopped!");

  return return_type::OK;
}

return_type ImuHardwareInterface::read()
{
  T_imu_date_.pitch = 1;
  T_imu_date_.yaw = 2;
  T_imu_date_.roll = 3;
  // RCLCPP_INFO(
  // rclcpp::get_logger("ImuHardwareInterface"), "Get pitch %.5f,yaw %.5f,roll %.5f",T_imu_date_.pitch,T_imu_date_.yaw,T_imu_date_.roll);
  return return_type::OK;
}

}  // namespace warrior_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  warrior_hardware::ImuHardwareInterface,
  hardware_interface::SensorInterface)
