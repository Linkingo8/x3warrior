
#include "warrior_hardware/serialport_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <zlib.h>

#include <fcntl.h> /* File control definitions */
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace warrior_hardware
{
return_type SerialPorttHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  Go1_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  Go1_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  Go1_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  Go1_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  Go1_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  Go1_commands_moments_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // 3 state interfaces and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestHardwareInterface"),
        "Joint '%s' has %d command interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == "moment"))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestHardwareInterface"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, "moment");
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestHardwareInterface"),
        "Joint '%s'has %d state interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestHardwareInterface"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> SerialPorttHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &Go1_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &Go1_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &Go1_accelerations_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SerialPorttHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &Go1_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &Go1_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "moment",&Go1_commands_moments_[i]));
  }
  return command_interfaces;
}

return_type SerialPorttHardwareInterface::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("TestHardwareInterface"), "Starting... please wait...");

  // Set some default values
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (std::isnan(Go1_positions_[i]))
    {
      Go1_positions_[i] = 0;
    }
    if (std::isnan(Go1_velocities_[i]))
    {
      Go1_velocities_[i] = 0;
    }
    if (std::isnan(Go1_accelerations_[i]))
    {
      Go1_accelerations_[i] = 0;
    }
    if (std::isnan(Go1_commands_positions_[i]))
    {
      Go1_commands_positions_[i] = 0;
    }
    if (std::isnan(Go1_commands_velocities_[i]))
    {
      Go1_commands_velocities_[i] = 0;
    }
    if (std::isnan(Go1_commands_moments_[i]))
    {
      Go1_commands_moments_[i] = 0;
    }
  }
  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type SerialPorttHardwareInterface::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("TestHardwareInterface"), "Stopping... please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("TestHardwareInterface"), "System successfully stopped!");

  return return_type::OK;
}

return_type SerialPorttHardwareInterface::read()
{
  return return_type::OK;
}

return_type SerialPorttHardwareInterface::write()
{
  return return_type::OK;
}

}  // namespace warrior_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  warrior_hardware::SerialPorttHardwareInterface,
  hardware_interface::SystemInterface)
