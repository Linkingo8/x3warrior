
#include "warrior_hardware/test_hardware_interface.hpp"

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
return_type TestHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  control_mode_.resize(info_.joints.size(), integration_level_t::POSITION);

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
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestHardwareInterface"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
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

std::vector<hardware_interface::StateInterface> TestHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_accelerations_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TestHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION,&hw_commands_accelerations_[i]));
  }
  return command_interfaces;
}


return_type TestHardwareInterface::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("TestHardwareInterface"), "Starting... please wait...");

  // Set some default values
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
    }
    if (std::isnan(hw_velocities_[i]))
    {
      hw_velocities_[i] = 0;
    }
    if (std::isnan(hw_accelerations_[i]))
    {
      hw_accelerations_[i] = 0;
    }
    if (std::isnan(hw_commands_positions_[i]))
    {
      hw_commands_positions_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i]))
    {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_accelerations_[i]))
    {
      hw_commands_accelerations_[i] = 0;
    }
    // control_mode_[i] = integration_level_t::POSITION;
    control_mode_[i] = integration_level_t::VELOCITY;
    // control_mode_[i] = integration_level_t::ACCELERATION;
  }
  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("TestHardwareInterface"), "System successfully started! %u",
    control_mode_[0]);
  return return_type::OK;
}

return_type TestHardwareInterface::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("TestHardwareInterface"), "Stopping... please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("TestHardwareInterface"), "System successfully stopped!");

  return return_type::OK;
}

return_type TestHardwareInterface::read()
{
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    switch (control_mode_[i])
    {
      case integration_level_t::UNDEFINED:
        RCLCPP_INFO(
          rclcpp::get_logger("TestHardwareInterface"),
          "Nothing is using the hardware interface!");
        return return_type::OK;
        break;
      case integration_level_t::POSITION:
        hw_accelerations_[i] = 0;
        hw_velocities_[i] = 0;
        hw_positions_[i] = hw_commands_positions_[i];
        break;
      case integration_level_t::VELOCITY:
        hw_accelerations_[i] = 0;
        hw_velocities_[i] = hw_commands_velocities_[i];
        break;
      case integration_level_t::ACCELERATION:
        hw_accelerations_[i] = hw_commands_accelerations_[i];
        break;
    }
    // RCLCPP_INFO(
    //   rclcpp::get_logger("TestHardwareInterface"),
    //   "Got pos: %.5f, vel: %.5f, acc: %.5f for joint %d!", hw_positions_[i], hw_velocities_[i],hw_accelerations_[i], i);
  }
  return return_type::OK;
}

return_type TestHardwareInterface::write()
{
  // RCLCPP_INFO(
  //   rclcpp::get_logger("TestHardwareInterface"),
  //   "Writing...");
  for (std::size_t i = 0; i < hw_commands_positions_.size(); i++)
  {
    // Simulate sending commands to the hardware
    // RCLCPP_INFO(
    //   rclcpp::get_logger("TestHardwareInterface"),
    //   "Got the commands pos: %.5f, vel: %.5f, acc: %.5f for joint %d, control_lvl: %d",
    //   hw_commands_positions_[i], hw_commands_velocities_[i], hw_commands_accelerations_[i], i,control_mode_[i]);
  }
  return return_type::OK;
}

}  // namespace warrior_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  warrior_hardware::TestHardwareInterface,
  hardware_interface::SystemInterface)
