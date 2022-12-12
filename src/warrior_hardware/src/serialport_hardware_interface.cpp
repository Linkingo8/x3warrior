
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
  Go1_commands_damp_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  Go1_commands_zero_moments_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  Go1_commands_moment_and_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // 3 state interfaces and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 6)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SerialPorttHardwareInterface"),
        "Joint '%s' has %d command interfaces. 6 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == "moment"||
          joint.command_interfaces[0].name == "damp"||
          joint.command_interfaces[0].name == "zero_moment"||
          joint.command_interfaces[0].name == "moment_and_position"))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SerialPorttHardwareInterface"),
        "Joint '%s' has %s command interface. Expected %s, %s, %s,%s,%sor %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, "moment","damp","zero_moment","moment_and_position");
        return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SerialPorttHardwareInterface"),
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
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "damp",&Go1_commands_damp_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "zero_moment",&Go1_commands_zero_moments_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "moment_and_position",&Go1_commands_moment_and_position_[i]));
  }
  return command_interfaces;
}

return_type SerialPorttHardwareInterface::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("SerialPorttHardwareInterface"), "Starting... please wait...");
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
    if (std::isnan(Go1_commands_damp_[i]))
    {
      Go1_commands_damp_[i] = 0;
    }
    if (std::isnan(Go1_commands_zero_moments_[i]))
    {
      Go1_commands_zero_moments_[i] = 0;
    }
    if (std::isnan(Go1_commands_moment_and_position_[i]))
    {
      Go1_commands_moment_and_position_[i] = 0;
    }
  }
  //go1 hardware interface
  Go1_port_config_ = std::make_shared<Go1Config>();
  //go1 date process and crc check
  Go1_data_process_ = std::make_shared<Go1DataProcess>(0x0000);
  if(Go1_port_config_->open("/dev/ttyUSB0") !=  0)
  {
    RCLCPP_INFO(rclcpp::get_logger("SerialPorttHardwareInterface"), "Go1Port hardware failed been open!");
        return hardware_interface::return_type::ERROR;    
  }
  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type SerialPorttHardwareInterface::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("SerialPorttHardwareInterface"), "Stopping... please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("SerialPorttHardwareInterface"), "System successfully stopped!");

  return return_type::OK;
}

return_type SerialPorttHardwareInterface::read()
{
  uint8_t buff[17]{0};
  // RCLCPP_INFO(
  // rclcpp::get_logger("SerialPorttHardwareInterface"), "reading.....");
  return return_type::OK;
}

return_type SerialPorttHardwareInterface::write()
{
  /*Iterate through the commond interface to decide the control mode*/
    /*head*/
    Go1_data_process_->Go1_head_set();
    Go1_data_process_->Go1_id_set();
    for (uint8_t i = 0; i < GO1_NUM; i++)
    {
      /* code */
      Go1_data_process_->Go1_speed_set(i,0.05f,6.2686f);
    }
    Go1_data_process_->Go1_crc_append();
    Go1_data_process_->Go1_buff_print();
    
    uint8_t buff[17] = {0};
    uint8_t read_buff[16] = {0};
    
    buff[0] = 0xFE;
    buff[1] = 0xEE;
    //包头
    buff[2] = 0x10;//0000 0010
    //ID 模式 设置
    buff[3] = 0x00;
    buff[4] = 0x00;
    //前馈力矩等于0
    buff[5] = 0xFF;//
    buff[6] = 0x00;
    //速度值为36
    buff[7] = 0x00;
    buff[8] = 0x00;
    buff[9] = 0x00;
    buff[10] = 0x00;
    //与位置无关，全部赋值为0

    buff[11] = 0x00;
    buff[12] = 0x00;
    //刚度系数等于0
    buff[13] = 0x40;
    buff[14] = 0x00;
   
   Go1_port_config_->write_frame(Go1_data_process_->Go1_buff_get(0),17);
  //  for(int i = 0; i < 17; i++)
  //  {
  //   RCLCPP_INFO(
  //  rclcpp::get_logger("SerialPorttHardwareInterface"), "writing...buff[%d],%x",i,buff[i]);

  //  }
  //   RCLCPP_INFO(
  //  rclcpp::get_logger("SerialPorttHardwareInterface"), "writing...");
  return return_type::OK;
}

}  // namespace warrior_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  warrior_hardware::SerialPorttHardwareInterface,
  hardware_interface::SystemInterface)
