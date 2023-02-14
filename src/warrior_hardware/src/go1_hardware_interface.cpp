
#include "warrior_hardware/go1_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <zlib.h>
#include <cstring>
#include <fcntl.h> /* File control definitions */
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace warrior_hardware
{
return_type Go1HardwareInterface::configure(
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
    Go1_commands_torques_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    Go1_commands_damp_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    Go1_commands_zero_torques_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    Go1_commands_torque_and_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    //go1 hardware interface
    Go1_port_config_ = std::make_shared<Go1Config>();
    //go1 date process and crc check
    Go1_data_process_ = std::make_shared<Go1DataProcess>(0x0000);

    /*hardware param*/
    char id_temp[1]{0};

    auto Go1_LF_ID_it = info_.hardware_parameters.find("Go1_LF_id");
    memcpy(id_temp,Go1_LF_ID_it->second.c_str(),1);
    /*char -> 16*/
    GO1_LF_ID_ = Go1_data_process_->charToHex(id_temp,1);
    RCLCPP_INFO(rclcpp::get_logger("Go1HardwareInterface"),"Go1_LF_id '%x' ",GO1_LF_ID_);

    auto Go1_LB_ID_it = info_.hardware_parameters.find("Go1_LB_id");
    memcpy(id_temp,Go1_LB_ID_it->second.c_str(),1);
    /*char -> 16*/
    GO1_LB_ID_ = Go1_data_process_->charToHex(id_temp,1);
    RCLCPP_INFO(rclcpp::get_logger("Go1HardwareInterface"),"Go1_LB_id '%x' ",GO1_LB_ID_);

    auto Go1_RF_ID_it = info_.hardware_parameters.find("Go1_RF_id");
    memcpy(id_temp,Go1_RF_ID_it->second.c_str(),1);
    /*char -> 16*/
    GO1_RF_ID_ = Go1_data_process_->charToHex(id_temp,1);
    RCLCPP_INFO(rclcpp::get_logger("Go1HardwareInterface"),"Go1_RF_id '%x' ",GO1_RF_ID_);

    auto Go1_RB_ID_it = info_.hardware_parameters.find("Go1_RB_id");
    memcpy(id_temp,Go1_RB_ID_it->second.c_str(),1);
    /*char -> 16*/
    GO1_RB_ID_ = Go1_data_process_->charToHex(id_temp,1);
    RCLCPP_INFO(rclcpp::get_logger("Go1HardwareInterface"),"Go1_LF_id '%x' ",GO1_RB_ID_);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
{
      // 3 state interfaces and 3 command interfaces on each joint
      if (joint.command_interfaces.size() != 6)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("Go1HardwareInterface"),
          "Joint '%s' has %d command interfaces. 6 expected.", joint.name.c_str());
        return return_type::ERROR;
      }

      if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
            joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
            joint.command_interfaces[0].name == "torque"||
            joint.command_interfaces[0].name == "damp"||
            joint.command_interfaces[0].name == "zero_torque"||
            joint.command_interfaces[0].name == "torque_and_position"))
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("Go1HardwareInterface"),
          "Joint '%s' has %s command interface. Expected %s, %s, %s,%s,%sor %s.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY, "torque","damp","zero_torque","torque_and_position");
          return return_type::ERROR;
      }

      if (joint.state_interfaces.size() != 3)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("Go1HardwareInterface"),
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

std::vector<hardware_interface::StateInterface> Go1HardwareInterface::export_state_interfaces()
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
Go1HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &Go1_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &Go1_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "torque",&Go1_commands_torques_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "damp",&Go1_commands_damp_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "zero_torque",&Go1_commands_zero_torques_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "torque_and_position",&Go1_commands_torque_and_position_[i]));
  }
  return command_interfaces;
}

return_type Go1HardwareInterface::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("Go1HardwareInterface"), "Starting... please wait...");
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
    if (std::isnan(Go1_commands_torques_[i]))
    {
      Go1_commands_torques_[i] = 0;
    }
    if (std::isnan(Go1_commands_damp_[i]))
    {
      Go1_commands_damp_[i] = 0;
    }
    if (std::isnan(Go1_commands_zero_torques_[i]))
    {
      Go1_commands_zero_torques_[i] = 0;
    }
    if (std::isnan(Go1_commands_torque_and_position_[i]))
    {
      Go1_commands_torque_and_position_[i] = 0;
    }
  }
  /* initilize the buff to 0 */
  Go1_data_process_->Go1_buff_zero();
  if(Go1_port_config_->open("/dev/ttyUSB0") !=  0)
  {
    RCLCPP_INFO(rclcpp::get_logger("Go1HardwareInterface"), "Go1Port hardware failed been open!");
    // init transmission.
        return hardware_interface::return_type::ERROR;    
  }
  
  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type Go1HardwareInterface::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("Go1HardwareInterface"), "Stopping... please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("Go1HardwareInterface"), "System successfully stopped!");

  return return_type::OK;
}

return_type Go1HardwareInterface::read()
{
  uint8_t buff[17]{0};
  Go1_port_config_->read_frames(buff,17);  
  uint8_t id = (buff[2] & 0xF);
  if(id<4 && buff[0] == 0xFD && buff[1]==0xEE)
  {
    Go1_data_process_->Go1_data_rec(id,buff);
    Go1_velocities_[id] = Go1_data_process_->Go1_velocities_export(id);
    Go1_positions_[id] = Go1_data_process_->Go1_positions_export(id);
    Go1_accelerations_[id] = Go1_data_process_->Go1_torques_export(id);
    // Go1_data_process_->give_id_to_go1_processor(id);
    for(int i = 0; i<4; i++)
      RCLCPP_INFO(
        rclcpp::get_logger("Go1HardwareInterface"), "Go1_positions_[%d]:%.5f",i,Go1_positions_[i]);
  }
  memset(buff,0,17);
  return return_type::OK;
}

return_type Go1HardwareInterface::write()
{
      Go1_data_process_->give_id_to_go1_processor();
    //transmit
    if(Go1_data_process_->id_now() != -1)
    {
      /*head*/
      Go1_data_process_->Go1_head_set(Go1_data_process_->id_now());
      /*id*/
      Go1_data_process_->Go1_id_set(Go1_data_process_->id_now());
      /*control data*/
      // Go1_data_process_->Go1_torque_set(0,Go1_commands_torques_[0]);
      Go1_data_process_->Go1_torque_set(Go1_data_process_->id_now(),Go1_commands_torques_[Go1_data_process_->id_now()]);
      // RCLCPP_INFO(rclcpp::get_logger("Go1HardwareInterface"), "\n\n4: Go1_commands_torques_[0] %f:.....\n\n",Go1_commands_torques_[0]);
      /*crc*/
      Go1_data_process_->Go1_crc_append(Go1_data_process_->id_now());
      /*write*/
      Go1_port_config_->write_frame(Go1_data_process_->Go1_buff_get(Go1_data_process_->id_now()),17);
      // RCLCPP_INFO(rclcpp::get_logger("Go1HardwareInterface"), "id %d",Go1_data_process_->id_now());
  }
  return return_type::OK;
}
}
  // namespace warrior_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  warrior_hardware::Go1HardwareInterface,
  hardware_interface::SystemInterface)
