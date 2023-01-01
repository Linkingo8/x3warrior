
#include "warrior_hardware/MF9025_hardware_interface.hpp"
#include "warrior_hardware/hardware_singleton.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <cstring>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace warrior_hardware
{
return_type MF9025HardwareInterface::configure(const hardware_interface::HardwareInfo & info_)
{
    if (configure_default(info_) != return_type::OK)
    {
      return return_type::ERROR;
    }
    /*init interface varible*/
    LK_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_commands_torque_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_torque_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    //LK date process
    MF9025_data_process_ = std::make_shared<MF9025DataProcess>();
    /*hardware param*/
    char id_temp[3]{0};
    auto MF9025_Left_ID_it_ = info_.hardware_parameters.find("left_id");
    memcpy(id_temp,MF9025_Left_ID_it_->second.c_str(),3);
    /*char -> 16*/
    MF9025_left_id_ = MF9025_data_process_->charToHex(id_temp,3);
    RCLCPP_FATAL(rclcpp::get_logger("MF9025HardwareInterface"),"left_id '%x' ",MF9025_left_id_);

    auto MF9025_Right_ID_it_ = info_.hardware_parameters.find("right_id");
    memcpy(id_temp,MF9025_Right_ID_it_->second.c_str(),3);
    MF9025_right_id_ = MF9025_data_process_->charToHex(id_temp,3);
    RCLCPP_FATAL(rclcpp::get_logger("MF9025HardwareInterface"),"right_id '%x' ",MF9025_right_id_);

    // auto imu_ID_it_ = info_.hardware_parameters.find("imu_id");
    // imu_ID_ = atoi(imu_ID_it_->second.c_str());
    /*sensor: there is one senosor so use this way of writting for the time of being*/
    // const auto & sensor_state_interfaces = info_.sensors[0].state_interfaces;
    // if (sensor_state_interfaces.size() != ANGLE_NUM)
    // {
    //   return return_type::ERROR;
    // }
    // for (const auto & imu_key : {"pitch", "yaw", "roll"})
    // {
    //   if (
    //     std::find_if(
    //       sensor_state_interfaces.begin(), sensor_state_interfaces.end(), [&imu_key](const auto & interface_info) {
    //         return interface_info.name == imu_key;
    //       }) == sensor_state_interfaces.end())
    //   {
    //     return return_type::ERROR;
    //   }
    // }
  /*joint*/
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // 3 state interfaces and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != LK_COMMOND_NUM)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MF9025HardwareInterface"),
        "Joint '%s' has %d command interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == "torque"))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MF9025HardwareInterface"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, "torque");
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != LK_STATE_NUM)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MF9025HardwareInterface"),
        "Joint '%s'has %d state interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == "torque"))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MF9025HardwareInterface"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, "torque");
      return return_type::ERROR;
    }
  }

    fprintf(stderr, "MF9025HardwareInterface configured successfully.\n");
    // initialize the angle value
    return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
MF9025HardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // const auto & sensor_name = info_.sensors[0].name;
   #ifdef RM_IMU_USE
    // state_interfaces.emplace_back(
    //   hardware_interface::StateInterface(sensor_name, "pitch", &RM_imu_date_.pitch));
    // state_interfaces.emplace_back(
    //   hardware_interface::StateInterface(sensor_name, "yaw", &RM_imu_date_.yaw));
    // state_interfaces.emplace_back(
    //   hardware_interface::StateInterface(sensor_name, "roll", &RM_imu_date_.roll));
   #endif

  #ifdef T_IMU_USE
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "pitch", &T_imu_date_.pitch));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "yaw", &T_imu_date_.yaw));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "roll", &T_imu_date_.roll));
  #endif
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &LK_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &LK_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "torque", &LK_torque_[i]));
  }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MF9025HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &LK_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &LK_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "torque",&LK_commands_torque_[i]));
  }
  return command_interfaces;
}

return_type MF9025HardwareInterface::start()
{
  RCLCPP_INFO(
  rclcpp::get_logger("MF9025HardwareInterface"), "Starting... please wait...");
  #ifdef RM_IMU_USE
  MF9025HardwareInterface::ind_ = 0;
  // Set some default values
  if (std::isnan(RM_imu_date_.pitch))
  {
    RM_imu_date_.pitch = 0;
  }
  if (std::isnan(RM_imu_date_.yaw))
  {
    RM_imu_date_.yaw = 0;
  }
  if (std::isnan(RM_imu_date_.roll))
  {
    RM_imu_date_.roll = 0;
  }
  #endif

  #ifdef T_IMU_USE
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
  #endif  

  // Set some default values
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (std::isnan(LK_commands_positions_[i]))
    {
      LK_commands_positions_[i] = 0;
    }
    if (std::isnan(LK_commands_velocities_[i]))
    {
      LK_commands_velocities_[i] = 0;
    }
    if (std::isnan(LK_commands_torque_[i]))
    {
      LK_commands_torque_[i] = 0;
    }
    if (std::isnan(LK_positions_[i]))
    {
      LK_positions_[i] = 0;
    }
    if (std::isnan(LK_velocities_[i]))
    {
      LK_velocities_[i] = 0;
    }
    if (std::isnan(LK_torque_[i]))
    {
      LK_torque_[i] = 0;
    }
  }
if(CanConfig::config_status() == 0)
{
  can_device_num_ = VCI_FindUsbDevice2(pInfo1_);
  RCLCPP_INFO(rclcpp::get_logger("MF9025HardwareInterface"), ">>USBCAN DEVICE NUM:%d...",can_device_num_);

  if(VCI_OpenDevice(VCI_USBCAN2,0,0)!=1)//打开设备
  {
    RCLCPP_INFO(rclcpp::get_logger("MF9025HardwareInterface"), "failed to open can port");   
    return hardware_interface::return_type::ERROR;         
  }
  if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo_)==1)//读取设备序列号、版本等信息。
  {
    RCLCPP_INFO(rclcpp::get_logger("MF9025HardwareInterface"), ">>Get VCI_ReadBoardInfo success!\n");  
    RCLCPP_INFO(rclcpp::get_logger("MF9025HardwareInterface"), ">>Serial_Num:%c\n", pInfo_.str_Serial_Num[0]); 
  }
  else
  {
    printf(">>Get VCI_ReadBoardInfo error!\n");
    return hardware_interface::return_type::ERROR;
  }
  
  //初始化参数，严格参数二次开发函数库说明书。
  VCI_INIT_CONFIG config;
  config.AccCode=0;
  config.AccMask=0xFFFFFFFF;
  config.Filter=1;//接收所有帧
  config.Timing0=0x00;/*波特率1000 Kbps  0x00  0x14*/
  config.Timing1=0x14;
  config.Mode=0;//正常模式

  if(VCI_InitCAN(VCI_USBCAN2,0,0,&config) != 1)
  {
    RCLCPP_INFO(rclcpp::get_logger("MF9025HardwareInterface"), ">>Init CAN1 error\n\n"); 
    VCI_CloseDevice(VCI_USBCAN2,0);
  }

  if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
  {
    RCLCPP_INFO(rclcpp::get_logger("MF9025HardwareInterface"), ">>Start CAN1 error\n\n"); 
    VCI_CloseDevice(VCI_USBCAN2,0);
  }
}
  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("MF9025HardwareInterface"), "Sensor successfully started!");
  return return_type::OK;
}

return_type MF9025HardwareInterface::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("MF9025HardwareInterface"), "Stopping... please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("MF9025HardwareInterface"), "Imu successfully stopped!");

  return return_type::OK;
}

return_type MF9025HardwareInterface::read()
{
    if((reclen_=VCI_Receive(VCI_USBCAN2,0,ind_,rec_,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(int q1=0;q1<reclen_;q1++)
			{
        // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Index:%04d  ",count_);count_++;
        // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "CAN%d RX ID:0x%08X", ind_+1, rec_[q1].ID);
        // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"),"DLC:0x%02X",rec_[q1].DataLen);//帧长度
        switch(rec_[q1].ID)
        {
          case LEFT_ID:
          {
            MF9025_data_process_->MF9025_message_rec(rec_,q1);
            LK_velocities_[0] = MF9025_data_process_->MF9025_velocitise_export(rec_,q1);
            LK_positions_[0] = MF9025_data_process_->MF9025_position_export(rec_,q1);
            LK_torque_[0] = MF9025_data_process_->MF9025_torque_export(rec_,q1);
            RCLCPP_INFO(
              rclcpp::get_logger("MF9025HardwareInterface"), "LEFT_ID speed:%f",LK_velocities_[0]);
            break;
          }
          case RIGHT_ID:
          {
            MF9025_data_process_->MF9025_message_rec(rec_,q1);
            LK_velocities_[1] = MF9025_data_process_->MF9025_velocitise_export(rec_,q1);
            LK_positions_[1] = MF9025_data_process_->MF9025_position_export(rec_,q1);
            LK_torque_[1] = MF9025_data_process_->MF9025_torque_export(rec_,q1);
            break;
          }
        }
			}
		}
    ind_=!ind_;//变换通道号，以便下次读取另一通道，交替读取。	

  return return_type::OK;
}

return_type MF9025HardwareInterface::write()
{
  LK_commands_velocities_[0] = 2000;
  MF9025_data_process_->MF9025_speed_set(1,LK_commands_velocities_[0]);
  // rclcpp::get_logger(("MF9025HardwareInterface"), "LEFT_ID speed set: %f",LK_commands_velocities_[0]);
  MF9025_data_process_->MF9025_commond_send(LEFT_ID);
  return return_type::OK;
}
}  // namespace warrior_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  warrior_hardware::MF9025HardwareInterface,
  hardware_interface::SystemInterface)
