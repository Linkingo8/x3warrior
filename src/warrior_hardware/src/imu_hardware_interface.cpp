
#include "warrior_hardware/imu_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "warrior_hardware/can_driver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace warrior_hardware
{
return_type ImuHardwareInterface::configure(const hardware_interface::HardwareInfo & info_)
{
    if (configure_default(info_) != return_type::OK)
    {
      return return_type::ERROR;
    }
    /*init interface varible*/
    LK_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_commands_moment_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    LK_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    /*hardware param*/
    // auto imu_ID_it_ = info_.hardware_parameters.find("imu_id");
    // imu_ID_ = atoi(imu_ID_it_->second.c_str());
    /*sensor: there is one senosor so use this way of writting for the time of being*/
    const auto & sensor_state_interfaces = info_.sensors[0].state_interfaces;
    if (sensor_state_interfaces.size() != ANGLE_NUM)
    {
      return return_type::ERROR;
    }
    for (const auto & imu_key : {"pitch", "yaw", "roll"})
    {
      if (
        std::find_if(
          sensor_state_interfaces.begin(), sensor_state_interfaces.end(), [&imu_key](const auto & interface_info) {
            return interface_info.name == imu_key;
          }) == sensor_state_interfaces.end())
      {
        return return_type::ERROR;
      }
    }
  /*joint*/
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // 3 state interfaces and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != LK_COMMOND_NUM)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CanHardwareInterface"),
        "Joint '%s' has %d command interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == "moment"))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CanHardwareInterface"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, "moment");
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != LK_STATE_NUM)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CanHardwareInterface"),
        "Joint '%s'has %d state interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CanHardwareInterface"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return return_type::ERROR;
    }
  }

    fprintf(stderr, "CanHardwareInterface configured successfully.\n");
    // initialize the angle value
    return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
ImuHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    const auto & sensor_name = info_.sensors[0].name;
   #ifdef RM_IMU_USE
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "pitch", &RM_imu_date_.pitch));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "yaw", &RM_imu_date_.yaw));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(sensor_name, "roll", &RM_imu_date_.roll));
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
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &LK_accelerations_[i]));
  }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ImuHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &LK_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &LK_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "moment",&LK_commands_moment_[i]));
  }
  return command_interfaces;
}

return_type ImuHardwareInterface::start()
{
  RCLCPP_INFO(
  rclcpp::get_logger("ImuHardwareInterface"), "Starting... please wait...");
  #ifdef RM_IMU_USE
  ImuHardwareInterface::ind_ = 0;
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
    if (std::isnan(LK_commands_moment_[i]))
    {
      LK_commands_moment_[i] = 0;
    }
    if (std::isnan(LK_positions_[i]))
    {
      LK_positions_[i] = 0;
    }
    if (std::isnan(LK_velocities_[i]))
    {
      LK_velocities_[i] = 0;
    }
    if (std::isnan(LK_accelerations_[i]))
    {
      LK_accelerations_[i] = 0;
    }
  }

  can_device_num_ = VCI_FindUsbDevice2(pInfo1_);
  RCLCPP_INFO(rclcpp::get_logger("ImuHardwareInterface"), ">>USBCAN DEVICE NUM:%d...",can_device_num_);

  if(VCI_OpenDevice(VCI_USBCAN2,0,0)!=1)//打开设备
  {
    RCLCPP_INFO(rclcpp::get_logger("ImuHardwareInterface"), "failed to open can port");   
    return hardware_interface::return_type::ERROR;         
  }
  if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo_)==1)//读取设备序列号、版本等信息。
  {
    RCLCPP_INFO(rclcpp::get_logger("ImuHardwareInterface"), ">>Get VCI_ReadBoardInfo success!\n");  
    RCLCPP_INFO(rclcpp::get_logger("ImuHardwareInterface"), ">>Serial_Num:%c\n", pInfo_.str_Serial_Num[0]); 
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
    RCLCPP_INFO(rclcpp::get_logger("ImuHardwareInterface"), ">>Init CAN1 error\n\n"); 
    VCI_CloseDevice(VCI_USBCAN2,0);
  }

  if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
  {
    RCLCPP_INFO(rclcpp::get_logger("ImuHardwareInterface"), ">>Start CAN1 error\n\n"); 
    VCI_CloseDevice(VCI_USBCAN2,0);
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("ImuHardwareInterface"), "Sensor successfully started!");
  return return_type::OK;
}

return_type ImuHardwareInterface::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("ImuHardwareInterface"), "Stopping... please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("ImuHardwareInterface"), "Imu successfully stopped!");

  return return_type::OK;
}

return_type ImuHardwareInterface::read()
{
   #ifdef RM_IMU_USE
    if((reclen_=VCI_Receive(VCI_USBCAN2,0,ind_,rec_,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(int q1=0;q1<reclen_;q1++)
			{
        // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Index:%04d  ",count_);count_++;
        // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "CAN%d RX ID:0x%08X", ind_+1, rec_[q1].ID);
        // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"),"DLC:0x%02X",rec_[q1].DataLen);//帧长度
        switch(rec_[q1].ID)
        {
          case IMU_PARAM_ID:
          {
            rm_imu_data.accel_rangle = rec_[q1].Data[0] &0x0F;
            rm_imu_data.gyro_rangle = (rec_[q1].Data[0] &0xF0) >> 4;
            rm_imu_data.sensor_control_temperature = rec_[q1].Data[2];
            rm_imu_data.imu_sensor_rotation = rec_[q1].Data[3] & 0x1F;
            rm_imu_data.ahrs_rotation_sequence = (rec_[q1].Data[3] & 0xE0) >> 5;
            rm_imu_data.quat_euler = rec_[q1].Data[4] & 0x01;
            switch(rm_imu_data.gyro_rangle)
            {
                case 0: rm_imu_data.gyro_sen = GYRO_2000_SEN; break;
                case 1: rm_imu_data.gyro_sen = GYRO_1000_SEN; break;
                case 2: rm_imu_data.gyro_sen = GYRO_500_SEN;  break;
                case 3: rm_imu_data.gyro_sen = GYRO_250_SEN;  break;
                case 4: rm_imu_data.gyro_sen = GYRO_125_SEN;  break;
            }
            switch(rm_imu_data.accel_rangle)
            {
                case 0: rm_imu_data.accel_sen = ACCEL_3G_SEN;  break;
                case 1: rm_imu_data.accel_sen = ACCEL_6G_SEN;  break;
                case 2: rm_imu_data.accel_sen = ACCEL_12G_SEN; break;
                case 3: rm_imu_data.accel_sen = ACCEL_24G_SEN; break;
            }
            break;
          }
		      case IMU_QUAT_ID:
          {
            if(rm_imu_data.quat_euler && rec_[q1].DataLen == 6)
            {
              memcpy(rm_imu_data.euler_angle, &rec_[q1].Data[0], rec_[q1].DataLen);
              rm_imu_data.euler_angle_fp32[0] = rm_imu_data.euler_angle[0] * 0.0001f;
              rm_imu_data.euler_angle_fp32[1] = rm_imu_data.euler_angle[1] * 0.0001f;
              rm_imu_data.euler_angle_fp32[2] = rm_imu_data.euler_angle[2] * 0.0001f;
            }
            else if(rm_imu_data.quat_euler == 0 && rec_[q1].DataLen == 8)
            {

              memcpy(rm_imu_data.quat, &rec_[q1].Data[0], rec_[q1].DataLen);                   // received datas were magnified 10000 timnes
              rm_imu_data.quat_fp32[0] = rm_imu_data.quat[0] * 0.0001f;          // 10000 times smaller
              rm_imu_data.quat_fp32[1] = rm_imu_data.quat[1] * 0.0001f;          // 10000 times smaller
              rm_imu_data.quat_fp32[2] = rm_imu_data.quat[2] * 0.0001f;          // 10000 times smaller
              rm_imu_data.quat_fp32[3] = rm_imu_data.quat[3] * 0.0001f;          // 10000 times smaller 
            }
            break;
          }
          case IMU_GYRO_ID:
          { 
              memcpy(rm_imu_data.gyro_int16, &rec_[q1].Data[0],6);
              rm_imu_data.gyro_fp32[0] = rm_imu_data.gyro_int16[0] * rm_imu_data.gyro_sen;
              rm_imu_data.gyro_fp32[1] = rm_imu_data.gyro_int16[1] * rm_imu_data.gyro_sen;
              rm_imu_data.gyro_fp32[2] = rm_imu_data.gyro_int16[2] * rm_imu_data.gyro_sen;
              rm_imu_data.sensor_temperature = (int16_t)((rec_[q1].Data[6] << 3) | (rec_[q1].Data[7] >> 5));
              if (rm_imu_data.sensor_temperature > 1023)
              {
              rm_imu_data.sensor_temperature -= 2048;
              }
              break;
          }
          case IMU_ACCEL_ID:
          {
              memcpy(rm_imu_data.accel_int16, &rec_[q1].Data[0],6);
              rm_imu_data.accel_fp32[0] = rm_imu_data.accel_int16[0] * rm_imu_data.accel_sen;
              rm_imu_data.accel_fp32[1] = rm_imu_data.accel_int16[1] * rm_imu_data.accel_sen;
              rm_imu_data.accel_fp32[2] = rm_imu_data.accel_int16[2] * rm_imu_data.accel_sen;
              memcpy(&rm_imu_data.sensor_time, (&rec_[q1].Data[0] + 6), 2);
              break;
          }
          case IMU_MAG_ID:
          {
             memcpy(rm_imu_data.mag_int16, &rec_[q1].Data[0],6);
             break;
          }
        }
        // if(rec_[q1].ExternFlag==0) RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," Standard ");//帧格式：标准帧
        // if(rec_[q1].ExternFlag==1) RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," Extend   ");//帧格式：扩展帧
        // if(rec_[q1].RemoteFlag==0) RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," Data   ");//帧类型：数据帧
        // if(rec_[q1].RemoteFlag==1) RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," Remote ");//帧类型：远程帧

        // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," data:0x");//帧长度
        // for(int w1 = 0; w1 < rec_[q1].DataLen; w1++)
        // {
        //   RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," %02X", rec_[q1].Data[w1]);//帧长度
        // }
        // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," TimeStamp:0x%08X",rec_[q1].TimeStamp);//时间标识。
			}
      RM_imu_date_.yaw   = atan2f(rm_imu_data.quat_fp32[0]*rm_imu_data.quat_fp32[3]+rm_imu_data.quat_fp32[1]*rm_imu_data.quat_fp32[2],
                              rm_imu_data.quat_fp32[0]*rm_imu_data.quat_fp32[0]+rm_imu_data.quat_fp32[1]*rm_imu_data.quat_fp32[1]-0.5f)*100.f;
      RM_imu_date_.pitch = asinf(2*(rm_imu_data.quat_fp32[0]*rm_imu_data.quat_fp32[2]-rm_imu_data.quat_fp32[1]*rm_imu_data.quat_fp32[3]))*100.f;
      RM_imu_date_.roll  = atan2f(rm_imu_data.quat_fp32[0]*rm_imu_data.quat_fp32[1]+rm_imu_data.quat_fp32[2]*rm_imu_data.quat_fp32[3],
                              rm_imu_data.quat_fp32[0]*rm_imu_data.quat_fp32[0]+rm_imu_data.quat_fp32[3]*rm_imu_data.quat_fp32[3]-0.5f)*100.f;
		}
    ind_=!ind_;//变换通道号，以便下次读取另一通道，交替读取。	
   #endif

  
  #ifdef T_IMU_USE
    T_imu_date_.pitch = 1;
    T_imu_date_.yaw = 2;
    T_imu_date_.roll = 3;
  #endif
    // RCLCPP_INFO(
    // rclcpp::get_logger("ImuHardwareInterface"), "Get pitch %.5f,yaw %.5f,roll %.5f",RM_imu_date_.pitch,RM_imu_date_.yaw,RM_imu_date_.roll);

  return return_type::OK;
}

return_type ImuHardwareInterface::write()
{
  speedControl_LK_L_ = LK_commands_positions_[0];
  send_9025_[0].ID=0x141;
	send_9025_[0].SendType=0;
	send_9025_[0].RemoteFlag=0;
	send_9025_[0].ExternFlag=0;
	send_9025_[0].DataLen=8;
  send_9025_[0].Data[0] = 0xA2;
  send_9025_[0].Data[1] = 0x00;
  send_9025_[0].Data[2] = 0x00;
  send_9025_[0].Data[3] = 0x00;

  send_9025_[0].Data[4] = *(uint8_t *)(&speedControl_LK_L_);
  send_9025_[0].Data[5] = *((uint8_t *)(&speedControl_LK_L_)+1);
  send_9025_[0].Data[6] = *((uint8_t *)(&speedControl_LK_L_)+2);
  send_9025_[0].Data[7] = *((uint8_t *)(&speedControl_LK_L_)+3);

  if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_9025_, 1) == 1)
  {
    printf("CAN1 TX ID:0x%08X",send_9025_[0].ID);
    if(send_9025_[0].ExternFlag==0) printf(" Standard ");
    if(send_9025_[0].ExternFlag==1) printf(" Extend   ");
    if(send_9025_[0].RemoteFlag==0) printf(" Data   ");
    if(send_9025_[0].RemoteFlag==1) printf(" Remote ");
    printf("DLC:0x%02X",send_9025_[0].DataLen);
    printf(" data:0x");
    printf("\n");
  }
  // for (std::size_t i = 0; i < LK_commands_positions_.size(); i++)
  // {
  //  RCLCPP_INFO(
  //     rclcpp::get_logger("CanHardwareInterface"),
  //     "Got the commands pos: %.5f",
  //     LK_commands_positions_[i]);
  // }
  return return_type::OK;
}
}  // namespace warrior_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  warrior_hardware::ImuHardwareInterface,
  hardware_interface::SystemInterface)
