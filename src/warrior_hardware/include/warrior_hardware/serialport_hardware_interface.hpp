#ifndef WARRIOR_HARDWARE__SERIALPORT_HARDWARE_INTERFACE_HPP_
#define WARRIOR_HARDWARE__SERIALPORT_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "warrior_hardware/go1_config.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "warrior_hardware/visibility_control.h"

using hardware_interface::return_type;

namespace warrior_hardware
{
class SerialPorttHardwareInterface
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SerialPorttHardwareInterface);

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

  std::vector<double> Go1_commands_positions_;
  std::vector<double> Go1_commands_velocities_;
  std::vector<double> Go1_commands_moments_;
  std::vector<double> Go1_positions_;
  std::vector<double> Go1_velocities_;
  std::vector<double> Go1_accelerations_;
  std::shared_ptr<go1_config> Go1_port_config_;
};

}  // namespace warrior_hardware
#endif  // WARRIOR_HARDWARE__TEST_HARDWARE_INTERFACE_HPP_
