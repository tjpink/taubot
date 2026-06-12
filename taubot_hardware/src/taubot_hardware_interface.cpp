// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//#include "taubot_hardware/diffbot_system.hpp"
#include "taubot_hardware/taubot_hardware_interface.hpp"

//#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace taubot_hardware
{
hardware_interface::CallbackReturn TaubotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TaubotHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TaubotHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TaubotHardwareInterface"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TaubotHardwareInterface"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TaubotHardwareInterface"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

// open the serial port at /dev/ttyAMA0
  time_ = std::chrono::system_clock::now();

  std::string uart_str("/dev/ttyAMA0");
  comms.setup(uart_str); 
  if(!comms.connected())
  {   
    std::cout << "error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TaubotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TaubotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn TaubotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("TaubotHardwareInterface"), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "Successfully activated!");

  // sent and initialise pid value of the micro controller
 char msg[43] = "set pid";
  char num_str[12];
  comms.dtoa(num_str, pid_kp, 12);
  strncat((char*) msg, num_str, sizeof(num_str)-1);
  comms.dtoa(num_str, pid_ki, 12);
  strncat((char*) msg, num_str, sizeof(num_str)-1);
  comms.dtoa(num_str, pid_kd, 12);
  strncat((char*) msg, num_str, sizeof(num_str)-1);
 
  if(!comms.serialConnection())
  {
     printf("Serial connection error");
     return hardware_interface::CallbackReturn::ERROR;
  }
  comms.sendMsg(msg, sizeof(msg));
  RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "Serial write %s to /dev/ttyAMA0 .........", msg);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TaubotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("TaubotHardwareInterface"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "Successfully deactivated!");

  comms.closeComms();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TaubotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  int enc_ticks[2];
  uint i;

  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  char msg[43] = "read";
  comms.sendMsg(msg, sizeof(msg));
  if(!comms.serialConnection())
  {
     printf("Serial connection error");
  }
  if(comms.receiveMsg(13))
  {
    RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "UART error, please   check.");
  }
  else
  {
    enc_ticks[0] = comms.getEncLeftTicks();
    enc_ticks[1] = comms.getEncRightTicks();
    std::cout << "enc_left_ticks = " << comms.getEncLeftTicks() << std::endl;
    std::cout << "enc_right_ticks = " << comms.getEncRightTicks() << std::endl;
  }

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (i=0; i<info_.joints.size(); i++)
  {
    double pos_prev = hw_positions_[i];
    hw_positions_[i] = hw_positions_[i] + enc_ticks[i] * rads_per_tick;                 // rad
    hw_velocities_[i] = (hw_positions_[i] - pos_prev) / deltaSeconds;   // rad/s

    RCLCPP_INFO(
      rclcpp::get_logger("TaubotHardwareInterface"),
      "Got position state %.5f rads and velocity state %.5f rad/s for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());

    double velocity_mpers = hw_velocities_[i] / rads_per_tick / wheel_ticks_per_meter;

    RCLCPP_INFO(
      rclcpp::get_logger("TaubotHardwareInterface"),
      "Velocity %.5f m/s for '%s'",  velocity_mpers, info_.joints[i].name.c_str());

    RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "DeltSeconds is: %.5f",   deltaSeconds);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type taubot_hardware ::TaubotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("TaubotHardwareInterface"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());

    //hw_velocities_[i] = hw_commands_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // Write command to ucontroller
  char msg[43] = {0};
  double num1 = hw_commands_[0];
  double num2 = hw_commands_[1];
  char num_str[22] = {0};
  comms.dtoa(num_str, num1, 22);
  strncat((char*) msg, num_str, sizeof(num_str)-1);
  comms.dtoa(num_str, num2, 22);
  strncat((char*) msg, num_str, sizeof(num_str)-1);
  msg[sizeof(msg)-1] = '\0';
  if(!comms.serialConnection())
  {
     printf("Serial connection error");
  }
  comms.sendMsg(msg, sizeof(msg));
  RCLCPP_INFO(rclcpp::get_logger("TaubotHardwareInterface"), "Serial write %s to /dev/ttyAMA0", msg);

  return hardware_interface::return_type::OK;
}

}  // namespace taubot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  taubot_hardware::TaubotHardwareInterface, hardware_interface::SystemInterface)
