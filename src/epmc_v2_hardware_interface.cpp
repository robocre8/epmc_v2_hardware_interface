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

#include "epmc_v2_hardware_interface/epmc_v2_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

namespace epmc_v2_hardware_interface
{
  auto epmcReadWriteTime = std::chrono::system_clock::now();
  std::chrono::duration<double> epmcReadWriteDuration;
  float epmcReadWriteTimeInterval = 0.01; // 100Hz

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &info)
  {
    if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS )
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    config_.motor0_wheel_name = info_.hardware_parameters["motor0_wheel_name"];
    config_.motor1_wheel_name = info_.hardware_parameters["motor1_wheel_name"];
    config_.motor2_wheel_name = info_.hardware_parameters["motor2_wheel_name"];
    config_.motor3_wheel_name = info_.hardware_parameters["motor3_wheel_name"];
    config_.port = info_.hardware_parameters["port"];
    config_.cmd_vel_timeout_ms = info_.hardware_parameters["cmd_vel_timeout_ms"];

    if (config_.motor0_wheel_name != "")
      motor0_.setup(config_.motor0_wheel_name);
    if (config_.motor1_wheel_name != "")
      motor1_.setup(config_.motor1_wheel_name);
    if (config_.motor2_wheel_name != "")
      motor2_.setup(config_.motor2_wheel_name);
    if (config_.motor3_wheel_name != "")
      motor3_.setup(config_.motor3_wheel_name);

    // for (const hardware_interface::ComponentInfo &joint : info_.joints)
    // {
    //   // epmc_v2 System has exactly 8 states states and four command interface on each joint
    //   if (joint.command_interfaces.size() != 1)
    //   {
    //     RCLCPP_FATAL(
    //         rclcpp::get_logger("EPMC_V2_HardwareInterface"),
    //         "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
    //         joint.command_interfaces.size());
    //     return hardware_interface::CallbackReturn::ERROR;
    //   }

    //   if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    //   {
    //     RCLCPP_FATAL(
    //         rclcpp::get_logger("EPMC_V2_HardwareInterface"),
    //         "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
    //         joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
    //     return hardware_interface::CallbackReturn::ERROR;
    //   }

    //   if (joint.state_interfaces.size() != 2)
    //   {
    //     RCLCPP_FATAL(
    //         rclcpp::get_logger("EPMC_V2_HardwareInterface"),
    //         "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
    //         joint.state_interfaces.size());
    //     return hardware_interface::CallbackReturn::ERROR;
    //   }

    //   if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    //   {
    //     RCLCPP_FATAL(
    //         rclcpp::get_logger("EPMC_V2_HardwareInterface"),
    //         "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
    //         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    //     return hardware_interface::CallbackReturn::ERROR;
    //   }

    //   if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    //   {
    //     RCLCPP_FATAL(
    //         rclcpp::get_logger("EPMC_V2_HardwareInterface"),
    //         "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
    //         joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
    //     return hardware_interface::CallbackReturn::ERROR;
    //   }
    // }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> EPMC_V2_HardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    if (config_.motor0_wheel_name != ""){
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor0_.name, hardware_interface::HW_IF_POSITION, &motor0_.angPos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor0_.name, hardware_interface::HW_IF_VELOCITY, &motor0_.angVel));
    }

    if (config_.motor1_wheel_name != ""){
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor1_.name, hardware_interface::HW_IF_POSITION, &motor1_.angPos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor1_.name, hardware_interface::HW_IF_VELOCITY, &motor1_.angVel));
    }

    if (config_.motor2_wheel_name != ""){
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor2_.name, hardware_interface::HW_IF_POSITION, &motor2_.angPos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor2_.name, hardware_interface::HW_IF_VELOCITY, &motor2_.angVel));
    }

    if (config_.motor3_wheel_name != ""){
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor3_.name, hardware_interface::HW_IF_POSITION, &motor3_.angPos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(motor3_.name, hardware_interface::HW_IF_VELOCITY, &motor3_.angVel));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EPMC_V2_HardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    if (config_.motor0_wheel_name != ""){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(motor0_.name, hardware_interface::HW_IF_VELOCITY, &motor0_.cmdAngVel));
    }

    if (config_.motor1_wheel_name != ""){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(motor1_.name, hardware_interface::HW_IF_VELOCITY, &motor1_.cmdAngVel));
    }

    if (config_.motor2_wheel_name != ""){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(motor2_.name, hardware_interface::HW_IF_VELOCITY, &motor2_.cmdAngVel));
    }

    if (config_.motor3_wheel_name != ""){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(motor3_.name, hardware_interface::HW_IF_VELOCITY, &motor3_.cmdAngVel));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Configuring ...please wait...");
    if (epmcV2_.connected())
    {
      epmcV2_.disconnect();
    }
    epmcV2_.connect(config_.port);

    delay_ms(2000);

    epmcV2_.clearDataBuffer();
    epmcV2_.writeSpeed(0.0, 0.0, 0.0, 0.0);

    int cmd_timeout = std::stoi(config_.cmd_vel_timeout_ms.c_str());
    epmcV2_.setCmdTimeout(cmd_timeout); // set motor command timeout
    cmd_timeout = epmcV2_.getCmdTimeout();

    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "motor_cmd_timeout_ms: %d ms", (cmd_timeout));

    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Cleaning up ...please wait...");
    if (epmcV2_.connected())
    {
      epmcV2_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Activating ...please wait...");
    if (!epmcV2_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    epmcV2_.clearDataBuffer();
    epmcV2_.writeSpeed(0.0, 0.0, 0.0, 0.0);

    running_ = true;
    io_thread_ = std::thread(&EPMC_V2_HardwareInterface::serialReadWriteLoop, this);
    epmcReadWriteTime = std::chrono::system_clock::now();

    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Successfully Activated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn EPMC_V2_HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Deactivating ...please wait...");

    running_ = false;
    if (io_thread_.joinable()) {
      io_thread_.join();
    }

    epmcV2_.clearDataBuffer();
    epmcV2_.writeSpeed(0.0, 0.0, 0.0, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("EPMC_V2_HardwareInterface"), "Successfully Deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type EPMC_V2_HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (config_.motor0_wheel_name != ""){
      motor0_.angPos = pos_cache_[0];
      motor0_.angVel = vel_cache_[0];
    }
    if (config_.motor1_wheel_name != ""){
      motor1_.angPos = pos_cache_[1];
      motor1_.angVel = vel_cache_[1];
    }  
    if (config_.motor2_wheel_name != ""){
      motor2_.angPos = pos_cache_[2];
      motor2_.angVel = vel_cache_[2];
    }   
    if (config_.motor3_wheel_name != ""){
      motor3_.angPos = pos_cache_[3];
      motor3_.angVel = vel_cache_[3];
    } 

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type epmc_v2_hardware_interface ::EPMC_V2_HardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (config_.motor0_wheel_name != "") cmd_cache_[0] = motor0_.cmdAngVel;
    if (config_.motor1_wheel_name != "") cmd_cache_[1] = motor1_.cmdAngVel;
    if (config_.motor2_wheel_name != "") cmd_cache_[2] = motor2_.cmdAngVel;
    if (config_.motor3_wheel_name != "") cmd_cache_[3] = motor3_.cmdAngVel;

    return hardware_interface::return_type::OK;
  }

  void EPMC_V2_HardwareInterface::serialReadWriteLoop()
  {
    while (running_) {
      epmcReadWriteDuration = (std::chrono::system_clock::now() - epmcReadWriteTime);
      if (epmcReadWriteDuration.count() > epmcReadWriteTimeInterval)
      {
        try {
          float pos0, pos1, pos2, pos3, v0, v1, v2, v3;
          // Read latest state from hardware
          epmcV2_.readMotorData(pos0, pos1, pos2, pos3, v0, v1, v2, v3);
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            pos_cache_[0] = pos0; pos_cache_[1] = pos1; pos_cache_[2] = pos2; pos_cache_[3] = pos3;
            vel_cache_[0] = v0;   vel_cache_[1] = v1;   vel_cache_[2] = v2;   vel_cache_[3] = v3;

            // Write latest commands
            epmcV2_.writeSpeed(cmd_cache_[0], cmd_cache_[1], cmd_cache_[2], cmd_cache_[3]);
          }
        }
        catch (...) {
          // Ignore read/write errors
        }
        epmcReadWriteTime = std::chrono::system_clock::now();
      }
    }
  }

} // namespace epmc_v2_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(epmc_v2_hardware_interface::EPMC_V2_HardwareInterface, hardware_interface::SystemInterface)
