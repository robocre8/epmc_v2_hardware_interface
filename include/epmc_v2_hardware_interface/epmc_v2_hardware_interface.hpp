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

#ifndef EPMC_V2_HARDWARE_INTERFACE
#define EPMC_V2_HARDWARE_INTERFACE

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "epmc_v2_hardware_interface/epmc_v2.hpp"
#include "epmc_v2_hardware_interface/motor.hpp"
#include "epmc_v2_hardware_interface/imu.hpp"

namespace epmc_v2_hardware_interface
{
  class EPMC_V2_HardwareInterface : public hardware_interface::SystemInterface
  {

    struct Config
    {
      std::string motor0_wheel_name = "";
      std::string motor1_wheel_name = "";
      std::string motor2_wheel_name = "";
      std::string motor3_wheel_name = "";
      std::string port = "";
      std::string cmd_vel_timeout_ms = "";
      std::string imu_sensor_name = "";
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(EPMC_V2_HardwareInterface);

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    EPMC_V2 epmcV2_; // serial communication
    Config config_;        // configuration
    Motor motor0_;      // motor0 setup
    Motor motor1_;      // motor1 setup
    Motor motor2_;      // motor2 setup
    Motor motor3_;      // motor3 setup
    IMU imu_;
  };

} // namespace epmc_v2_hardware_interface

#endif // EPMC_V2_HARDWARE_INTERFACE
