## EPMC (Easy PID Motor Controller) Version 2 ROS2 Hardware Interface Package

## How to Use the Package

#### Prequisite
- ensure you've already set up your microcomputer or PC system with [`ros-humble`](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) with [`colcon`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) and your `ros workspace` also setup

- install the `libserial-dev` package on your linux machine
  ```shell
  sudo apt-get update
  sudo apt install libserial-dev
  ```

- install `rosdep` so you can install necessary ros related dependencies for the package.
  ```shell
  sudo apt-get update
  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update
  ```

#

#### How to build the epmc_ros_hw_plugin plugin package 
- In the `src/` folder of your `ros workspace`, clone the repo

- from the `src/` folder, cd into the root directory of your `ros workspace` and run rosdep to install all necessary ros dependencies
  ```shell
  cd ../
  rosdep install --from-paths src --ignore-src -r -y
  ```
- build the `epmc_v2_hardware_interface` package with colcon (in the root folder of your ros workspace):
  ```shell
  colcon build --packages-select epmc_v2_hardware_interface
  ```
> [!NOTE]   
> The **epmc_v2_hardware_interface** package will now be available for use in any project in your ros workspace.
> You can see example of how the use the `epmc_v2_hardware_interface` in the `example_control_file`