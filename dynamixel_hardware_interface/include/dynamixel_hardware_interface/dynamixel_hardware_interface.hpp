/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
/* Authors: Hye-Jong KIM, Yong-Ho Na */

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL_HARDWARE_INTERFACE_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "dynamixel_hardware_interface/visibility_control.h"
#include "dynamixel_hardware_interface/dynamixel/dynamixel.hpp"

#include "dynamixel_interfaces/msg/dynamixel_state.hpp"
#include "dynamixel_interfaces/srv/get_data_from_dxl.hpp"
#include "dynamixel_interfaces/srv/set_data_to_dxl.hpp"
#include "dynamixel_interfaces/srv/reboot_dxl.hpp"

#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_buffer.h"

#include "std_srvs/srv/set_bool.hpp"

#define PRESENT_POSITION_INDEX 0
#define PRESENT_VELOCITY_INDEX 1
#define PRESENT_ERROT_INDEX 2

namespace dynamixel_hardware_interface
{

constexpr char HW_IF_HARDWARE_STATE[] = "hardware_state";
constexpr char HW_IF_TORQUE_ENABLE[] = "torque_enable";


typedef struct HandlerVarType_
{
  uint8_t id;
  std::string name;
  std::vector<std::string> interface_name_vec;
  std::vector<std::shared_ptr<double>> value_ptr_vec;
} HandlerVarType;

typedef enum DxlStatus
{
  DXL_OK = 0,
  HW_ERROR = 1,
  COMM_ERROR = 2,
  REBOOTING = 3,
};

typedef enum DxlTorqueStatus
{
  TORQUE_ENABLED = 0,
  TORQUE_DISABLED = 1,
  REQUESTED_TO_ENABLE = 2,
  REQUESTED_TO_DISABLE = 3,
};

class DynamixelHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>, rclcpp::Node
{
public:
  DynamixelHardware();
  ~DynamixelHardware();

  // RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  ///// ros
  rclcpp::Logger logger_;

  ///// dxl error
  DxlStatus dxl_status_;
  DxlError dxl_comm_err_;
  std::map<uint8_t /*id*/, uint8_t /*err*/> dxl_hw_err_;
  DxlTorqueStatus dxl_torque_status_;
  std::map<uint8_t /*id*/, bool /*enable*/> dxl_torque_state_;

  double err_timeout_sec_;

  DxlError CheckError(DxlError dxl_comm_err);

  bool CommReset();

  ///// dxl variable
  std::shared_ptr<Dynamixel> dxl_comm_;
  std::string port_name_;
  std::string baud_rate_;
  std::vector<uint8_t> dxl_id_;

  ///// sensor variable
  std::vector<uint8_t> sensor_id_;
  std::map<uint8_t /*id*/, std::string /*interface_name*/> sensor_item_;

  ///// handler variable
  std::vector<HandlerVarType> hdl_trans_states_;
  std::vector<HandlerVarType> hdl_trans_commands_;
  std::vector<HandlerVarType> hdl_joint_states_;
  std::vector<HandlerVarType> hdl_joint_commands_;

  ///// handler sensor variable
  std::vector<HandlerVarType> hdl_gpio_sensor_states_;
  std::vector<HandlerVarType> hdl_sensor_states_;

  // joint <-> transmission matrix
  size_t num_of_joints_;
  size_t num_of_transmissions_;
  double ** transmission_to_joint_matrix_;
  double ** joint_to_transmission_matrix_;

  bool InitDxlItems();
  bool InitDxlReadItems();
  bool InitDxlWriteItems();

  void ReadSensorData(const HandlerVarType & sensor);

  ///// fuction
  void SetMatrix();
  void CalcTransmissionToJoint();
  void CalcJointToTransmission();

  void SyncJointCommandWithStates();

  void ChangeDxlTorqueState();

  ///// ROS
  using DynamixelStateMsg = dynamixel_interfaces::msg::DynamixelState;
  using StatePublisher = realtime_tools::RealtimePublisher<DynamixelStateMsg>;
  rclcpp::Publisher<DynamixelStateMsg>::SharedPtr dxl_state_pub_;
  std::unique_ptr<StatePublisher> dxl_state_pub_uni_ptr_;
  // srv server
  rclcpp::Service<dynamixel_interfaces::srv::GetDataFromDxl>::SharedPtr get_dxl_data_srv_;
  void get_dxl_data_srv_callback(
    const std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Request> request,
    std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Response> response);

  rclcpp::Service<dynamixel_interfaces::srv::SetDataToDxl>::SharedPtr set_dxl_data_srv_;
  void set_dxl_data_srv_callback(
    const std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Request> request,
    std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Response> response);

  rclcpp::Service<dynamixel_interfaces::srv::RebootDxl>::SharedPtr reboot_dxl_srv_;
  void reboot_dxl_srv_callback(
    const std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Request> request,
    std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Response> response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_dxl_torque_srv_;
  void set_dxl_torque_srv_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // timer
  int ros_update_freq_;
  std::thread ros_update_thread_;
};

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL_HARDWARE_INTERFACE_HPP_
