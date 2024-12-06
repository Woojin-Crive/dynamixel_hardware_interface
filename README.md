# **Dynamixel Hardware Interface User Guide**

## **1. Introduction**

ROS 2 package providing a hardware interface for controlling [Dynamixel](https://www.robotis.us/dynamixel/) motors via the ros2_control framework. This repository includes the **dynamixel_hardware_interface plugin** for seamless integration with ROS 2 control, along with the dynamixel_interfaces package containing custom message definitions used by the interface

---

## 2. **Prerequisites**

![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
This package currently supports ROS 2 Humble only. Ensure that ROS 2 Humble is properly installed (ROS 2 Humble installation guide).

- Dynamixel SDK: Install the Dynamixel SDK using the following command:

  ```bash
  sudo apt install ros-humble-dynamixel-sdk
  ```

- Hardware Requirements:

  - Dynamixel servos
  - USB2 Dynamixel or U2D2 adapter
  - Proper power supply for Dynamixel motors.

---

## **3. Installation**

1. Clone the repository into your ROS workspace:

   ```bash
   cd ~/your_workspace/src
   git clone https://github.com/your-repository/dynamixel_hardware_interface.git
   ```

2. Build the package:

   ```bash
   cd ~/your_workspace
   colcon build
   ```

3. Source your workspace:

   ```bash
   source ~/your_workspace/install/setup.bash
   ```

---

## 4. Currently Used Packages

This project integrates with the following ROS 2 packages to provide extended functionality:

- **[open_manipulator_x](https://github.com/ROBOTIS-GIT/open_manipulator_x)**
  A ROS-based open-source software package designed for the **Open Manipulator-X**, a 4-DOF robotic arm. It provides essential features like motion planning, kinematics, and control utilities for seamless integration with ROS 2 environments.

- **[open_manipulator_y](https://github.com/ROBOTIS-GIT/open_manipulator_y)**
  A ROS-based package tailored for the **Open Manipulator-Y**, a 6-DOF robotic arm. This package offers enhanced compatibility and extended functionalities for advanced manipulator control and operations.

----



## 5. Configuration

To effectively use the **Dynamixel Hardware Interface** in a ROS 2 control system, you need to configure specific parameters in your `ros2_control` hardware description file. Below is a concise explanation of the key parameters, illustrated with examples from the **OpenManipulator-X ROS 2 control.xacro** file.

1. **Port Settings**: Define serial port and baud rate for communication.
2. **Hardware Setup**: Configure joints and transmissions.
3. **Joints**: Control and monitor robot joints.
4. **GPIO**: Define and control Dynamixel motors.

#### **1. Port and Communication Settings**

These parameters define how the interface communicates with the Dynamixel motors:

- **`port_name`**: Serial port for communication.
   **Default**: `/dev/ttyUSB0`

  ```xml
  <param name="port_name">/dev/ttyUSB0</param>
  ```

- **`baud_rate`**: Communication baud rate.
   **Default**: `1000000`

  ```xml
  <param name="baud_rate">1000000</param>
  ```

- **`error_timeout_sec`**: Timeout for communication errors.
   **Default**: `0.2`

  ```xml
  <param name="error_timeout_sec">0.2</param>
  ```

------

#### **2. Hardware Configuration**

These parameters define the hardware setup:

- **`number_of_joints`**: Total number of joints.
   **Default**: `5`

  ```xml
  <param name="number_of_joints">5</param>
  ```

- **`number_of_transmissions`**: Number of transmissions.
   **Default**: `5`

  ```xml
  <param name="number_of_transmissions">5</param>
  ```

- **Transmission Matrices**: Define joint-to-transmission mappings.

  ```xml
  <param name="transmission_to_joint_matrix">
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1
  </param>
  ```

------

#### **3. Joint Configuration**

Joints define the control and state interfaces for robot movement:

##### **Key Attributes**

- **`name`**: Unique joint name.
   Example: `${prefix}joint1`

##### **Sub-Elements**

1. **`<command_interface>`**: Sends commands to joints.

   ```xml
   <command_interface name="position">
     <param name="min">${-pi*0.1}</param>
     <param name="max">${pi*0.1}</param>
   </command_interface>
   ```

2. **`<state_interface>`**: Monitors joint state data.

   ```xml
   <state_interface name="position"/>
   <state_interface name="velocity"/>
   <state_interface name="effort"/>
   ```

##### **Example Joint Configuration**

```xml
<joint name="${prefix}joint1">
  <command_interface name="position">
    <param name="min">${-pi*0.1}</param>
    <param name="max">${pi*0.1}</param>
  </command_interface>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

------

#### **4. GPIO Configuration**

The GPIO tag defines Dynamixel motors or similar components.

##### **Key Attributes**

- **`name`**: Unique name for the motor (e.g., `dxl1`).
- **`ID`**: Motor ID (e.g., `11`).

##### **Sub-Elements**

1. **`<param>`**: Motor-specific settings.

   ```xml
   <param name="type">dxl</param>
   <param name="ID">11</param>
   ```

2. **`<command_interface>`**: Sends commands to motors.

   ```xml
   <command_interface name="Goal Position"/>
   ```

3. **`<state_interface>`**: Monitors motor state data.

   ```xml
   <state_interface name="Present Position"/>
   <state_interface name="Present Velocity"/>
   <state_interface name="Present Current"/>
   ```

##### **Example GPIO Configuration**

```xml
<gpio name="dxl1">
  <param name="type">dxl</param>
  <param name="ID">11</param>
  <command_interface name="Goal Position"/>
  <state_interface name="Present Position"/>
  <state_interface name="Present Velocity"/>
  <state_interface name="Present Current"/>
  <param name="Position P Gain">800</param>
  <param name="Position I Gain">100</param>
  <param name="Position D Gain">100</param>
  <param name="Drive Mode">0</param>
</gpio>
```

------

###

## **6. Usage**

Ensure the parameters are configured correctly in your `ros2_control` YAML file or XML launch file.

- Example Parameter Configuration

```xml
<ros2_control>
    <param name="dynamixel_state_pub_msg_name">dynamixel_hardware_interface/dxl_state</param>
    <param name="get_dynamixel_data_srv_name">dynamixel_hardware_interface/get_dxl_data</param>
    <param name="set_dynamixel_data_srv_name">dynamixel_hardware_interface/set_dxl_data</param>
    <param name="reboot_dxl_srv_name">dynamixel_hardware_interface/reboot_dxl</param>
    <param name="set_dxl_torque_srv_name">dynamixel_hardware_interface/set_dxl_torque</param>
</ros2_control>
```

------

#### Parameter Descriptions

##### 1. **dynamixel_state_pub_msg_name**

- **Description**: Defines the topic name for publishing the Dynamixel state.

- **Default Value**: `dynamixel_hardware_interface/dxl_state`

  ```bash
  ros2 topic echo /dynamixel_hardware_interface/dxl_state
  ```

------

##### 2. **get_dynamixel_data_srv_name**

- **Description**: Specifies the service name for retrieving Dynamixel data.

- **Default Value**: `dynamixel_hardware_interface/get_dxl_data`

  ```bash
  ros2 service call /dynamixel_hardware_interface/get_dxl_data dynamixel_sdk_custom_interfaces/srv/GetMotorState "{id: 1}"
  ```

------

##### 3. **set_dynamixel_data_srv_name**

- **Description**: Specifies the service name for setting Dynamixel data.

- **Default Value**: `dynamixel_hardware_interface/set_dxl_data`

  ```bash
  ros2 service call /dynamixel_hardware_interface/set_dxl_data dynamixel_sdk_custom_interfaces/srv/SetMotorData "{id: 1, address: 64, data: 1}"
  ```

------

##### 4. **reboot_dxl_srv_name**

- **Description**: Specifies the service name for rebooting Dynamixel motors.

- **Default Value**: `dynamixel_hardware_interface/reboot_dxl`

  ```bash
  ros2 service call /dynamixel_hardware_interface/reboot_dxl dynamixel_sdk_custom_interfaces/srv/RebootDxl "{id: 1}"
  ```

------

##### 5. **set_dxl_torque_srv_name**

- **Description**: Specifies the service name for enabling or disabling torque on Dynamixel motors.

- **Default Value**: `dynamixel_hardware_interface/set_dxl_torque`

  ```bash
  ros2 service call /dynamixel_hardware_interface/set_dxl_torque std_srvs/srv/SetBool "{data: true}"
  ```



---

## **7. Troubleshooting**

### **Common Issues**

1. **Device Not Found**

   - Ensure the correct port is specified in the YAML file (e.g., `/dev/ttyUSB0`).

   - Check if the USB device has proper permissions:

     ```bash
     sudo chmod 666 /dev/ttyUSB0
     ```

2. **No Response from Motor**

   - Verify the motor ID and baud rate match the configuration file.
   - Confirm the motor is powered and connected properly.

---

## **8. Contributing**

We welcome contributions! Please follow the guidelines in [CONTRIBUTING.md](CONTRIBUTING.md) to submit issues or pull requests.

---

## **9. License**

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

