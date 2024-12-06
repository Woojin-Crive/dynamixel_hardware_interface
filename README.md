
---

# **Dynamixel Hardware Interface User Guide**

## **Introduction**
ROS 2 package providing a hardware interface for controlling [Dynamixel](https://www.robotis.us/dynamixel/) motors via the ros2_control framework. This repository includes the **dynamixel_hardware_interface plugin** for seamless integration with ROS 2 control, along with the dynamixel_interfaces package containing custom message definitions used by the interface

---

## **Prerequisites**
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

## **Installation**

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

## Currently Used Packages

This project integrates with the following ROS 2 packages to provide extended functionality:

- **[open_manipulator_x](https://github.com/ROBOTIS-GIT/open_manipulator_x)**
  A ROS-based open-source software package designed for the **Open Manipulator-X**, a 4-DOF robotic arm. It provides essential features like motion planning, kinematics, and control utilities for seamless integration with ROS 2 environments.

- **[open_manipulator_y](https://github.com/ROBOTIS-GIT/open_manipulator_y)**
  A ROS-based package tailored for the **Open Manipulator-Y**, a 6-DOF robotic arm. This package offers enhanced compatibility and extended functionalities for advanced manipulator control and operations.

These packages are critical for the operation and development of Dynamixel-based robotic systems, ensuring precise and reliable performance.
---

## **Usage**

### **a. Controlling Motors**
Use ROS2 topics or services to send commands to the motors. For example:
- **Topic Example**: Publish commands to control motor positions:
   ```bash
   ros2 topic pub /dynamixel_command sensor_msgs/JointState "{name: ['joint1'], position: [1.57]}"
   ```

- **Service Example**: Call services to get motor states or change modes:
   ```bash
   ros2 service call /get_motor_state dynamixel_sdk_custom_interfaces/srv/GetMotorState "{id: 1}"
   ```

### **b. Monitoring Motor State**
Subscribe to the topic `/motor_states` to receive real-time feedback on motor positions, velocities, and loads:
   ```bash
   ros2 topic echo /motor_states
   ```

---

## **Troubleshooting**

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

## **Contributing**
We welcome contributions! Please follow the guidelines in [CONTRIBUTING.md](CONTRIBUTING.md) to submit issues or pull requests.

---

## **License**
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

