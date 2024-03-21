## **TinyMovr**

![GitHub](https://img.shields.io/github/license/yourusername/TinyMovr)
![GitHub last commit](https://img.shields.io/github/last-commit/yourusername/TinyMovr)
![GitHub contributors](https://img.shields.io/github/contributors/yourusername/TinyMovr)
![GitHub issues](https://img.shields.io/github/issues/yourusername/TinyMovr)

### **Overview**

`TinyMovr` is a ROS package designed for controlling BLDC motors using the TinyMovr API. It allows for motor control based on velocity commands published to `cmd_vel`. This repository provides a comprehensive solution for controlling motors and obtaining wheel odometry readings using encoder feedback.

### **Features**

- **TinyMovr Integration:** Utilizes the TinyMovr API for controlling BLDC motors.
- **Velocity Control:** Accepts velocity commands published to the `cmd_vel` topic for motor control.
- **Teleoperation Support:** Compatible with teleoperation tools like `teleop_twist_keyboard` or `move_base`.
- **Wheel Odometry:** Publishes wheel odometry readings using encoder feedback from the motors.
- **UKF Implementation:** Includes an Unscented Kalman Filter (UKF) for estimating wheel odometry.

### **Installation**

#### **Prerequisites**

- ROS (Robot Operating System) installed on your system. Follow the [ROS installation instructions](http://wiki.ros.org/ROS/Installation) if you haven't already installed it.
- TinyMovr API installed. Refer to the TinyMovr documentation for installation instructions.

#### **Building**

Clone the repository into your ROS workspace and build it using `catkin_make`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/TinyMovr.git
cd ..
catkin_make
```

### **Usage**

1. Launch the `TinyMovr` node:

```bash
roslaunch TinyMovr TinyMovr.launch
```

2. Publish velocity commands to the `cmd_vel` topic to control the motors.

### **Configuration**

- **Motor Configuration:** Adjust motor parameters and control settings in the configuration files located at `TinyMovr/config/`.

### **Contributing**

Contributions are welcome! If you find any issues or want to suggest improvements, feel free to open an issue or submit a pull request.

### **License**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### **Acknowledgments**

- Special thanks to the developers of the TinyMovr API for providing a robust interface for controlling BLDC motors.
- Credits to the ROS community for creating teleoperation and odometry packages that enhance the functionality of `TinyMovr`.
