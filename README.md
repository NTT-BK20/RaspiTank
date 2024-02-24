# Raspitank

<p align='center'>
    <img src=img/raspitank.jpg width="600">
</p>

A differential drive robot is controlled using ROS2 Humble running on a Raspberry Pi 4 (running Ubuntu server 22.04). The vehicle is equipped with a Raspberry Pi camera for visual feedback and an RPlidar A1 sensor used for Simultaneous Localization and Mapping (SLAM), autonomous navigation and obstacle avoidance. Additionally, an MPU6050 inertial measurement unit (IMU) is employed by the `robot_localization` package on the robot, to fuse IMU sensor data and the wheel encoders data, using an Extended Kalman Filter (EKF) node, to provide more accurate robot odometry estimates.

Hardware components are written for the Waveshare Motor Driver HAT and MPU6050 sensor to be accessed by the `ros2_control` differential drive controller and Imu sensor broadcaster respectively, via the `ros2_control` resource manager.

## 🗃️ Package Overview

- [`raspitank_base`](./raspitank_base/) : Contains the ROS2 control hardware component for the raspitank with low-level code for the Arduino Nano.
- [`raspitank_bringup`](./raspitank_bringup/) : Contains hardware component for the MPU6050 module, launch files to bring up the camera, lidar and the real raspitank.
- [`raspitank_description`](./raspitank_description/) : Contains the URDF description files for raspitank, sensors and `ros2 control`.
- [`raspitank_gazebo`](./raspitank_gazebo/) : Contains configuration, launch and world files needed to simulate raspitank in Gazebo.
- [`raspitank_navigation`](./raspitank_navigation/) : Contains launch, configuration and map files used for raspitank navigation.
- [`raspitank_slam`](./raspitank_slam/) : Contains configuration files for the slam toolbox and RViz, launch file to generate maps using SLAM.
- [`raspitank_teleop`](./raspitank_teleop/) : Contains configuration and launch files used to enable joystick control of the raspitank in simulation and physically.

## 🧰 Hardware

### Part list

The following components were used in this project:

|     | Part                                                                                                                                 |
| --- | ------------------------------------------------------------------------------------------------------------------------------------ |
| 1   | Raspberry Pi 4 (4 GB)                                                                                                                |
| 2   | Arduino Nano (Type C)                                                                                                                |
| 3   | [YP100 Metal Robot Tank Car Chassis Kit](https://www.amazon.com/Professional-Raspberry-MicroBit-Caterpillar-Education/dp/B09KKRY84S) |
| 4   | 2x Encoder DC Motor 12V 333rpm                                                                                                       |
| 5   | [MPU6050 board](https://nshopvn.com/product/cam-bien-gia-toc-gy-521-6dof-imu-mpu6050/)                                               |
| 6   | [RPlidar A1](https://hshop.vn/products/cam-bien-laser-radar-lidar-rplidar-a1)                                                        |
| 7   | Raspberry Pi camera v2.3                                                                                                             |
| 8   | Mount for Raspberry Pi camera                                                                                                        |
| 9   | Module Buck DC-DC Vin 9-36V Vout 5V 5A                                                                                               |
| 10  | Xbox One X                                                                                                                           |
| 11  | L298N DC Motor Driver                                                                                                                |
| 12  | Pin Lipo 3s 11.1V 2200 mAH                                                                                                           |
| 13  | Female to Female Dupont jumper cables                                                                                                |
| 14  | Breadboard 400 Tie Points                                                                                                            |

### Project Wiring and Assembly

The MPU6050 board pins were connected to the Raspberry Pi 4 GPIO pins as follows for use with the I2C communication protocol:

| MPU6050 board | GPIO.BOARD | GPIO.BCM |
| ------------- | ---------- | -------- |
| VCC           | 3.3V       | 3.3V     |
| GND           | GND        | GND      |
| SCL           | 05         | GPIO03   |
| SDA           | 03         | GPIO02   |

The screw terminal blocks on the L298N DC Motor Driver are connected to the motor wires and Arduino Nano as follows:

| Arduino Nano pin   | L298N pin   |
| ------------------ | ----------- |
| D5                 | IN4 (R Rev) |
| D6                 | IN1 (L Rev) |
| D9                 | IN3 (R Fwd) |
| D10                | IN2 (L Fwd) |
| Red wire (R 12V)   | OUT1        |
| White wire (R GND) | OUT2        |
| Red wire (L 12V)   | OUT3        |
| White wire (L GND) | OUT4        |

| Arduino Nano pin | Connected to Encoder Motor |
| ---------------- | -------------------------- |
| D2               | Yellow wire (Left motor A) |
| D3               | Yellow wire (Left motor B) |
| A4               | Green wire (Right motor A) |
| A5               | Green wire (Right motor A) |
| GND              | 2x Black wire (GND)        |
| 3.3V             | 2x Blue wire (3.3~5V)      |

Finally, the Raspberry Pi camera is connected to the ribbon slot on the Raspberry Pi 4 and the RPlidar A1 sensor is plugged into one of the RPi 4's USB ports.

### Gazebo Simulation
Youtube: [Link](https://youtu.be/c58yLrJIEjk)

🚧	***(Work in Progress)*** 

## Acknowledgment

- [Lidarbot](https://github.com/TheNoobInventor/lidarbot)
- [Articulated Robotics](https://articulatedrobotics.xyz/)
- [Automatic Addison](https://automaticaddison.com/)
- [Diffbot](https://github.com/ros-mobile-robots/diffbot)
- [Linorobot2](https://github.com/linorobot/linorobot2)
- [Mini pupper](https://github.com/mangdangroboticsclub/mini_pupper_ros)
- [Robotics Backend](https://roboticsbackend.com/)
