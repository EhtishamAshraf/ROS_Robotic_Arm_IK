# 🦾 UR5 Inverse Kinematics using MoveIT in ROS
This repository contains a MoveIt configuration package for controlling a Universal Robots UR5 robotic arm equipped with a Robotiq 85 2-finger gripper. It includes a Python-based implementation for Inverse Kinematics (IK) to plan and execute precise motion trajectories. 

The main contribuation of this repo is a custom package named "UR5_control" along with a MoveIt configuration package called "ur5_gripper_moveit_config". 

👉 Gazebo is being used for simulation of the arm. 🤖🛠

##### 📺 Demo Video
You can watch the demo video of the arm in action by clicking on the below image:
[![Watch the video](https://github.com/EhtishamAshraf/ROS_Robotic_Arm_IK/blob/60bff498d70ef3fdcbd5e8199567287aac43b96b/ur5_control/Images/Gazebo.png)

> In the first half of the video, I am controlling the gripper and the arm group using MoveIt rviz Gui, and in the second half python is being used for controlling the arm.

![Rviz](https://github.com/EhtishamAshraf/ROS_Robotic_Arm_IK/blob/60bff498d70ef3fdcbd5e8199567287aac43b96b/ur5_control/Images/Rviz.png)

## 📥 Cloning the Repository
Create a ros workspace, inside it, create a src folder and navigate into it and run the following command to clone the repo:
```bash
git clone https://github.com/EhtishamAshraf/ROS_Robotic_Arm_IK.git
```
Run "catkin_make" inside the main workspace, to build the workspace and compile all ROS packages, ensuring that your system recognizes the newly cloned code.
```bash
catkin_make 
```
# 🤖⚙️ UR5 Robotic Arm
In robotics , **Forward Kinematics (FK)** and **Inverse Kinematics (IK)** are essential techniques used to describe and control the motion of robotic arms.

![FK_IK](https://github.com/EhtishamAshraf/ROS_Robotic_Arm_IK/blob/c0da7bc2267630df0f25671ed0f90d084a8b1dd1/ur5_control/Images/FK_IK.png)

**Forward Kinematics (FK)**

Forward Kinematics is the process of determining the position and orientation of the end effector based on the known joint parameters.

**Inverse Kinematics (IK)**

Inverse Kinematics is the reverse process of Forward Kinematics. Given the desired position and orientation of the end effector, IK calculates the joint angles required to achieve that desired position.

**Denavit-Hartenberg (DH) Table**

The image below shows the frames attached to each joint of the Eva arm.

   • A dot indicates a direction inward (into the plane), and a cross indicates the direction is outward.

   • Z-axis is represented by Blue, X-axis by Red, and Y-axis by Green

![ur5 Frames](https://github.com/EhtishamAshraf/ROS_Robotic_Arm_IK/blob/60bff498d70ef3fdcbd5e8199567287aac43b96b/ur5_control/Images/ur5_frames.png)

Based on these frames, the following DH table is derived:
![DH Table](https://github.com/EhtishamAshraf/ROS_Robotic_Arm_IK/blob/c0da7bc2267630df0f25671ed0f90d084a8b1dd1/ur5_control/Images/DH%20Table.png)


The DH table is constructed using the following four parameters for each joint:

   **θ (theta):** The angle of rotation about previous Z-axis to align X-axis with current X-axis.

   **a: The link length** — the distance between two consecutive origins measured along the X-axis.
   It's along the common normal between two Z-axes (and the X-axis lies along this common normal). Common normal is a vector perpendicular to both Z's.

   **d: The link offset** — the distance between two consecutive origins measured along the Z-axis.

   **α (alpha):** The link twist — the rotation about the X-axis needed to align the previous Z-axis with the current Z-axis.

   **Use the right-hand rule to determine the sign of rotation:**
   If the rotation appears counterclockwise (looking along the positive axis), the angle is positive.

   If the rotation appears clockwise (looking along the positive axis), the angle is negative.

- [`Universal Robots`](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/) – refer for more information.

## ⚙️ Execution: Follow the steps to launch and run the repository. 🚀
- Open a new terminal on your laptop (navigate to ~/path_to_your_workspace/src), 
  and run the following command to initialize the Gazebo and Rviz with the UR5 arm:
```bash
roslaunch ur5_control spawn_ur5_gripper.launch
```
- run the forward kinematics node by typing the following command:
```bash
rosrun ur5_control forward_kinematics.py
```
- run the inverse kinematics node by typing the following command:
```bash
rosrun ur5_control inverse_kinematics.py
```
- control the gripper (opening and closing) by running the following command:
```bash
rosrun ur5_control gripper.py
```
⚡ Make sure to run `source devel/setup.bash` in each new terminal before typing in the above commands.

