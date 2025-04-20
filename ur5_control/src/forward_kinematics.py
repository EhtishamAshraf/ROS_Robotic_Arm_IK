#!/usr/bin/env python3

""" Code to perform Forward Kinematics for UR5 Robot using MoveIt! """

import rospy
import moveit_commander
from tf.transformations import euler_from_quaternion

# Initialize ROS node & MoveIt commander
rospy.init_node("ur5_fk_node", anonymous=True)
moveit_commander.roscpp_initialize([])

# Loading robot model and scene
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator") 

# Forward Kinematics Function
def move_to_joint_angles():
    
    rospy.loginfo("\nEnter 6 joint angles for UR5 (comma-separated, in radians): ")
    input_angles = input(">>> ").strip()
    
    try:
        target_joints = [float(angle) for angle in input_angles.split(",")]

        if len(target_joints) != 6:
            rospy.logwarn("Please enter exactly 6 joint values.")
            return
        
        rospy.loginfo(f"Moving to Joint Angles: {target_joints}")
        group.set_joint_value_target(target_joints)
        success = group.go(wait=True)
        group.stop()

        if success:
            rospy.loginfo("Movement successful...")
        else:
            rospy.logwarn("Motion planning failed! Check joint limits or obstacles.")

    except ValueError:
        rospy.logwarn("Invalid input! Please enter numbers separated by commas.")


if __name__ == "__main__":
    rospy.loginfo("\nRobot is ready!")

    while not rospy.is_shutdown():
        rospy.loginfo("\nOptions:")
        rospy.loginfo("1 - Move Robot to Manual Joint Angles")
        rospy.loginfo("2 - Exit")
        
        choice = input(">>> Select an option: ").strip()

        if choice == "1":
            move_to_joint_angles()
        elif choice == "2":
            rospy.loginfo("Exiting...")
            break
        else:
            rospy.logwarn("Invalid choice! Please enter 1, or 2.")

    moveit_commander.roscpp_shutdown()


