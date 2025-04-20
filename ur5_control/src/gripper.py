#!/usr/bin/env python3

"""Code to control the gripper of a UR5 robot using MoveIt!."""

import moveit_commander
import sys
import rospy

def close_gripper(gripper):
    """
    Closes the gripper by setting the target position for the 'robotiq_85_left_knuckle_joint'.
    """
    rospy.loginfo("Closing gripper...")

    # Set the target joint value for the gripper
    gripper.set_joint_value_target({"robotiq_85_left_knuckle_joint": 0.785})

    # Execute the motion
    success = gripper.go(wait=True)
    gripper.stop()  # Ensure the gripper stops after moving

    if success:
        rospy.loginfo("Gripper closed successfully.")
    else:
        rospy.logerr("Failed to close the gripper.")
        
def open_gripper(gripper):
    """
    Opens the gripper by setting the target position for the 'robotiq_85_left_knuckle_joint'.
    """
    rospy.loginfo("Opening gripper...")

    gripper.set_joint_value_target({"robotiq_85_left_knuckle_joint": 0})

    success = gripper.go(wait=True)
    gripper.stop() 

    if success:
        rospy.loginfo("Gripper Opened successfully.")
    else:
        rospy.logerr("Failed to open the gripper.")

def main():
    rospy.init_node('gripper_control', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)

    gripper = moveit_commander.MoveGroupCommander("gripper")

    rospy.sleep(2.0)

    close_gripper(gripper)   # Close the gripper
    
    rospy.sleep(2.0)

    open_gripper(gripper)    # Open the gripper

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

