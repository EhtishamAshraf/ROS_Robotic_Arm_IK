#!/usr/bin/env python3

""" Code to perform Inverse Kinematics for UR5 Robot using MoveIt! """

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations as tf

def close_gripper(gripper):
    """
    Closes the gripper by setting the target position for the 'robotiq_85_left_knuckle_joint'.
    """
    rospy.loginfo("Closing gripper...")

    # Set the target joint value for the gripper
    gripper.set_joint_value_target({"robotiq_85_left_knuckle_joint": 0.785})

    # Execute the motion
    success = gripper.go(wait=True)
    gripper.stop()  

    if success:
        rospy.loginfo("Gripper closed successfully.")
    else:
        rospy.logerr("Failed to close the gripper.")


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface_tutorial', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    group = moveit_commander.MoveGroupCommander("manipulator")
    gripper = moveit_commander.MoveGroupCommander("gripper")
    
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
    
    rospy.loginfo("Reference frame: %s", group.get_planning_frame())
    rospy.loginfo("End effector: %s", group.get_end_effector_link())
    
    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.5)

    # Target position
    target_pose = geometry_msgs.msg.Pose()
    quaternion = tf.quaternion_from_euler(-1.689, 1.447, -1.567)
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]
    target_pose.position.x = 0.815
    target_pose.position.y = 0.191
    target_pose.position.z = 0.200
    
    group.set_pose_target(target_pose)
    
    # Visualize the planning
    plan = group.plan()
    rospy.loginfo("Visualizing plan %s", "SUCCESS" if plan else "FAILED")
    
    # Move the group arm
    group.go(wait=True)
    rospy.sleep(1.0)
    group.stop()
    group.clear_pose_targets()

    rospy.sleep(2.0)

    # Close the gripper
    close_gripper(gripper)
    
    moveit_commander.roscpp_shutdown()
    
if __name__ == '__main__':
    main()
