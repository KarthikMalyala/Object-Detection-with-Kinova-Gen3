#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

### PICK & PLACE TASKS WITH ARNA ###
### Karthik Malyala
### 12/14/2021

import sys

from yaml.events import MappingStartEvent
import rospy
import time
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers

from kortex_driver.srv import *
from kortex_driver.msg import *

class ExampleFullArmMovement:
    global x, y, z

    def __init__(self):
        try:
            rospy.init_node('example_full_arm_movement_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
            rospy.wait_for_service(play_cartesian_trajectory_full_name)
            self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name, PlayCartesianTrajectory)

            play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            rospy.wait_for_service(play_joint_trajectory_full_name)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            if (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def home(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(5.0)
        return True

    def send_pose(self, x, y, z):
        self.last_action_notif_type = None

        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = x
        req.input.target_pose.y = y 
        req.input.target_pose.z = z
        req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
        req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
        req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def final_pose(self, x, y, z):
        self.last_action_notif_type = None

        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = x - 0.02
        req.input.target_pose.y = y
        req.input.target_pose.z = z 
        req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
        req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
        req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def gripper(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True


    def callback(self, msg):
        global x,y,z, target
        obj = AlvarMarkers()
        obj = msg

        for item in obj.markers:
            if (item.id) == target:
                x = item.pose.pose.position.x
                y = item.pose.pose.position.y
                z = item.pose.pose.position.z
            # else:
            #     rospy.loginfo("No Target Found")

    def main(self):
        global x, y, z, target
        x = 0
        y = 0
        z = 0
        target = 0

        #rospy.Subscriber("marker_to_kinova", geometry_msgs.msg.Point, self.callback)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.callback)

        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass


        if success:
        #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Activate the action notifications
            success &= self.subscribe_to_a_robot_notification()
            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            success &= self.home()
            #*******************************************************************************

            #*******************************************************************************
            # Example of gripper command
            # Let's fully open the gripper
            if self.is_gripper_present:
                success &= self.gripper(0.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")  
            #*******************************************************************************

            #*******************************************************************************
            # Set the refecaserence frame to "Mixed"
            success &= self.cartesian_reference_frame()

            #count = 0
            case = 1

            success &= self.send_pose(0.579, 0, 0.189)

            while not rospy.is_shutdown():
                target = input("Enter your desired target: ")
                rospy.loginfo("Waiting for publisher input")
                rospy.loginfo(" " + str(x-0.05) + " " + str(y) + " " + str(z))
                case = 1

                while(case != 0):

                    # CASE 1: Start at home, move to Case 2 for scanning
                    if case == 1:
                        print("Case 1")
                        #success &= self.home()
                        success &= self.send_pose(0.579, 0, 0.189)
                        case = 2

                    # CASE 2: Start Scanning for the desired target and test the distance between the arm's current position and the target
                    if case == 2:
                        print("Case 2")
                        rospy.loginfo(" " + str(x-0.05) + " " + str(y) + " " + str(z))
                        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
                        curx = feedback.base.commanded_tool_pose_x
                        cury = feedback.base.commanded_tool_pose_y 
                        curz = feedback.base.commanded_tool_pose_z
                        rospy.loginfo("Current Position: " + str(curx) + " " + str(cury) + " " + str(curz))
                        # If distance is minimal, go to Case 4 for fine movement
                        if ((curx <= x + 0.1 and curx >= x - 0.1) and (cury <= y + 0.1 and cury >= y - 0.1) and (curz <= z + 0.1 and curz >= z - 0.1)):
                            case = 4
                        # Else, go to case 3 for sequential movement
                        else:
                            case = 3

                    # CASE 3: Sequential movement towards the target to reach the proximity
                    if case == 3:
                        print("Case 3")
                        diffx = x - curx
                        diffy = y - cury
                        diffz = z - curz
                        success &= self.send_pose((curx + (diffx - 0.1)), (cury + (diffy - 0.0)), (curz + (diffz - 0.0)))
                        time.sleep (0.1)
                        # After sequential movement, go back to Case 2
                        case = 2

                    # CASE 4: Fine movement towards the target with no offset
                    if case == 4:
                        print("Case 4")
                        success &= self.final_pose((x), y, z)
                        time.sleep (0.001)
                        # If the target is in close proximity, activate gripper in Case 5
                        if ((curx <= x + 0.02 and curx >= x - 0.02) and (cury <= y + 0.02 and cury >= y - 0.02) and (curz <= z + 0.02 and curz >= z - 0.02)):
                            case = 5
                        # Else, scan again and move further
                        else:
                            case = 2

                    # CASE 5: Grips the target once found
                    if case == 5:
                        print("Case 5")
                        success &= self.gripper(0.6)
                        case = 6
                    
                    # CASE 6: Sends it back home and asks if you we would like to ungrip the target
                    if case == 6:
                        print("Case 6")
                        time.sleep(0.01)
                        success &= self.home()
                        if not (curx <= 0.579 + 0.02 and curx <= 0.579 - 0.02):
                            success &= self.send_pose(0.579, 0, 0.429)

                        ungrip = input("Would you like to ungrip? (0 (Yes) / 1 (No))")
                        if (ungrip == 0):
                            success &= self.gripper(0.0)
                            break
                        else:
                            break

                    time.sleep(0.25)

                #success &= self.home()
                if not (curx <= 0.579 + 0.02 and curx <= 0.579 - 0.02):
                    success &= self.home()

                # If you would like to repeat the process for another object, press 0 for Yes
                quit = input("Would you like to quit? (0 (Yes) / 1 (No)) ")
                if (quit == 0):
                    break
                # Else, loop again
                else:
                    continue


        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)
        #if not success:
            #rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleFullArmMovement()
    ex.main()