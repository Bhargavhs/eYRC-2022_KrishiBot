#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

class Ur5Moveit:

    # Constructor
    def __init__(self, group):

        rospy.init_node('node_eg2_predefined_pose', anonymous=True)

        self._planning_group = group
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        # rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        # rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        # rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_predefined_pose(self, arg_pose_name):
        
        self.__init__("gripper")

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        try:
            goal.trajectory = plan[1]
        except:
           goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        # rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
        
        return 
    
    def set_joint_angles(self, arg_list_joint_angles):
        self.__init__("arm")

        list_joint_values = self._group.get_current_joint_values()
        
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        # if (flag_plan == True):
        #     rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        # else:
        #     rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    phase = 1
    ur5 = Ur5Moveit("arm")
    
    phase1_joint_angles = [math.radians(73),
                            math.radians(60),
                            math.radians(-90),
                            math.radians(45),
                            math.radians(0),
                            math.radians(1)]
    
    # phase2_joint_angles = [math.radians(74),
    #                         math.radians(64),
    #                         math.radians(-91),
    #                         math.radians(0),
    #                         math.radians(4),
    #                         math.radians(1)]
    
    phase2_joint_angles = [math.radians(49),
                            math.radians(100),
                            math.radians(-122),
                            math.radians(-15),
                            math.radians(-118),
                            math.radians(1)]
    
    phase3_joint_angles = [math.radians(0),
                            math.radians(0),
                            math.radians(0),
                            math.radians(0),
                            math.radians(0),
                            math.radians(0)]
    
    phase4_joint_angles = [math.radians(-9),
                            math.radians(25),
                            math.radians(9),
                            math.radians(-1),
                            math.radians(39),
                            math.radians(5)]
    
    phase7_joint_angles = [math.radians(-48),
                            math.radians(-30),
                            math.radians(-111),
                            math.radians(-187),
                            math.radians(-66),
                            math.radians(-10)]
    
    phase8_joint_angles = [math.radians(-21),
                            math.radians(53),
                            math.radians(-37),
                            math.radians(-45),
                            math.radians(-136),
                            math.radians(19)]

    while not rospy.is_shutdown():
        if phase == 1 :
            ur5.set_joint_angles(phase1_joint_angles)
            phase = 2
        if phase == 2:
            ur5.set_joint_angles(phase2_joint_angles)
            phase = 4
        # if phase == 3:
        #     ur5.go_to_predefined_pose("close")
        #     phase = 4
        # if phase == 4:
        #     ur5.set_joint_angles(phase4_joint_angles)
        #     phase = 6
        # if phase == 5:
        #     ur5.go_to_predefined_pose("open")
        #     phase = 6
        # if phase == 6:
        #     ur5.set_joint_angles(phase7_joint_angles)
        #     phase = 7
        # if phase == 7:
        #     ur5.go_to_predefined_pose("close")
        #     phase = 8
        # if phase == 8:
        #     ur5.set_joint_angles(phase8_joint_angles)
        #     phase = 9
        # if phase == 9:
        #     ur5.go_to_predefined_pose("open")
        #     phase = 10          
    del ur5


if __name__ == '__main__':
    main()

