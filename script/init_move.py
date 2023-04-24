import rospy
from ur_move import MoveGroupPythonIntefaceTutorial, gen_pose
import actionlib
import ur_dashboard_msgs.msg

def init_robot_position():
    global ur_control
    ## Trigger robot arm controller to play
    client_set_mode = actionlib.SimpleActionClient('/ur_hardware_interface/set_mode',
                                                   ur_dashboard_msgs.msg.SetModeAction)
    client_set_mode.wait_for_server()

    mode_action_goal = ur_dashboard_msgs.msg.SetModeGoal(target_robot_mode=ur_dashboard_msgs.msg.RobotMode.RUNNING,
                                                         stop_program=True, play_program=True)
    client_set_mode.send_goal(mode_action_goal)
    client_set_mode.wait_for_result(rospy.Duration.from_sec(5.0))
    res = client_set_mode.get_result()
    if not res.success:
        rospy.loginfo("Start program failed.")
        #exit()
    ## End of trigger

    #ur_control = MoveGroupPythonIntefaceTutorial()
    rospy.sleep(1)


    # ur_control.add_basebox()
    #ur_control.remove_allobjects()
    #ur_control.setup_scene()
    #add_camerabox(ur_control)
    #ur_control.group.get_current_pose()
    rospy.loginfo('init ok')

    # Home position
    #JS_READY= [-1.4280279318438929, -1.5552557150470179, -1.5555499235736292, -1.5861824194537562, 1.6085649728775024, -0.7085111776935022]
    JS_READY= [-2.0083072821246546, -1.3708232084857386, -1.5726779142962855, -1.753381077443258, 1.6079654693603516, -0.7075637022601526]
    #ur_control.go_to_joint_state(*JS_READY)
    rospy.sleep(1)

if __name__ == "__main__":

    rospy.init_node("initializer_node", anonymous=True)

    ## Trigger robot arm controller to play
    client_set_mode = actionlib.SimpleActionClient('/ur_hardware_interface/set_mode',
                                                   ur_dashboard_msgs.msg.SetModeAction)
    client_set_mode.wait_for_server()

    mode_action_goal = ur_dashboard_msgs.msg.SetModeGoal(target_robot_mode=ur_dashboard_msgs.msg.RobotMode.RUNNING,
                                                         stop_program=True, play_program=True)
    client_set_mode.send_goal(mode_action_goal)
    client_set_mode.wait_for_result(rospy.Duration.from_sec(5.0))
    res = client_set_mode.get_result()
    if not res.success:
        rospy.loginfo("Start program failed.")
        #exit()
    ## End of trigger

    ur_control = MoveGroupPythonIntefaceTutorial()
    rospy.sleep(1)

    JS_READY= [-2.0083072821246546, -1.3708232084857386, -1.5726779142962855, -1.753381077443258, 1.6079654693603516, -0.7075637022601526]
    ur_control.go_to_joint_state(*JS_READY)
    
    # 6 prime directions
    # ur_control.speedl_control([0.01,0,0,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([-0.01,0,0,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([0,0.01,0,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([0,-0.01,0,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([0,0,0.01,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([0,0,-0.01,0,0,0], 1, 1)
    # rospy.sleep(1)

    # # 8 diagnol directions
    # ur_control.speedl_control([0.01,0.01,0.01,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([-0.01,0.01,0.01,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([0.01,-0.01,0.01,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([0.01,0.01,-0.01,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([-0.01,-0.01,0.01,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([0.01,-0.01,-0.01,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([-0.01,0.01,-0.01,0,0,0], 1, 1)
    # rospy.sleep(1)
    # ur_control.speedl_control([-0.01,-0.01,-0.01,0,0,0], 1, 1)
    # rospy.sleep(1)



    # ur_control.speedl_control([0,0,0,0,0,0], 1, 1)
    rospy.sleep(1)
