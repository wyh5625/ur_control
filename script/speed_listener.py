import rospy
from std_msgs.msg import Float64MultiArray

from ur_move import MoveGroupPythonIntefaceTutorial, gen_pose
import actionlib
import ur_dashboard_msgs.msg

global ur_control

def cartVelCallback(msg):
    global ur_control
    ur_control.speedl_control([-msg.data[0],-msg.data[1],msg.data[2],0,0,0], 1, 1)



if __name__ == "__main__":
    global ur_control
    rospy.init_node("speed_listener", anonymous=True)
    rospy.Subscriber("/ur_speed", Float64MultiArray, cartVelCallback)



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
    #rospy.sleep(1)
    ur_control.group.get_current_pose()
    JS_READY= [-2.0083072821246546, -1.3708232084857386, -1.5726779142962855, -1.753381077443258, 1.6079654693603516, -0.7075637022601526]
    ur_control.go_to_joint_state(*JS_READY)
    rospy.loginfo('init ok')

    rospy.sleep(1)

    rospy.spin()

