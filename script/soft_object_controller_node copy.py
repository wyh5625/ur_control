import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
import tensorflow as tf
from tensorflow import keras
from numpy import sqrt, sign
import numpy as np
from copy import copy

#from ur_move import MoveGroupPythonIntefaceTutorial, gen_pose
import actionlib
import ur_dashboard_msgs.msg

global FPS, M, N, cart_v_real, contorl_gain, MAX_VEL, TOLERANCE, p, received, getFeedbackPoint, moved, startGo, target, MINIMUM_DATA_SIZE, model, ur_control

def build_model():
    m = keras.Sequential([
        keras.layers.Dense(16, activation=tf.nn.relu, input_shape=(M,)),
        keras.layers.Dense(16, activation=tf.nn.relu),
        keras.layers.Dense(N)
    ])
    optimizer = tf.optimizers.RMSprop(0.001)
    m.compile(loss='mse', optimizer=optimizer, metrics=['mae'])
    return m

def fit_model(train_data, train_labels):
    global model
    model.fit(train_data, train_labels, epochs=10, validation_split=0.2, verbose=0)

def norm(x):
    scale = 2.2250738585072014E-308
    y = 0.0
    for k in range(3):
        absxk = abs(x[k])
        if absxk > scale:
            t = scale / absxk
            y = 1.0 + y * t * t
            scale = absxk
        else:
            t = absxk / scale
            y += t * t
    return scale * sqrt(y)

def getFeatures():
    c_p = copy(p)
    # if c_p[0] > c_p[3]:
    #     for i in range(3):
    #         c_p[i], c_p[i+3] = c_p[i+3], c_p[i]
    return c_p

def highPass(l, threshold):
    return list(0.0 if i < threshold else i for i in l)

def cartVelCallback(msg):
    global cart_v_real, moved, received
    cart_v_real = copy(msg.data)
    cart_v_real = highPass(cart_v_real, 1e-4)
    for i in cart_v_real:
        if not i == 0:
            moved = True
    #moved = True
    received = True

def feedbackCallback(msg):
    global p, getFeedbackPoint
    p = copy(msg.data)
    getFeedbackPoint = True

def startCallback(msg):
    global startGo
    startGo = msg.data

def NN_controller():
    global received, moved, getFeedbackPoint, model, ur_control

    
    cartVelPublisher = rospy.Publisher('/eef_cart_velocity/command', Float64MultiArray, queue_size=1)
    rate = rospy.Rate(FPS)

    x = x_old = x_v = x_star = xd = y = y_predict = [0] * 3
    train_data = None
    label = None

    first_time = True
    stop_flag = False
    count = 0
    model = None
    minimum_data_requirement = False
    while not rospy.is_shutdown():
        if stop_flag:
            if startGo:
                count += 1
                cartVelPublisher.publish((0,)*M)
                #ur_control.speedl_control([0,0,0,0,0,0], 1, 1)
            print ("Reached the target SUCCESSFULLY")
            # # Only for hole-task
            # if count*1.0/FPS > 0.5 and count*1.0/FPS < 1.2:
            #     armCommand = [0] * 6
            #     armCommand[2] = armCommand[5] = -0.06
            #     cartVelPublisher.publish(tuple(armCommand))
            # elif count*1.0/FPS >= 1.2 and count*1.0/FPS < 2:
            #     cartVelPublisher.publish((0,)*M)
            # elif count*1.0/FPS >= 2:
            #     break
            rate.sleep()
            # continue
            break

        if received and getFeedbackPoint:
            x_old = copy(x)
            x = getFeatures()
            x_v = list((x[i] - x_old[i]) * FPS for i in range(N))
            x_star = list(target[i] - x[i] for i in range(N))
            y = copy(cart_v_real)
            # y = highPass(y, 1e-4)
            if moved:
                if first_time:
                    model = build_model()
                    train_data = np.array([x_star])
                    label = np.array([x_star])
                    first_time = False
                else:
                    train_data = np.concatenate((train_data, np.asarray([x_v])))
                    label = np.concatenate((label, np.asarray([y])))
                    fit_model(train_data, label)
                    print (train_data.shape)
                    if train_data.shape[0] > MINIMUM_DATA_SIZE:
                        minimum_data_requirement = True
                        y_predict = model.predict(np.array([x_star])).flatten()
            received = False
            getFeedbackPoint = False
            moved = False

            stopFlag = True
            print ("deltX:")
            for i in range(N):
                if abs(target[i] - x[i]) > TOLERANCE:
                    stopFlag = False
                print (target[i] - x[i])
            #print

            if not minimum_data_requirement:
                print ("Need more data")
            else:
                print ("Velocity: ")
                armCommand = list(i*contorl_gain if abs(i*contorl_gain) < MAX_VEL else MAX_VEL*sign(i) for i in y_predict)
                print (armCommand)

                if startGo:
                    p_m = Float64MultiArray(data=armCommand)
                    cartVelPublisher.publish(p_m)
                    #ur_control.speedl_control([armCommand[0],armCommand[1],armCommand[2],0,0,0], 1, 1)
                    print ("sended velocity command")
            print ("-----------------------")
            #print

        else:
            print ("Didn't receive the cart_v_real or get feedback points")

        rate.sleep()

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
    global FPS, M, N, cart_v_real, contorl_gain, MAX_VEL, TOLERANCE, p, received, getFeedbackPoint, moved, startGo, target, MINIMUM_DATA_SIZE
    FPS = 10
    N = 3
    M = 3
    cart_v_real = []
    contorl_gain = 1
    MAX_VEL = 0.06
    TOLERANCE = 0.006
    p = []
    startGo = received = getFeedbackPoint = moved = False
    target = [0.04462408181279898, 0.10905619338154793, 0.802609458565712]
    MINIMUM_DATA_SIZE = 50

    rospy.init_node("soft_object_controller_demo", anonymous=True)
    rospy.Subscriber("/ee_cart_velocity", Float64MultiArray, cartVelCallback)
    rospy.Subscriber("/soft_object_tracking/centroid", Float64MultiArray, feedbackCallback)
    rospy.Subscriber("/ur5/go", Bool, startCallback)
    

    try:
        init_robot_position()
        NN_controller()
    except rospy.ROSInterruptException:
        pass
