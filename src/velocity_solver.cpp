#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <stdio.h>
#include <iostream>
#include <string>

#define LAMBDA 0.0001
#define JOINT_NUMBER 6
using namespace KDL;

//for IK_Vel use
JntArray q;

//for FK_Vel use
JntArrayVel qdot;

int map_joint_states[] = {2, 1, 0, 3, 4, 5};

//callback to get joint state;
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    if (msg->position.size() != JOINT_NUMBER) {
        std::cout << "wrong joint size" << '\n';
        return;
    }

    JntArray q_temp(JOINT_NUMBER);
    JntArray qdot_temp(JOINT_NUMBER);
    for (int i = 0; i < JOINT_NUMBER; i++) {
        q_temp(i) = msg->position[map_joint_states[i]];
        qdot_temp(i) = msg->velocity[map_joint_states[i]];
    }
    q = q_temp;
    qdot = JntArrayVel(q_temp, qdot_temp);

}

Twist v_in;

void cartVelCallback(const std_msgs::Float64MultiArray &msg) {
    std::cout << "received desired cartesian: ";
    v_in = Twist(Vector(msg.data[0], msg.data[1], msg.data[2]), Vector(0, 0, 0));
    std::cout << "eef velocity command" << " v_x:" << msg.data[0] << " v_y:" << msg.data[1] << " v_z:" << msg.data[2]
                  << " m/s ";
    std::cout << "\n";
}


int main(int argc, char **argv){
    ros::init(argc, argv, "velocity_solver", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;

    //subscrib topic /joint_state to get joint state
    ros::Subscriber subJointStates = node.subscribe("/joint_states", 1000, jointStatesCallback);
    ros::Subscriber subCartVelocities = node.subscribe("/eef_cart_velocity/command", 1000, cartVelCallback);

    //cartesian velocity of ee publisher
    ros::Publisher cart_vPublisher = node.advertise<std_msgs::Float64MultiArray>(
            "/ee_cart_velocity", 1);

    //cartesian velocity of ee publisher
    ros::Publisher cart_pPublisher = node.advertise<std_msgs::Float64MultiArray>(
            "/ee_cart_pos", 1);

    //joint velocity commander
    ros::Publisher jointGroupVelPublisher = node.advertise<std_msgs::Float64MultiArray>(
            "/joint_group_vel_controller/command", 1);

    std::string robot_desc_string;
    KDL::Tree tree;
    Chain chain;  //for FK_Vel use
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree)) {
        ROS_ERROR("Failed to construct kdl tree");
        return -1;
    }

    if (tree.getNrOfJoints() != JOINT_NUMBER) {
        ROS_ERROR("Wrong Joint Number:%d\n", tree.getNrOfJoints());
        return -1;
    }

    JntArray qdot_out(JOINT_NUMBER);

    //tree IK_Vel
    // std::vector<std::string> endpoints;
    // endpoints.push_back("ee_link");
    // TreeFkSolverPos_recursive fkSolver(tree);
    // TreeIkSolverVel_wdls ikSolverVel(tree, endpoints);
    // ikSolverVel.setLambda(LAMBDA);

    //for FK_Vel use
    //SegmentMap::const_iterator rootSegment = tree.getRootSegment();
    //std::cout << "root name:" << (*rootSegment).first << '\n';
    tree.getChain("base_link", "ee_link", chain);
    ChainFkSolverVel_recursive fkSolverVel(chain);

    ChainFkSolverPos_recursive fkSolver = ChainFkSolverPos_recursive(chain);

    //chain IK_VEL
    ChainIkSolverVel_pinv ikSolverVel(chain);

    //initialize v_in
    v_in = Twist(Vector(0, 0, 0), Vector(0, 0, 0));

    std_msgs::Float64MultiArray joint_vels;
    joint_vels.data.resize(JOINT_NUMBER);
    bool velocity_limit = false;
    ros::Rate rate(30);
    while (ros::ok()) {
        int ret_0 = -666;

        if (q.rows() > 0) {
            ret_0 = ikSolverVel.CartToJnt(q, v_in, qdot_out);
        }
        //compute IK joint Vel command and publish
        if (ret_0 == SolverI::E_NOERROR) {
            // std::cout << "Listening to topic /eef_cart_velocity/command\n";
            // std::cout << "Send Joint Velocities:" << "  ";
            for (int i = 0; i < JOINT_NUMBER; i++) {
                double joint_velocity;
                // tree method
                // joint_velocity = qdot_out(i);
                // chain method
                joint_velocity = qdot_out(i);

                if (!velocity_limit && (joint_velocity > 1.3 || joint_velocity < -1.3)) {
                    velocity_limit = true;
                    std::cout << "Joint Velocity reached the limit!" << '\n';
                }
                //reset velocity
                if (argc != 1 || velocity_limit) joint_velocity = 0;
                joint_vels.data[i] = joint_velocity;
                // std::cout << joint_velocity << " ";
            }
            // std::cout << '\n';
            jointGroupVelPublisher.publish(joint_vels);
        }
            //tree method
            //else if(ret == -666 ){
            //chain method
        else if (ret_0 == -666) {
            std::cout << "waiting for joint states data[Please Wait]" << '\n';
        }
        // std::cout << "------------------------" << '\n';

        //compute FK Vel from the robot state and publish
        FrameVel cart_v_real_temp;
        int ret = -1;
        if (qdot.qdot.rows() > 0){
            ret = fkSolverVel.JntToCart(qdot, cart_v_real_temp);
        }

        if (ret == SolverI::E_NOERROR){
            std_msgs::Float64MultiArray cart_v_real;
            cart_v_real.data.resize(3);

            for (int i = 0; i < 3; i++) {
                cart_v_real.data[i] = cart_v_real_temp.p.v[i];
            }

            cart_vPublisher.publish(cart_v_real);
        }

        //compute FK position from the robot state and publish
        Frame cartpos;
        // Calculate forward position kinematics
        int ret2 = -1;
        if (q.rows() > 0){
            ret2 = fkSolver.JntToCart(q, cartpos);
        }
        if (ret2 == SolverI::E_NOERROR){
            std_msgs::Float64MultiArray cart_p;
            cart_p.data.resize(3);
            for(int i = 0; i < 3; i++){
                cart_p.data[i] = cartpos.p[i];
            }
            cart_pPublisher.publish(cart_p);
        }else{
            std::cout << "cart position solver error" << std::endl;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;



}