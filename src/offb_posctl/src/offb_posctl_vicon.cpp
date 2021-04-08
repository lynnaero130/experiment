/*
 * position_control.cpp
 *
 * Author:mz
 *
 * Time: 2018.11.27
 *
 * 说明: mavros位置控制示例程序
 *      输入：mavros发布的位置/速度信息
 *      输出：无人机的推力和姿态信息
 *      采用位置环/速度环串级PID控制，位置环P控制，速度环PID控制
 */
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <pwd.h>
#include <stdio.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <Eigen/Core>
#include <dynamic_reconfigure/server.h>
#include <offb_Config.h>

#include <ros/ros.h>
#include "PID.h"
#include "FILTER.h"
#include "DOB.h"


//topic
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <offb_posctl/controlstate.h>

using namespace Eigen;
using namespace std;
// //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

geometry_msgs::PoseStamped pos_ref;         //无人机参考位置

mavros_msgs::State current_state;           //无人机当前状态(mode arm)
sensor_msgs::Imu   imu_drone;               //读入的无人机的IMU信息 包括姿态角和线加速度

geometry_msgs::PoseStamped  pos_drone;      //读入的无人机当前位置
geometry_msgs::PoseStamped  pos_drone_last; //读入的无人机上一次位置

geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度

geometry_msgs::Vector3 acc_receive;         //读入的无人机线加速度
geometry_msgs::Vector3 angle_receive;       //读入的无人机姿态（欧拉角）

geometry_msgs::Quaternion orientation_target;   //发给无人机的姿态指令

geometry_msgs::Vector3 angle_des;            //线性模型输出的理想值
//geometry_msgs::Vector3 angle_dis;            //DOB控制器估计的扰动值
geometry_msgs::Vector3 angle_target;            //经DOB控制器作用后的实际系统输入值
geometry_msgs::Vector3 vel_target, vel_read;
geometry_msgs::Vector3 pos_error;
geometry_msgs::Vector3 filter_in;
geometry_msgs::Vector3 filter_out;
geometry_msgs::Vector3 ref_pos_msg;
geometry_msgs::Vector3 temp_angle;

// debug data
geometry_msgs::Vector3 vel_vicon;
mavros_msgs::AttitudeTarget targetattitude;



float thrust_target;        //期望推力
float Yaw_Init, angle_init, angle_vicon, angle_deviation;
float Yaw_Locked = 0;           //锁定的偏航角(一般锁定为0)
float refPx=0,refPy=0,refPz=0,refVx=0,refVy=0,refVz=0,refAx=0,refAy=0,refAz=0;
float px_ini = -3.0;
float pz_ini = 0;
float py_ini=0;
float vx_ini = -0.1;
float vz_ini = 0.0;
float vy_ini=0;

PID PIDX, PIDY, PIDZ, PIDVX, PIDVY, PIDVZ;    //声明PID类
DOB DOBX, DOBY, DOBZ;                         //声明DOB类
FILTER FilterDerVY(10), FilterDerVZ(10);
FILTER FilterAY(10), FilterAZ(10);
offb_posctl::offb_Config param;
std::ofstream logfile;
std::ofstream debugfile;
offb_posctl::controlstate controlstate_msg;
const float MAX_POSITION_MEASURE_ERROR = 0.2;
bool startattitudecotrolflag=false;
bool planstopflag=false;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//欧拉角转四元数
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);
nav_msgs::Odometry current_relativepostwist_msg;
geometry_msgs::Point plane_expected_position;
geometry_msgs::Point plane_expected_velocity;
geometry_msgs::Point plane_expected_acceleration;
offb_posctl::controlstate controlstatearray_msg;
nav_msgs::Odometry  planned_postwist_msg;
mavros_msgs::AttitudeTarget target_atti_thrust_msg;

float get_ros_time(ros::Time time_begin);
int pix_controller(float cur_time);
float satfunc(float data, float Max, float Thres);
int set_file();
void data_log(std::ofstream &logfile, float cur_time);
void debug_log(std::ofstream &debugfile, float cur_time);
void readRefTrajectory();
void useRefTrajectory(int counter);
float impedancecontrol();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/**
 * 参数读取callback函数，用于获取飞行控制器的参数
 * @param config
 */
void param_cb(const offb_posctl::offb_Config &config)
{
    param = config;
}

/**
 * 通过callback函数获取期望的位置指令
 * @param msg
 */
void ref_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pos_ref = *msg;
}

/**
 * 通过callback函数获取无人机当前的飞行状态
 * @param msg
 */
void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

//获取无人机的IMU信息
bool hasGotImu = false;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    hasGotImu = true;
    imu_drone = *msg;
    acc_receive = imu_drone.linear_acceleration;
    angle_receive = quaternion2euler(imu_drone.orientation.x, imu_drone.orientation.y, imu_drone.orientation.z, imu_drone.orientation.w);
//    ROS_ERROR_STREAM("drone angle_receive.x:"<<angle_receive.x);
}

bool planeupdateflag= false;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pos_drone = *msg;
    planeupdateflag=true;
//    cout<<"pos_drone.pose.position.x"<<pos_drone.pose.position.x<<endl;
}

/**
 * 通过callback函数获取无人机当前的飞行速度
 * @param msg
 */
bool vel_initialized = false;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    if(!vel_initialized)
    {
        vel_initialized = true;
        vel_drone = *msg;
        vel_read = vel_drone.twist.linear;
        return;
    }
    vel_drone = *msg;
    // lowpass filter 1p
    vel_read.x = 0.25 * vel_drone.twist.linear.x + 0.75 * vel_read.x;
    vel_read.y = 0.25 * vel_drone.twist.linear.y + 0.75 * vel_read.y;
    vel_read.z = 0.25 * vel_drone.twist.linear.z + 0.75 * vel_read.z;

    vel_vicon = vel_drone.twist.linear;
}
bool contstaterecieveflag= false;
int controlcounter=0;
int discretizedpointpersecond=0;
void controlstate_cb(const offb_posctl::controlstate::ConstPtr &msg)
{
    if(planstopflag==false)
    {
        controlstatearray_msg = *msg;
        controlcounter = controlstatearray_msg.inicounter;
//    cout<<"controlcounter"<<controlcounter;
        if(contstaterecieveflag == false)//第一次回调时初始化，之后这个flag一直是true
        {
            contstaterecieveflag= true;
            discretizedpointpersecond=controlstatearray_msg.discrepointpersecond;
        }
//        ROS_ERROR_STREAM( "controlstatearray_msg.discrepointpersecond: "<<controlstatearray_msg.discrepointpersecond);
    }

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    // ros初始化，节点为position_control
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;

    // 通过参数服务器的方式，获取控制器的参数
    dynamic_reconfigure::Server<offb_posctl::offb_Config> server;
    dynamic_reconfigure::Server<offb_posctl::offb_Config>::CallbackType ff;
    ff = boost::bind(&param_cb, _1);
    server.setCallback(ff);

    // 【订阅】无人机当前状态/位置/速度信息
    ros::Subscriber pos_ref_sub  = nh.subscribe<geometry_msgs::PoseStamped>("/cmd/pos_ref", 10, ref_cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub   = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_cb);
//
//    //Gazebo 仿真数据
//    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
//    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 10, vel_cb);
    //vicon 数据
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mocap/pose", 10, pos_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mocap/vel", 10, vel_cb);
    ros::Subscriber controlstate_sub = nh.subscribe<offb_posctl::controlstate>("ocp/control_state", 1, controlstate_cb);
 // 【发布】飞机姿态/拉力信息 坐标系:NED系
//    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float32>("/cmd/thrust", 10);
//    ros::Publisher orientataion_pub = nh.advertise<geometry_msgs::Quaternion>("/cmd/orientation", 10);


    ros::Publisher error_pub = nh.advertise<geometry_msgs::Vector3>("/plot/pos_error", 10);
    ros::Publisher refPos_pub = nh.advertise<geometry_msgs::Vector3>("/plot/ref_pos", 10);
//    ros::Publisher filter_pub = nh.advertise<geometry_msgs::Vector3>("/plot/filter_out", 10);
//    ros::Publisher filter_pub2 = nh.advertise<geometry_msgs::Vector3>("/plot/filter_in", 10);

    //debug pub
    ros::Publisher viconvel_pub = nh.advertise<geometry_msgs::Vector3>("/plot/viconvel_read", 10);
    ros::Publisher filtervel_pub = nh.advertise<geometry_msgs::Vector3>("/plot/filtervel_read", 10);
    ros::Publisher actual_rpy_pub = nh.advertise<geometry_msgs::Vector3>("drone/current_rpy", 1);//飞机当前的rpy角
    ros::Publisher ocplan_postwist_pub = nh.advertise<nav_msgs::Odometry>("ocplan_positiontwist", 1);//bvp计算的期望位置

    //    ros::Publisher filtervel_pub2 = nh.advertise<geometry_msgs::Vector3>("/plot/filtervel2_read", 10);

    ros::Publisher targetvel_pub = nh.advertise<geometry_msgs::Vector3>("/plot/target_vel", 10);
    ros::Publisher target_atti_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("atucaltrajectory",1, true);
    ros::Publisher current_relativepostwist_pub = nh.advertise<nav_msgs::Odometry>("current_relative_postwist", 1);


    // if the rate is too high (for instance:100hz, mavros will be blocked, and the uav cannot be stable under any pid params, cry)
    ros::Rate rate(30.0);

    set_file();
    // 读取PID参数
    std::string paraadr("/home/lynn/catkin_ws/src/gazebo_ros_learning/offb_posctl/src/param");
    if (param.readParam(paraadr.c_str()) == 0) {
        std::cout << "read config file error!" << std::endl;
        return 0;
    }

//    // 设置位置环PID参数 比例参数 积分参数 微分参数
    PIDX.setPID(param.x_p, param.x_i, param.x_d);
    PIDY.setPID(param.y_p, param.y_i, param.y_d);
    PIDZ.setPID(param.z_p, param.z_i, param.z_d);

    // 设置位置环积分上限 控制量最大值 误差死区
    PIDX.set_sat(0.3, 4, 0.0);
    PIDY.set_sat(0.3, 4, 0.0);
    PIDZ.set_sat(0.4, 5, 0.0);

//    // 设置速度环PID参数 比例参数 积分参数 微分参数
//    PIDVX.setPID(param.vx_p, param.vx_i, param.vx_d);
//    PIDVY.setPID(param.vy_p, param.vy_i, param.vy_d);
//    PIDVZ.setPID(param.vz_p, param.vz_i, param.vz_d);
    // 设置速度环积分上限 控制量最大值 误差死区
    PIDVX.set_sat(0.5, 5, 0);
    PIDVY.set_sat(0.5, 5, 0);
    PIDVZ.set_sat(1.5, 5.0, 0);

    angle_target.x = 0;
    angle_target.y = 0;
    thrust_target = 0.3;


//    vel_read2.x = 0.0;
//    vel_read2.y = 0.0;
//    vel_read2.z = 0.0;

    // 等待和飞控的连接
    while(ros::ok() && current_state.connected == 0)
    {
        ros::Duration(1).sleep();
        ros::spinOnce();
        ROS_INFO("Not Connected");
    }
    ROS_INFO("Connected!!");

    // 等待获取无人机的IMU数据，用于无人机飞行过程中的偏航修正
    while(ros::ok() && !hasGotImu)
    {
        ros::Duration(1).sleep();
        ros::spinOnce();
        ROS_INFO_STREAM("waitting for IMU message ...");
    }
//    auto angle_receive = quaternion2euler(imu_drone.orientation.x, imu_drone.orientation.y, imu_drone.orientation.z, imu_drone.orientation.w);
    Yaw_Init = angle_receive.z;
    ROS_INFO_STREAM("Got Yaw_Init: " << Yaw_Init);

    plane_expected_position.z=0.3;// plus 0.3 to avoid the sudden stop when switch to offboard
    plane_expected_position.x=0;
    plane_expected_position.y=0;
    plane_expected_velocity.x=0;
    plane_expected_velocity.y=0;
    plane_expected_velocity.z=0;
    plane_expected_acceleration.x=0;
    plane_expected_acceleration.y=0;
    plane_expected_acceleration.z=0;

//    targetattitude.orientation.x = 0;
//    targetattitude.orientation.y = 0;
//    targetattitude.orientation.z = 0;
//    targetattitude.orientation.w = 1;
//    targetattitude.thrust = 0.1;
    //send a few setpoints before starting
//    for(int i = 5; ros::ok() && i > 0; --i){
////        target_atti_thrust_pub.publish(targetattitude);
//        pix_controller(0);
//        target_atti_thrust_msg.header.stamp = ros::Time::now();
//        target_atti_thrust_msg.orientation = orientation_target;
//        target_atti_thrust_msg.thrust = thrust_target;
//        target_atti_thrust_pub.publish(target_atti_thrust_msg);
//        ros::spinOnce();
//        rate.sleep();
//    }


    while(ros::ok() && current_state.mode != "OFFBOARD")
    {
//        target_atti_thrust_pub.publish(targetattitude);
        pix_controller(0);
        target_atti_thrust_msg.header.stamp = ros::Time::now();
        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_msg.thrust = thrust_target;
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Not OFFBOARD");
    }

    // 记录启控时间
    int quad_state=0;//0 climbhover, 1 AggressiveFly, 2 AdhesionPhase, 3 AdhesionFailure, 4 AdhesionSuccess
    int tempcounter=0;
    float ascentvel=15;
    float tempCurrentPx=0,tempCurrentPy=0,tempCurrentPz=0;
    float tempgoalPx=0,tempgoalPy=0,tempgoalPz=0;
    nav_msgs::Path actual_path;
    geometry_msgs::PoseStamped actual_pose_stamped;
    actual_path.header.stamp=ros::Time::now();
    actual_path.header.frame_id="ground_link";
    int pathseq=0;
    int lefnodeindex = 0;
    ros::Time begin_time_02 = ros::Time::now();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        float cur_time = get_ros_time(begin_time_02);  // 当前时间
        if (planeupdateflag && pos_drone.pose.position.z>=0.2)
        {
//           here we use orientation.y and z to contain the linear acceleration
            current_relativepostwist_msg.twist.twist.angular.y=FilterAY.filter(FilterDerVY.derivation((float)vel_drone.twist.linear.y,cur_time));
            current_relativepostwist_msg.twist.twist.angular.z=FilterAZ.filter(FilterDerVZ.derivation((float)vel_drone.twist.linear.z,cur_time));
            current_relativepostwist_msg.twist.twist.angular.y=min(max(current_relativepostwist_msg.twist.twist.angular.y,-2*9.8),2*9.8);
            current_relativepostwist_msg.twist.twist.angular.z=min(max(current_relativepostwist_msg.twist.twist.angular.z,-9.8),9.8);
            if(planstopflag==false)
            {
                px_ini = pos_drone.pose.position.x;
                py_ini = pos_drone.pose.position.y;
                pz_ini = pos_drone.pose.position.z;

                vx_ini = vel_drone.twist.linear.x;
                vy_ini = vel_drone.twist.linear.y;
                vz_ini = vel_drone.twist.linear.z;

                pz_ini=min(max((double)pz_ini, 0.2),2.5);
                vz_ini=min(max((double)vz_ini, -5.0),5.0);
                vy_ini=min(max((double)vy_ini, -5.0),5.0);

                current_relativepostwist_msg.pose.pose.position.x = px_ini;
                current_relativepostwist_msg.pose.pose.position.y = py_ini;
                current_relativepostwist_msg.pose.pose.position.z = pz_ini;

                current_relativepostwist_msg.twist.twist.linear.x = vx_ini;
                current_relativepostwist_msg.twist.twist.linear.y = vy_ini;
                current_relativepostwist_msg.twist.twist.linear.z = vz_ini;
//                current_relativepostwist_msg.pose.pose.orientation.y=FilterDerVY();
//                current_relativepostwist_msg.pose.pose.orientation.z=omegax_ini;
//            current_relativepostwist_msg.pose.pose.orientation.z=thrust_ini;
//            current_relativepostwist_msg.pose.pose.orientation.w=tau_ini;
                current_relativepostwist_msg.header.stamp = pos_drone.header.stamp;
                current_relativepostwist_pub.publish(current_relativepostwist_msg);
            }

            actual_pose_stamped.pose.position.x=pos_drone.pose.position.x;
            actual_pose_stamped.pose.position.y=pos_drone.pose.position.y;
            actual_pose_stamped.pose.position.z=pos_drone.pose.position.z;
//            actual_pose_stamped.pose.orientation=pose_drone_odom.pose.pose.orientation;
            actual_pose_stamped.header.stamp=ros::Time::now();
            actual_pose_stamped.header.frame_id="ground_link";
//            actual_pose_stamped.header.seq=i;
            actual_path.poses.push_back(actual_pose_stamped);
            path_pub.publish(actual_path);

//            std::cout << "py_ini:  " << py_ini << " pz_ini:  " << pz_ini << " phi_ini:  " << phi_ini << " vy_ini:  "
//                      << vy_ini <<" vz_ini: "<<vz_ini<<" omegax_ini: "<<omegax_ini<<" thrust_ini: "<<thrust_ini<<" tau_ini: "<<tau_ini<<std::endl;//输出,放到py文件中求解
            planeupdateflag = false;
        }

        ros::spinOnce();//刷新callback的消息
        cout<<"-------quad_state"<<quad_state<<endl;
        switch (quad_state) {
            case 0:
                plane_expected_position.z=plane_expected_position.z+ascentvel*cur_time;// plus 0.3 to avoid the sudden stop when switch to offboard
                plane_expected_position.z=min(plane_expected_position.z,0.5);
                plane_expected_position.x=0;
                plane_expected_position.y=0;
                plane_expected_velocity.x=0;
                plane_expected_velocity.y=0;
                plane_expected_velocity.z=0;
                plane_expected_acceleration.x=0;
                plane_expected_acceleration.y=0;
                plane_expected_acceleration.z=0;

                pix_controller(cur_time);

                if(plane_expected_position.z>=0.5)
                {
                    tempcounter++;
                    if(tempcounter>=150)
                    {
                        quad_state=1;
                        controlcounter=1;
                        tempcounter=0;
                    }
                }
                break;
            case 1:
//                cout<<"-------controlcounter"<<controlcounter<<endl;
                if(contstaterecieveflag)  //订阅到bvp计算的控制量则flag为true,用于起始时刻,还没算出bvp时
                {
                    lefnodeindex = controlcounter;
                    cout<<"-------lefnodeindex"<<lefnodeindex<<endl;
                    if (lefnodeindex < controlstatearray_msg.arraylength)
                    {
//                        Here,we used lefnodeindex-1, it is because we consider that the pos and vel error is in next state, not in current state
                        plane_expected_position.x=controlstatearray_msg.stateXarray[lefnodeindex];
                        plane_expected_position.y=controlstatearray_msg.stateYarray[lefnodeindex];
                        plane_expected_position.z=controlstatearray_msg.stateZarray[lefnodeindex];


                        plane_expected_velocity.x=controlstatearray_msg.stateVXarray[lefnodeindex];
                        plane_expected_velocity.y=controlstatearray_msg.stateVYarray[lefnodeindex];
                        plane_expected_velocity.z=controlstatearray_msg.stateVZarray[lefnodeindex];

                        plane_expected_acceleration.x=controlstatearray_msg.stateAXarray[lefnodeindex];
                        plane_expected_acceleration.y=controlstatearray_msg.stateAYarray[lefnodeindex];
                        plane_expected_acceleration.z=controlstatearray_msg.stateAZarray[lefnodeindex];
//                        ROS_ERROR_STREAM( "lefnodeindex: "<<lefnodeindex<<" controlstatearray_msg.arraylength: "<<controlstatearray_msg.arraylength<<" controlstatearray_msg.stateAZarray[lefnodeindex]:"<<controlstatearray_msg.stateAZarray[lefnodeindex]<<" controlstatearray_msg.stateAZarray[last]:"<<controlstatearray_msg.stateAZarray[controlstatearray_msg.arraylength-1]);

                    }
                    controlcounter++;
//                    if(controlcounter>=controlstatearray_msg.arraylength ||controlstatearray_msg.arraylength<=10)
                    if((fabs(plane_expected_position.y-controlstatearray_msg.stateYarray[controlstatearray_msg.arraylength-1])<=0.1 && fabs(plane_expected_position.z-controlstatearray_msg.stateZarray[controlstatearray_msg.arraylength-1])<=0.1))
//                    if((controlstatearray_msg.arraylength/controlstatearray_msg.discrepointpersecond)<1.0)
                    {
                        planstopflag= true;
                        ROS_ERROR_STREAM( "plane_expected_position.y: "<<plane_expected_position.y<<" plane_expected_position.z: "<<plane_expected_position.z<<" arraylength:"<<controlstatearray_msg.arraylength);
                    }
                    if((controlstatearray_msg.arraylength-controlcounter)<0.15*controlstatearray_msg.discrepointpersecond)
                    {
                        startattitudecotrolflag=true;
                        ROS_ERROR_STREAM("startattitudecotrolflag:"<<startattitudecotrolflag<<" leftindex: "<<lefnodeindex<<" left_controlpoints: "<<controlstatearray_msg.arraylength-controlcounter<<" plane_expected_position.z: "<<plane_expected_position.z<<" pos_drone.pose.position.z: "<<pos_drone.pose.position.z);
//                        ROS_ERROR_STREAM( "(controlstatearray_msg.arraylength-controlcounter): "<<(controlstatearray_msg.arraylength-controlcounter)<<" lefttime:"<<((controlstatearray_msg.arraylength-controlcounter)/controlstatearray_msg.discrepointpersecond));
                    }
                    if(controlcounter>=controlstatearray_msg.arraylength)
                    {
                        ROS_ERROR_STREAM( "plane_expected_position.y:"<<plane_expected_position.y<<" plane_expected_position.z:"<<plane_expected_position.z<<" current z: "<<pos_drone.pose.position.z);
                        quad_state=2;
                    }
//                    planned_postwist_msg.pose.pose.orientation.x=-atan(controlstatearray_msg.stateAYarray[lefnodeindex]/(controlstatearray_msg.stateAZarray[lefnodeindex]+9.8));
                }
//                cout<<"------leftindex:----"<<lefnodeindex<<endl;
                pix_controller(cur_time);

                planned_postwist_msg.pose.pose.position.x=plane_expected_position.x;
                planned_postwist_msg.pose.pose.position.y=plane_expected_position.y;
                planned_postwist_msg.pose.pose.position.z=plane_expected_position.z;


                planned_postwist_msg.twist.twist.linear.x=plane_expected_velocity.x;
                planned_postwist_msg.twist.twist.linear.y=plane_expected_velocity.y;
                planned_postwist_msg.twist.twist.linear.z=plane_expected_velocity.z;

                planned_postwist_msg.twist.twist.angular.x=PIDVX.Output;
                planned_postwist_msg.twist.twist.angular.y=PIDVY.Output;
                planned_postwist_msg.twist.twist.angular.z=PIDVZ.Output;

                planned_postwist_msg.pose.pose.orientation.x=-atan(PIDVY.Output/(PIDVZ.Output+9.8));
                ROS_INFO_STREAM("Apporaching_thrust_target: "<< thrust_target<<" roll:"<<angle_target.x<<" pitch:"<<angle_target.y<<" yaw:"<<angle_target.z);
                break;
            case 2:
                startattitudecotrolflag=false;
                tempcounter++;
                if(tempcounter<=controlstatearray_msg.parabolictime*controlstatearray_msg.discrepointpersecond)
                {
                    tempgoalPx=pos_drone.pose.position.x;
                    tempgoalPy=pos_drone.pose.position.y;
                    tempgoalPz=pos_drone.pose.position.z;
                    cout<<"tempcounter: "<<tempcounter<<"  parabolictime*30: "<<controlstatearray_msg.parabolictime*30<<endl;
                    ROS_ERROR_STREAM("temp goal goal goal Px:"<<tempgoalPx<<" Py: "<<tempgoalPy<<" Pz: "<<tempgoalPz<<" pitch:"<<angle_receive.x);
                    thrust_target=param.THR_HOVER;
                    break;
                }
                tempCurrentPx=pos_drone.pose.position.x;
                tempCurrentPy=pos_drone.pose.position.y;
                tempCurrentPz=pos_drone.pose.position.z;
                ROS_ERROR_STREAM("current Px:"<<tempCurrentPx<<" Py: "<<tempCurrentPy<<" Pz: "<<tempCurrentPz<<" pitch:"<<angle_receive.x);
              if(tempCurrentPz<controlstatearray_msg.wall_z-0.2||vel_read.z<-0.8)
//                if(tempCurrentPz<controlstatearray_msg.wall_z-0.25||tempCurrentPy<controlstatearray_msg.wall_y-0.20)
                {
                    quad_state=3;
                    tempcounter=0;
                }
              if(tempcounter>=30&&(vel_read.z>=-0.8||tempCurrentPz>=controlstatearray_msg.wall_z-0.2))
//                if(tempcounter>=30&&tempCurrentPz>=controlstatearray_msg.wall_z-0.15)
                {
                    quad_state=4;
                    tempcounter=0;
                }
                orientation_target = euler2quaternion(-atan(controlstatearray_msg.stateAYarray[lefnodeindex]/(controlstatearray_msg.stateAZarray[lefnodeindex]+9.8)), 0, angle_target.z);
                thrust_target  = param.THR_HOVER*cos(-atan(controlstatearray_msg.stateAYarray[lefnodeindex]/(controlstatearray_msg.stateAZarray[lefnodeindex]+9.8)))*cos(0);   //目标推力值 to alleviate the gravity's component along the drone's z axis
                thrust_target  = max((double)thrust_target,param.THR_HOVER*0.5);   //目标推力值,只是用来保证提供扭矩，the drone is easy to fall freely and crash
//                thrust_target=impedancecontrol();// this may make the attitude out of control when the thrust is too low and cause the collapse in test without wall
                ROS_INFO_STREAM("Duringsuck_thrust_target: "<< thrust_target<<" roll:"<<-atan(controlstatearray_msg.stateAYarray[lefnodeindex]/(controlstatearray_msg.stateAZarray[lefnodeindex]+9.8))<<" pitch:"<<0<<" yaw:"<<angle_target.z);
                break;
            case 3:
                plane_expected_position.x=tempCurrentPx;
                plane_expected_position.y=tempCurrentPy;
                plane_expected_position.z=tempCurrentPz;
                plane_expected_velocity.x=0;
                plane_expected_velocity.y=0;
                plane_expected_velocity.z=0;
                plane_expected_acceleration.x=0;
                plane_expected_acceleration.y=0;
                plane_expected_acceleration.z=0;
                pix_controller(cur_time);
                orientation_target = euler2quaternion(min(max(angle_target.x,-0.3),+0.3), angle_target.y, angle_target.z);

                break;
            case 4:
                orientation_target = euler2quaternion(-atan(controlstatearray_msg.stateAYarray[lefnodeindex]/(controlstatearray_msg.stateAZarray[lefnodeindex]+9.8)), 0, angle_target.z);
                thrust_target  = 0;   //目标推力值
                ROS_INFO_STREAM("Sucksuccess_thrust_target: "<< thrust_target<<" roll:"<<angle_target.x<<" pitch:"<<angle_target.y<<" yaw:"<<angle_target.z);
                break;
            default:
                break;
        }
//        if(quad_state!=4&&quad_state!=2)
//        {
//            pix_controller(cur_time);
//        }
        cout<<"refpos x: "<<plane_expected_position.x<<"   y:"<<plane_expected_position.y<<"  z:"<<plane_expected_position.z<<endl;

        ///publish plane current rpy
//        temp_angle = quaternion2euler(pos_drone.pose.orientation.x,
//                                      pos_drone.pose.orientation.y,
//                                      pos_drone.pose.orientation.z,
//                                      pos_drone.pose.orientation.w);//欧拉角
        actual_rpy_pub.publish(angle_receive);
//        cout<<"drone_roll--"<<temp_angle.x<<endl;

        ///publish thrust & orientation
//            std::cout << "thrust_target: " << thrust_target << std::endl;
        target_atti_thrust_msg.header.stamp = ros::Time::now();
        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_msg.thrust = thrust_target;
        target_atti_thrust_pub.publish(target_atti_thrust_msg);

        ///publish planned pos&twist in x&z
        planned_postwist_msg.header.stamp = ros::Time::now();
        ocplan_postwist_pub.publish(planned_postwist_msg);

        rate.sleep();

    }
    logfile.close();
    return 0;
}

/**
 * 获取当前时间 单位：秒
 */
float get_ros_time(ros::Time time_begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-time_begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - time_begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}


/**
 * 控制函数，采用了位置环、速度环串级PID控制，同时可选择增加DOB鲁棒控制模块
 * @param cur_time
 * @return
 */
int pix_controller(float cur_time)
{

//位 置 环
    //计算误差
    PIDX.setPID(param.MC_X_P, param.MC_X_I, 0);
    PIDY.setPID(param.MC_Y_P, param.MC_Y_I, 0);
    PIDZ.setPID(param.MC_Z_P, param.MC_Z_I, 0);
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDX.start_intergrate_flag = true;
    PIDY.start_intergrate_flag = true;
    PIDZ.start_intergrate_flag = true;
    if(current_state.mode != "OFFBOARD"){
        PIDX.start_intergrate_flag = false;
        PIDY.start_intergrate_flag = false;
        PIDZ.start_intergrate_flag = false;
    }
    float error_x = plane_expected_position.x - pos_drone.pose.position.x;
    float error_y = plane_expected_position.y - pos_drone.pose.position.y;
    float error_z = plane_expected_position.z - pos_drone.pose.position.z;
    PIDX.add_error(error_x, cur_time);
    PIDY.add_error(error_y, cur_time);
    PIDZ.add_error(error_z, cur_time);
    //计算输出
    PIDX.pid_output();
    PIDY.pid_output();
    PIDZ.pid_output();

//    std::cout << "current: x：" << pos_drone.pose.position.x << "\ty：" << pos_drone.pose.position.y << "\tz：" << pos_drone.pose.position.z << std::endl;
//    std::cout << "error: x：" << error_x << "\ty：" << error_y << "\tz：" << error_z << std::endl;
    float acc_xd = PIDX.Output;
    float acc_yd = PIDY.Output;
    float acc_zd = PIDZ.Output;

    //计算误差
    float error_vx = plane_expected_velocity.x - vel_read.x;
    float error_vy = plane_expected_velocity.y - vel_read.y;
    float error_vz = plane_expected_velocity.z - vel_read.z;
    float acc_vxd = param.MC_VX_P * error_vx;
    float acc_vyd = param.MC_VY_P * error_vy;
    float acc_vzd = param.MC_VZ_P * error_vz;

//    float feedforwardcoefx=(fabs(plane_expected_acceleration.x)+1)/(fabs(plane_expected_acceleration.x)+1+fabs((acc_xd+acc_vxd)*(acc_xd+acc_vxd-plane_expected_acceleration.x)));
//    float feedforwardcoefy=(fabs(plane_expected_acceleration.y)+1)/(fabs(plane_expected_acceleration.y)+1+fabs((acc_yd+acc_vyd)*(acc_yd+acc_vyd-plane_expected_acceleration.y)));
//    float feedforwardcoefz=(fabs(plane_expected_acceleration.z)+1)/(fabs(plane_expected_acceleration.z)+1+fabs((acc_zd+acc_vzd)*(acc_zd+acc_vzd-plane_expected_acceleration.z)));
    //计算输出
    float feedforwardcoefx=1/(1+fabs(acc_xd+acc_vxd)/(fabs(plane_expected_acceleration.x)+0.5));//0.5 is to avoid the zero of denominator
    float feedforwardcoefy=1/(1+fabs(acc_yd+acc_vyd)/(fabs(plane_expected_acceleration.y)+0.5));
    float feedforwardcoefz=1/(1+fabs(acc_zd+acc_vzd)/(fabs(plane_expected_acceleration.z)+0.5));
    cout<<"feedforwardcoefx:"<<feedforwardcoefx<<" y:"<<feedforwardcoefy<<" z:"<<feedforwardcoefz<<endl;

    if(startattitudecotrolflag==false)
    {
        PIDVX.Output=acc_xd+acc_vxd+feedforwardcoefx*plane_expected_acceleration.x;
        PIDVY.Output=acc_yd+acc_vyd+feedforwardcoefy*plane_expected_acceleration.y;;
        PIDVZ.Output=acc_zd+acc_vzd+feedforwardcoefz*plane_expected_acceleration.z;;
    }else{
        PIDVX.Output=plane_expected_acceleration.x;
        PIDVY.Output=plane_expected_acceleration.y;
        PIDVZ.Output=plane_expected_acceleration.z;
    }

    cout<<"acc_p_error  planz_a:"<<plane_expected_acceleration.z<<"  vz_a:"<<acc_vzd<<"  z:"<<acc_zd<<" roll: "<<angle_receive.x<<" PIDVZ.Output: "<<PIDVZ.Output<<endl;
//    cout<<"acc_v_error  x:"<<acc_vxd<<"  y:"<<acc_vyd<<"  z:"<<acc_vzd<<endl;
//    cout<<"PID refAcc  x:"<<plane_expected_acceleration.x<<"  y:"<<plane_expected_acceleration.y<<"  z:"<<plane_expected_acceleration.z<<endl;
//    cout<<"PID out_acc  x:"<<PIDVX.Output<<"  y:"<<PIDVY.Output<<"  z:"<<PIDVZ.Output<<endl;


    angle_target.x = asin(-PIDVY.Output/sqrt(pow(PIDVX.Output,2)+pow(PIDVY.Output,2)+pow(PIDVZ.Output+9.8,2)));
    angle_target.y = atan(PIDVX.Output/(PIDVZ.Output+9.8));
    angle_target.z = Yaw_Init;

//    // yaw 角度修正
//    angle_vicon = pos_drone.pose.orientation.y;     //此处用的yaw角和vicon实际建坐标系相关,需确认
////    std::cout << "Euler_vicon: yaw：" << angle_vicon * 60 << std::endl;
//    // std::cout << "Euler_vicon: roll：" << pos_drone.pose.orientation.x << "\tpitch：" << pos_drone.pose.orientation.y << "\tyaw：" << pos_drone.pose.orientation.z << std::endl;
//    angle_deviation = angle_vicon - angle_init;
//    if (fabs(angle_deviation) < 0.05)
//    {
//        angle_deviation = 0;
//    }
//    angle_target.z = angle_target.z + angle_deviation;

    orientation_target = euler2quaternion(min(angle_target.x+0.0*(angle_target.x-angle_receive.x),1.57), angle_target.y, angle_target.z);
    thrust_target  = (float)sqrt(pow(PIDVX.Output,2)+pow(PIDVY.Output,2)+pow(PIDVZ.Output+9.8,2))/9.8*(param.THR_HOVER);   //目标推力值
    ROS_INFO_STREAM("thrust_target: "<< thrust_target<<" roll:"<<angle_target.x<<" pitch:"<<angle_target.y<<" yaw:"<<angle_target.z);

    return 0;
}

/**
 * 将欧拉角转化为四元数
 * @param roll
 * @param pitch
 * @param yaw
 * @return 返回四元数
 */
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

/**
 * 将四元数转化为欧拉角形式
 * @param x
 * @param y
 * @param z
 * @param w
 * @return 返回Vector3的欧拉角
 */
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    temp.y = -asin(2.0 * (z * x - w * y));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

/**
 * 限幅函数
 * @param data 输入的数据
 * @param Max 数据的最大值
 * @param Thres 零死区
 * @return 返回输出值
 */

/**
 * 设置log文件的储存方式
 * @return
 */
int set_file()
{
    // 更新home的文件夹位置，在home目录下新建OFFBOARD_PX4_log文件夹，用于存放log数据
    id_t uid;
    struct passwd* pwd;
    uid = getuid();
    pwd = getpwuid(uid);
    chdir(pwd->pw_dir);
    mkdir("./HGJ_PX4_log", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // 获取系统时间，作为log文件的文件名进行保存
    time_t currtime = time(nullptr);
    tm* p = localtime(&currtime);
    char log_filename[100] = {0};
    char debug_filename[100] = {0};


    sprintf(log_filename,"./OFFBOARD_PX4_log/log_%d%02d%02d_%02d_%02d_%02d.csv",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
    sprintf(debug_filename,"./OFFBOARD_PX4_log/debug_%d%02d%02d_%02d_%02d_%02d.csv",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);


    // log输出文件初始化
    logfile.open(log_filename, std::ios::out);
    if (! logfile.is_open()){
        std::cerr<<"log to file error!"<<std::endl;
        return -1;
    }

    // debug输出文件初始化
    debugfile.open(debug_filename , std::ios::out);
    if (! debugfile.is_open()){
        std::cerr<<"debug to file error!"<<std::endl;
        return -1;
    }

}


/**
 * 将进入offboard后的位置&速度&姿态信息记录进文件
 * @param cur_time
 */
void data_log(std::ofstream &logfile, float cur_time)
{
    logfile << cur_time << "," << param.DOB_rate << ","
        <<pos_ref.pose.position.x <<","<<pos_ref.pose.position.y <<","<<pos_ref.pose.position.z <<","    //set_pos
        <<pos_drone.pose.position.x <<","<<pos_drone.pose.position.y <<","<<pos_drone.pose.position.z <<","    //uav_pos
        <<vel_target.x <<","<<vel_target.y <<","<<vel_target.z <<","                                           //set_vel
        <<vel_drone.twist.linear.x <<","<<vel_drone.twist.linear.y <<","<<vel_drone.twist.linear.z <<","       //uav_vel
//        <<angle_target.x  <<","<<angle_target.y  <<","<<angle_target.z  <<","                                  //set_att
//        <<angle_receive.x <<","<<angle_receive.y <<","<<angle_receive.z <<","                                  //uav_att
//        <<acc_receive.x   <<","<<acc_receive.y   <<","<<acc_receive.z   <<","                                  //uav_acc
        <<thrust_target<<std::endl;

}