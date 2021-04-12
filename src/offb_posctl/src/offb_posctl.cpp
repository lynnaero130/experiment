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
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <Eigen/Core>


#include <ros/ros.h>
#include "Parameter.h"
#include <PID.h>


//topic
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <chrono>
#include "iomanip"
#include <thread>
#include <unistd.h>
#include <mutex>


using namespace Eigen;//释放eigen命名空间 矩阵库
using namespace std;
//using namespace message_filters;
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
geometry_msgs::PoseStamped pos_ref;         //无人机参考位置
mavros_msgs::State current_state;           //无人机当前状态
geometry_msgs::PoseStamped pose_drone_odom;       //读入的无人机drone当前位置，x，y，z+姿态
sensor_msgs::Imu pose_drone_Imu;       //读入的无人机drone当前位置，x，y，z+姿态
geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度 线速度+角速度
geometry_msgs::Quaternion orientation_target;   //发给无人机的姿态指令  四元数
geometry_msgs::Vector3 angle_target;   //欧拉角
geometry_msgs::Vector3 vel_target;   //期望速度
geometry_msgs::Point plane_expected_position;
mavros_msgs::AttitudeTarget target_atti_thrust_msg; //最终发布的消息 油门+角度
geometry_msgs::Vector3 angle_receive;
std_msgs::Int64 counter;


float thrust_target;        //期望推力
float Yaw_Init;
bool got_initial_point = false;
PID PIDVX, PIDVY, PIDVZ;    //声明PID类
Parameter param;
std::ofstream logfile;
int controlfreq=25;//50;
float init_pos_x = -0.5;
float init_pos_y = -0.5;
float init_pos_z = 0.5;

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//欧拉角转四元数
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);//geometry_msgs的Quaternion类型的函数
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);

float get_ros_time(ros::Time time_begin);
int pix_controller(float cur_time);
void vector3dLimit(Vector3d &v, double limit) ; ///limit should be positive
Vector3d vectorElementMultiply(Vector3d v1, Vector3d v2);
void data_log(std::ofstream &logfile, float cur_time, float ros_time);
int ReadRowNumber(string fileway);
void Readdata(string fileway,vector<vector<double>> &ans);
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;

}//当有消息到达topic时会自动调用一次
bool planeupdateflag= true;
void plane_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pose_drone_odom = *msg;//pose_drone_odom是nav_msgs::Odometry类型
    planeupdateflag= true;

}
void plane_imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    pose_drone_Imu = *msg;//pose_drone_odom是nav_msgs::Odometry类型
	angle_receive = quaternion2euler(pose_drone_Imu.orientation.x, pose_drone_Imu.orientation.y, pose_drone_Imu.orientation.z, pose_drone_Imu.orientation.w);
}

void plane_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    vel_drone = *msg;
}

void ref_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pos_ref = *msg;
}

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)//argc  argument count 传参个数，argument value
{
    ros::init(argc, argv, "position_control");//初始化节点名称
    ros::NodeHandle nh;

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
            "mavros/cmd/arming"); //使能解锁飞机  创建client对象并向arming发出请求，服务类型为CommandBool
    ros::ServiceClient setmode_client = nh.serviceClient<mavros_msgs::SetMode>(
            "mavros/set_mode"); //设置为自动控制模式 服务类型为SetMode
    // 【订阅】
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1,
                                                                 state_cb);
    ros::Subscriber plane_position_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mocap/pose", 1,
                                                                               plane_pos_cb);//pos+twist
    ros::Subscriber plane_poseimu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1,
                                                                       plane_imu_cb);//pos+twist
    ros::Subscriber plane_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(
            "/mocap/vel", 1, plane_vel_cb); //twist

    ros::Subscriber pos_ref_sub  = nh.subscribe<geometry_msgs::PoseStamped>("/cmd/pos_ref", 10, ref_cb);

    // 【发布】
    ros::Publisher target_atti_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",1);
    ros::Publisher counter_pub = nh.advertise<std_msgs::Int64>("counter",1);
	ros::Publisher actual_rpy_pub = nh.advertise<geometry_msgs::Vector3>("drone/current_rpy", 1);//飞机当前的rpy角
    ros::Rate rate(controlfreq);   //50hz的频率发送/接收topic  ros与pixhawk之间,50Hz control frequency

    // log输出文件初始化
    logfile.open("/home/uav/lzy_ws/src/offb_posctl/log/log.csv", std::ios::out);

    if (!logfile.is_open()) {
        ROS_ERROR("log to file error!");
        return 0;
    }

    // 读取PID参数
    std::string paraadr("/home/uav/lzy_ws/src/offb_posctl/src/param");
    if (param.readParam(paraadr.c_str()) == 0) {
        std::cout << "read config file error!" << std::endl;
        return 0;
    }

    /// 设置速度环PID参数 比例参数 积分参数 微分参数
    PIDVX.setPID(param.vx_p, param.vx_i, param.vx_d);
    PIDVY.setPID(param.vy_p, param.vy_i, param.vy_d);
    PIDVZ.setPID(param.vz_p, param.vz_i, param.vz_d);
    // 设置速度环积分上限 控制量最大值 误差死区
    PIDVX.set_sat(6, 10, 0);
    PIDVY.set_sat(2, 3, 0);
    PIDVZ.set_sat(2, 5, 0);
    cout<<"????????????????????????"<<endl;
    /// 读取期望轨迹
    ifstream fin_u1;// the instance can be used to read the file
//    string fileway = "/home/uav/lzy_ws/src/offb_posctl/data/line.csv";
//    string fileway = "/home/uav/lzy_ws/src/offb_posctl/data/screw.csv";
    string fileway = "/home/uav/lzy_ws/src/offb_posctl/data/OG.csv";
    int r = ReadRowNumber(fileway);
    vector<vector<double>> trajectory(r, vector<double>(3, 0));
    Readdata(fileway,trajectory);



    /// 等待和飞控的连接
    while (ros::ok() && current_state.connected == 0) {
        ros::spinOnce(); //调用回调函数
        ros::Duration(1).sleep();
        ROS_INFO("Not Connected");
    }
    ROS_INFO("Connected!!");

//    angle_receive = quaternion2euler(pose_drone_Imu.orientation.x, pose_drone_Imu.orientation.y, pose_drone_Imu.orientation.z, pose_drone_Imu.orientation.w);
    Yaw_Init = angle_receive.z;

//    ///set initial hover position and pose
    target_atti_thrust_msg.orientation.x = 0;
    target_atti_thrust_msg.orientation.y = 0;
    target_atti_thrust_msg.orientation.z = 0;
    target_atti_thrust_msg.orientation.w = -1;
    target_atti_thrust_msg.thrust = 0.65; //65%的油门 50%与重力平衡即悬停
    ROS_INFO("got initial point ");

    for (int i = 10; ros::ok() && i >0; --i)// let drone take off slightly at begining, but this step seems useless because the drono has not been armed
    {
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        ros::spinOnce();//让回调函数有机会被执行
        rate.sleep();
    }
    ROS_INFO("OUT OF LOOP WAIT");


    ///------------------------------等待解锁飞机--------------------------------//
    while(ros::ok() && current_state.mode != "OFFBOARD")
    {
        plane_expected_position.x = init_pos_x;
        plane_expected_position.y = init_pos_y;
        plane_expected_position.z = init_pos_z;
        pix_controller(0);
        target_atti_thrust_msg.header.stamp = ros::Time::now();
        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_msg.thrust = thrust_target;
        if(thrust_target>0.58)
        {
            thrust_target = 0.58;
        }
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        std::cout << "thrust_target:  " << thrust_target << std::endl;
        angle_receive = quaternion2euler(pose_drone_Imu.orientation.x, pose_drone_Imu.orientation.y, pose_drone_Imu.orientation.z, pose_drone_Imu.orientation.w);
        std::cout << "Yaw:  " << angle_receive.z << std::endl;
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Not OFFBOARD");
    }
    ///------------------------------------------------------------------------//
    ///------------------------------- hover initial ---------------------------//
    ros::Time begin_time_01 = ros::Time::now();
    got_initial_point = true;
    int count = 0;
    while (ros::ok()&&(count<15*controlfreq||abs(pose_drone_odom.pose.position.x-init_pos_x)>=0.1))
    {
        cout << "------------Hovering----------------" << endl;
        ros::spinOnce();
//        cout << "plane_vel.x:" << vel_drone.twist.linear.x << endl;
        cout << "plane_pos.x:" << pose_drone_odom.pose.position.x << endl;
        float cur_time_01 = get_ros_time(begin_time_01);  // 相对时间
        if (count<10*controlfreq)
        {
            plane_expected_position.x = 0;
            plane_expected_position.y = 0;
            plane_expected_position.z = init_pos_z;
        } else{
            plane_expected_position.x = init_pos_x;
            plane_expected_position.y = init_pos_y;
            plane_expected_position.z = init_pos_z;
        }
        cout<<"plane_expected_position.x"<<plane_expected_position.x<<" plane_expected_position.y"<<plane_expected_position.y<<" plane_expected_position.z"<<plane_expected_position.z<<endl;
        pix_controller(cur_time_01);
        target_atti_thrust_msg.orientation = orientation_target;
        if(thrust_target>0.58)
         {
             thrust_target = 0.58;
        }
        target_atti_thrust_msg.thrust = thrust_target;
        std::cout << "thrust_target:  " << thrust_target << std::endl;
        std::cout << "Yaw_Init:  " << Yaw_Init << std::endl;
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        counter.data = count;
        counter_pub.publish(counter);
        count = count + 1;
		actual_rpy_pub.publish(angle_receive);
        rate.sleep();
    }
    ///------------------------------------------------------------------------//

    ///------------------------------- trajectory follow ---------------------------//
    count = 0;
    begin_time_01 = ros::Time::now();
    while (ros::ok())
    {
        cout << "------------Follow----------------" << endl;
        ros::spinOnce();
        cout << "plane_pos.x:" << pose_drone_odom.pose.position.x << endl;
        if (count<r)
        {
            plane_expected_position.x = trajectory[count][0];
            plane_expected_position.y = trajectory[count][1];
            plane_expected_position.z = trajectory[count][2];
        }
        else if ((count < r+5*controlfreq)&&(count>=r))
        {
            plane_expected_position.x = trajectory[r-1][0];
            plane_expected_position.y = trajectory[r-1][1];
            plane_expected_position.z = trajectory[r-1][2];
        } else
        {
            plane_expected_position.x = trajectory[r-1][0];
            plane_expected_position.y = trajectory[r-1][1];
            plane_expected_position.z = 0.2;
        }
        cout<<"plane_expected_position.x"<<plane_expected_position.x<<" plane_expected_position.y"<<plane_expected_position.y<<" plane_expected_position.z"<<plane_expected_position.z<<endl;

        float cur_time_01 = get_ros_time(begin_time_01);  // 相对时间
        pix_controller(cur_time_01);
        target_atti_thrust_msg.orientation = orientation_target;
        if(thrust_target>0.58)
        {
            thrust_target = 0.58;
        }
        target_atti_thrust_msg.thrust = thrust_target;
        std::cout << "thrust_target:  " << thrust_target << std::endl;
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        counter.data = count;
        counter_pub.publish(counter);
        count = count + 1;
		actual_rpy_pub.publish(angle_receive);
//        cout<<"count"<<count<<endl;
        rate.sleep();
    }
    ///------------------------------------------------------------------------//

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


///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void vector3dLimit(Vector3d &v, double limit)  ///limit should be positive
{
    if(limit > 0){
        for(int i=0; i<3; i++){
            v(i) = fabs(v(i)) > limit ? (v(i) > 0 ? limit : -limit) : v(i);
        }
    }
}

Vector3d vectorElementMultiply(Vector3d v1, Vector3d v2)
{
    Vector3d result;
    result << v1(0)*v2(0), v1(1)*v2(1), v1(2)*v2(2);
    return result;
}


int pix_controller(float cur_time)
{
//位 置 环
    //计算误差
    float error_x = plane_expected_position.x - pose_drone_odom.pose.position.x;
    float error_y = plane_expected_position.y - pose_drone_odom.pose.position.y;
    float error_z = plane_expected_position.z - pose_drone_odom.pose.position.z;
    std::cout << "error: x：" << error_x << "\ty：" << error_y << "\tz：" << error_z << std::endl;
    //计算指定速度误差
    float vel_xd = param.x_p * error_x;
    float vel_yd = param.y_p * error_y;
    float vel_zd = param.z_p * error_z;
    vel_target.x = vel_xd;
    vel_target.y = vel_yd;
    vel_target.z = vel_zd;

//速 度 环
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDVX.start_intergrate_flag = true;
    PIDVY.start_intergrate_flag = true;
    PIDVZ.start_intergrate_flag = true;
    if(got_initial_point == false){
        PIDVX.start_intergrate_flag = false;
        PIDVY.start_intergrate_flag = false;
        PIDVZ.start_intergrate_flag = false;
    }
    //计算误差
    float error_vx = vel_xd - vel_drone.twist.linear.x;
    float error_vy = vel_yd - vel_drone.twist.linear.y;
    float error_vz = vel_zd - vel_drone.twist.linear.z;
    //传递误差
    PIDVX.add_error(error_vx, cur_time); //把error放到list中
    PIDVY.add_error(error_vy, cur_time);
    PIDVZ.add_error(error_vz, cur_time);
    //计算输出
    PIDVX.pid_output();
    PIDVY.pid_output();
    PIDVZ.pid_output();

//    Matrix2f A_yaw;
//    A_yaw << sin(Yaw_Locked), cos(Yaw_Locked),
//            -cos(Yaw_Locked), sin(Yaw_Locked);
//    Vector2f mat_temp(PIDVX.Output,PIDVY.Output);       //赋值到期望推力和姿态 x是前后，y是左右
//    Vector2f euler_temp= 1/9.8 * A_yaw.inverse() * mat_temp;
//    angle_target.x = euler_temp[0];
//    angle_target.y = euler_temp[1];
//    std::cout << " PIDVX.pid_output(): " << PIDVX.Output << "\tangle_target.y: " << angle_target.y << std::endl;
////    angle_target.z = Yaw_Locked + Yaw_Init;
//    angle_target.z = Yaw_Init;

    angle_target.x = asin(-PIDVY.Output/sqrt(pow(PIDVX.Output,2)+pow(PIDVY.Output,2)+pow(PIDVZ.Output+9.8,2)));
    angle_target.y = atan(PIDVX.Output/(PIDVZ.Output+9.8));
    angle_target.z = Yaw_Init;

    orientation_target = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);
    thrust_target  = (float)sqrt(pow(PIDVX.Output,2)+pow(PIDVY.Output,2)+pow(PIDVZ.Output+9.8,2))/9.8*(0.45);   //目标推力值

//    std::cout << "PIDVZ.OUTPUT:  " << PIDVZ.Output << std::endl;
//    std::cout << "thrust_target:  " << thrust_target << std::endl;

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
    temp.y = asin(2.0 * (w * y - z * x));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

/**
 * 将进入offboard后的位置&速度&姿态信息记录进文件
 * @param cur_time
 */

void data_log(std::ofstream &logfile, float cur_time, float ros_time)
{
    logfile<<cur_time<<endl;

}
/**
 * 获得csv文件的行数，返回r
 */
int ReadRowNumber(string fileway)
{
    ifstream fin_u1;// the instance can be used to read the file
    string line; //used for store the one line data from csv temporarily
    int r = 0;
    fin_u1.open(fileway);// the directory of th data.csv
    while (!fin_u1.eof()) {// juddge whether the fin reach the end line of the csv file
        fin_u1 >> line;// read one line data to "line"
        fin_u1 >> line;// read one line data to "line"
        fin_u1 >> line;// read one line data to "line"
        r++;
    }
    fin_u1.close();
    return r;
}
/**
 * 读取csv文件为vector类型
 */
void Readdata(string fileway,vector<vector<double>> &ans)
{
    ifstream fin_u1;// the instance can be used to read the file
    string line; //used for store the one line data from csv temporarily
    int index = 0;
    fin_u1.open(fileway);// the directory of th data.csv
    while (!fin_u1.eof())
    {// juddge whether the fin reach the end line of the csv file
        fin_u1 >> line;// read one line data to "line"
        ans[index][0]= atof(line.c_str());
        fin_u1 >> line;
        ans[index][1]=atof(line.c_str());
        fin_u1 >> line;
        ans[index][2]=atof(line.c_str());
        cout<<ans[index][2]<<endl;
        index++;
    }
    fin_u1.close();
}
