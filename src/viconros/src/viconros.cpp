#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/PoseStamped.h> 
#include <geometry_msgs/TwistStamped.h> 
#include <sstream>
#include <string.h>
#include "CFetchViconData.h"
#include <fstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "viconros");
	ros::NodeHandle n("~");
	std::string ip;
	std::string model;
	std::string segment;


	n.getParam("host",ip);
	n.getParam("model",model);
	n.getParam("segment",segment);
	ROS_INFO("HOST:%s",ip.c_str());
	ROS_INFO("MODEL:%s; SEGMENT:%s",model.c_str(),segment.c_str());

	ros::Publisher vicon_pub = n.advertise<geometry_msgs::PoseStamped> ("/mocap/pose", 10);
	ros::Publisher vicon_pub1 = n.advertise<geometry_msgs::TwistStamped> ("/mocap/vel", 10);


	ros::Rate loop_rate(100);
	int count = 0;
	CFetchViconData * vicon=new CFetchViconData();
	const char * host=ip.c_str();
	ObjStatus objs;
	
	if(!(vicon->IsConnected))
    { 
            ROS_INFO("Connecting to %s",host);
            bool res=vicon->Connect(host);
            if(res==false)
            {
                ROS_INFO("Failed to connect!\r\n");
                    return 0;
            }
            else
            {
                ROS_INFO("Successfully connected!\r\n");
            }

    }
	while (ros::ok()) {
		geometry_msgs::PoseStamped msg;
		geometry_msgs::TwistStamped msg1;

//         objs=vicon->GetStatus(model.c_str(),segment.c_str());
        vicon->GetStatus(objs, model.c_str(),segment.c_str());

//		msg.header.stamp.sec=(int)objs.tm;
//		msg.header.stamp.nsec=(objs.tm-msg.header.stamp.sec)*10000*100000;
        msg.header.stamp=ros::Time::now();
		msg.pose.position.x =objs.pos[0];
		msg.pose.position.y =objs.pos[1];
		msg.pose.position.z =objs.pos[2];

         msg.pose.orientation.x =objs.ort[0];
         msg.pose.orientation.y =objs.ort[1];
         msg.pose.orientation.z =objs.ort[2];
         msg.pose.orientation.w =objs.ort[3];

//        msg.pose.orientation.x =objs.euler[0];
//        msg.pose.orientation.y =objs.euler[1];
//        msg.pose.orientation.z =objs.euler[2];
//        msg.pose.orientation.w =0;

//		msg1.header.stamp.sec=(int)objs.tm;
//		msg1.header.stamp.nsec=(objs.tm-msg.header.stamp.sec)*10000*100000;
        msg1.header.stamp=ros::Time::now();
		msg1.twist.linear.x =objs.vel[0];
		msg1.twist.linear.y =objs.vel[1];
		msg1.twist.linear.z =objs.vel[2];

		std::cout<<"current_POS"<<"   x:"<<objs.pos[0]<<"   y:"<<objs.pos[1]<<"   z:"<<objs.pos[2]<<std::endl;
//        std::cout<<"current_vel"<<"  vx:"<<objs.vel[0]<<"   vy:"<<objs.vel[1]<<"  vz:"<<objs.vel[2]<<std::endl;
//        std::cout<<"current_Euler"<<"   roll:"<<objs.euler[0]<<"   pitch:"<<objs.euler[1]<<"   yaw:"<<objs.euler[2]<<std::endl;
		//std::cout<<objs.pos[0]<<"-"<<objs.pos[1]<<std::endl;
		//ROS_INFO("position:%f-%f-%f; velocity: %f-%f-%f", msg.position.x,msg.position.y,msg.position.z,msg.velocity.x,msg.velocity.y,msg.velocity.z);
		
		
		vicon_pub.publish(msg);
		vicon_pub1.publish(msg1);

		ros::spinOnce();
		loop_rate.sleep();
		//++count;
	}
	return 0;
}
