#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nord_messages/NextNode.h"
#include "nord_messages/PoseEstimate.h"
#include <sstream>
#include <string>

double pos_x,pos_y,pos_dir;
double next_x,next_y,old_x,old_y;
int move,old_move;
double pi;
ros::NodeHandle n;
ros::Subscriber position_sub;
ros::Subscriber next_node_sub;
ros::Publisher next_node_pub;
ros::Subscriber bump_sub;
ros::Subscriber reach_sub;


bool flag_bump;

void PositionCallback(const nord_messages::PoseEstimate command){
	pos_x=command.x.mean;
	pos_y=command.y.mean;
	pos_dir=command.theta.mean;
}

void NextNodeCallback(const nord_messages::NextNode command){
	old_x=next_x;
	old_y=next_y;
	old_move=move;
	next_x=command.x;
	next_y=command.y;
	move=command.move;
}

void BumpCallback(const nord_messages::Vector2 command){
	double x,y;
	nord_messages::NextNode msg;
	
	x=y=10;
	
	msg.x=pos_x+cos(pos_dir+pi)*x;
	msg.y=pos_x+cos(pos_dir+pi)*y;
	msg.move=2;
	flag_bump=true;
	next_node_pub.publish(msg);
}

void ReachCallback(const std_msgs::Bool command){/*  IMPORTANT: THIS VALUE OF TRUE MUST NOT BE READ BY THE HIGHER LEVEL NODES!!!!!         */
	nord_messages::NextNode msg;

	if(flag_bump==true && command.data==true){
		msg.x=old_x;
		msg.y=old_y;
		msg.move=old_move;
		flag_bump=false;
		next_node_pub.publish(msg);
		
	}else if(flag_bump==true && command.data==false){
		//SHIT HAPPENED WHAT TO DO?
	}
}

int main(int argc, char **argv){

	/*if(argc<6){
		ROS_INFO("Not enough arguments: p_vel i_vel d_vel p_ang i_ang d_ang");
		return 1;
	}*/
	pi = 3.141592;
	ros::init(argc, argv, "nord_bump_control");
	
	position_sub=n.subscribe("/nord/estimation/pose_estimation",1,&PositionCallback);
	next_node_sub=n.subscribe("/nord/control/point",1,&NextNodeCallback);
	bump_sub=n.subscribe("/imu/bump",1,&BumpCallback);
	next_node_pub=n.advertise<nord_messages::NextNode>("/nord/control/point",1);
	reach_sub= n.subscribe<std_msgs::Bool>("/nord/houston/mission_result", 1,&ReachCallback);
	
	pos_x=pos_y=pos_dir=0;
	next_x=next_y=old_x=old_y=0;
	move=old_move=0;
	
	
	ros::Rate loop_rate(20);
	
	while(ros::ok()){
 
		
		ros::spinOnce();
		
		loop_rate.sleep(); // go to sleep

	}

	return 0;
};