#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.msg"
#include <sstream>
#include <string>
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "nord_messages/MotorTwist.h"
#include "nord_messages/NextNode.h"
#include "nord_messages/PoseEstimate.h"

#include "pid.hpp"
class PointControl
{
	public:

	ros::NodeHandle n;

	ros::Publisher twist_pub;
	ros::Subscriber next_node_sub;
	ros::Subscriber position_sub;
	ros::Publisher reach_pub;
	
	PointControl(char ** argv){
		next_node_sub=n.subscribe("/nord/control/point",1,&PointControl::NextNodeCallback, this);
		position_sub=n.subscribe("",1,&PointControl::PositionCallback, this);
		twist_pub = n.advertise<nord_messages::MotorTwist>("/motor_controller/twist", 1);
		reach_pub= n.advertise<std_msgs::Bool>("/nord/houston/mission_result", 1);
		
		twist.velocity=0;
		twist.angular_vel=0; 

		dist_point=0; 
		dir_point =0;
		next_x=0;
		next_y=0;
		move=0;
		pos_dir=0;
		pos_x=0;
		pos_y=0;
		pi = 3.141592;
		dt=1.0/10;
		des_dist=0;
		old=ros::Time::now();

		/*p_vel = std::stod(argv[1]); 	p_ang = std::stod(argv[4]);
		i_vel	= std::stod(argv[2]); 	i_ang = std::stod(argv[5]);
		d_vel = std::stod(argv[3]);   d_ang = std::stod(argv[6]);*/
		
		p_vel=2.1; i_vel=0.2; d_vel=-0.16;
		p_ang=3; i_ang=0.1; d_ang=-0.05;
		
		vel_pid =kontroll::pid<float>(p_vel, i_vel, d_vel);
		vel_pid.max =  0.6;//0.7 its equal to a PWM of approximately 160 and considering the 45º degree start moving forward this is the limit
		vel_pid.min = -0.6;
		ang_pid =kontroll::pid<float>(p_ang, i_ang, d_ang);
		ang_pid.max =  pi;//it can be bigger than 45deg per sec because when its a pure turn the PWM starts at zero, and not with the forward vel
		ang_pid.min = -pi;

		/*
		for(x=0;x<1000;x++){//testing
			vec_dist[x]=0.2+x*0.1;
			if(x>500){
				vec_degree[x]=25*pi/(180*(x+1));
			}else{
				vec_degree[x]=-25*pi/(180*(x+1));
			}
		}*/
		vec_x={1,1.6,1.2};
		vec_y={0.23,0.23,0.23};
		vec_i=0;
		move=1;

		// for (auto& e : vec_degree)
		// {
		// 	e *= pi/180;
		// }
		
	}
	
	void  ControlPart(){
		
		twist.velocity= vel_pid(des_dist,dist_point , dt);
   		twist.angular_vel = ang_pid(pos_dir, dir_point, dt);

	/*	if(real_vel<0.3 && real_vel>0){
			real_vel=0;
		}else if(real_vel>-0.3 && real_vel<0){
			real_vel=0;
		}*/
		
		
		// est_dist +=twist.linear.x*dt;
		// est_dir +=twist.angular.z*dt;
		
		twist_pub.publish(twist);
	}
	
	void NextNodeCallback(const nord_messages::NextNode command){
		next_x=command.x;
		next_y=command.y;
		move=command.move;
	}
	
	void PositionCallback(const nord_messages::PoseEstimate command){
		pos_x=command.x.mean;
		pos_y=command.y.mean;
		pos_dir=command.theta.mean;
		duration=command.stamp-old;
		dt=duration.toSec;
		old=command.stamp;
		
		float startmove=pi/12;
		float dist_thres=0.05;
		
		
		dist_point=sqrt(pow(float(next_x-pos_x),2.0)+pow(float(next_y-pos_y),2.0));
		dir_point=atan2(float(next_y-pos_y),float(next_x-pos_x));
		
		if(dist_point<dist_thres){
			msg_bool.data=true;
			reach_pub.publish(msg_bool);
			vec_i++;
			next_x=vec_x[vec_i];
			next_y=vec_y[vec_i];
		}
		
		if(move==1){
			if((dir_point-pos_dir)>pi){
				dir_point-= (2*pi);
			}else if((dir_point-pos_dir)>-pi){
				dir_point+= (2*pi);
			}
			if((dir_point-pos_dir)>(startmove)||(dir_point-pos_dir)<-(startmove)){
				dist_point=0;
			}
		}else if(move==2){
			dist_point=-dist_point;
			
			if(dir_point>0){
				dir_point-=pi;
			}else{
				dir_point+=pi;
			}
			
			if((dir_point-pos_dir)>pi){
				dir_point-= (2*pi);
			}else if((dir_point-pos_dir)>-pi){
				dir_point+= (2*pi);
			}
			
			if((dir_point-pos_dir)>(startmove)||(dir_point-pos_dir)<-(startmove)){
				dist_point=0;
			}
		}else{
			dist_point=0;
			dir_point=pos_dir;
		}
		
		ControlPart();
	}
	
	void print_info(){
		ROS_INFO("vel: [%f]", twist.velocity);
 		ROS_INFO("ang_vel: [%f]", twist.angular_vel);
 		ROS_INFO("des_dist: [%f]", des_dist);
 		ROS_INFO("est_dist: [%f]", dist_point);
 		ROS_INFO("des_dir: [%f]", pos_dir);
		ROS_INFO("est_dir: [%f]", dir_point);
 		ROS_INFO("p_vel: %f i_vel:%f  d_vel:%f ",p_vel,i_vel,d_vel);
 		ROS_INFO("p_ang: %f i_ang:%f  d_ang:%f ",p_ang,i_ang,d_ang);
	}
	
	private:

		nord_messages::MotorTwist twist; 
		
		std::vector<int> vec_x;
		std::vector<int> vec_y;
		//std::vector<double> vec_degree;
		int next_x,next_y,move;
		int pos_x,pos_y;
		float pos_dir;
		float pi;
		float dir_point,des_dist,dist_point;
		double dt;
		float p_vel,i_vel,d_vel; float p_ang,i_ang,d_ang;
		kontroll::pid<float> vel_pid; kontroll::pid<float> ang_pid;
		ros::Time old;
		ros::Duration duration;
		std_msgs::bool msg_bool;
		
		int vec_i;//testing variable dlete after

};
int main(int argc, char **argv){

	/*if(argc<6){
		ROS_INFO("Not enough arguments: p_vel i_vel d_vel p_ang i_ang d_ang");
		return 1;
	}*/
	
	ros::init(argc, argv, "nord_twist_publisher");
	
	PointControl run(argv); 
	ros::Rate loop_rate(10);
	
	while(ros::ok()){
 
		
		ros::spinOnce();
		run.print_info();
		loop_rate.sleep(); // go to sleep

	}

	return 0;
};
