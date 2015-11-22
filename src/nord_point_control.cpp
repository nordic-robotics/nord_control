#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <string>
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "nord_messages/MotorTwist.h"
#include "nord_messages/NextNode.h"
#include "nord_messages/PoseEstimate.h"
#include <math.h>

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
		position_sub=n.subscribe("/nord/estimation/gaussian",1,&PointControl::PositionCallback, this);
		twist_pub = n.advertise<nord_messages::MotorTwist>("/nord/motor_controller/twist", 1);
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
		
		p_vel=0.9; i_vel=0.01; d_vel=-0.005;
		p_ang=1.5; i_ang=0.04; d_ang=-0.01;
		p_ang_mov=3; i_ang_mov=0.5; d_ang_mov=-0.0225;
		
		vel_pid =kontroll::pid<double>(p_vel, i_vel, d_vel);
		vel_pid.max =  0.4;//0.7 its equal to a PWM of approximately 160 and considering the 45º degree start moving forward this is the limit
		vel_pid.min = -0.4;
		ang_pid =kontroll::pid<double>(p_ang, i_ang, d_ang);
		ang_pid.max =  pi;//it can be bigger than 45deg per sec because when its a pure turn the PWM starts at zero, and not with the forward vel
		ang_pid.min = -pi;
		ang_mov_pid=kontroll::pid<double>(p_ang_mov, i_ang_mov, d_ang_mov);
		ang_mov_pid.max= pi;
		ang_mov_pid.min=-pi;
		
		ang_pid.last_error = 0;
        ang_pid.last_input = 0;
        ang_pid.error_sum = 0;
		ang_mov_pid.last_error = 0;
        ang_mov_pid.last_input = 0;
        ang_mov_pid.error_sum = 0;
		
		ang_cont=0;
		last_ang_cont=0;

		/*
		for(x=0;x<1000;x++){//testing
			vec_dist[x]=0.2+x*0.1;
			if(x>500){
				vec_degree[x]=25*pi/(180*(x+1));
			}else{
				vec_degree[x]=-25*pi/(180*(x+1));
			}s
		}*/
		vec_x={0.17,0.17,0.99,0.99,0.17,0.17,0.99,0.99};
		vec_y={2.1,1.04,1.04,2.11,2.1,1.04,1.04,2.11};
		vec_i=0;
		next_x=vec_x[vec_i];
		next_y=vec_y[vec_i];
		move=1;

		// for (auto& e : vec_degree)
		// {
		// 	e *= pi/180;
		// }
		
	}
	
	void  ControlPart(){
		
		twist.velocity= vel_pid(des_dist,dist_point , dt);
   		
		
		if(des_dist!=dist_point){
			ang_cont=2;
			if(ang_cont!=last_ang_cont){
				ROS_INFO("STARTED AGAIN: %f",dir_point);
				ang_mov_pid.last_error = 0;
				ang_mov_pid.last_input = 0;
				ang_mov_pid.error_sum = 0;
			}
			twist.angular_vel = ang_mov_pid(0, dir_point, dt);
		}else{
			ang_cont=1;
			if(ang_cont!=last_ang_cont){
				ROS_INFO("STARTED AGAIN: %f",dir_point);
				ang_pid.last_error = 0;
				ang_pid.last_input = 0;
				ang_pid.error_sum = 0;
			}
			twist.velocity=0;
			twist.angular_vel = ang_pid(0, dir_point, dt);
		}
		last_ang_cont=ang_cont;
	/*	if(des_dist==dist_point){
			twist.velocity=0;
		}
		twist.angular_vel = ang_pid(0, dir_point, dt);*/


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
		dt=duration.toSec();
		old=command.stamp;
		
		double startmove=pi/22;
		double dist_thres=0.025;

		double x1,y1;
		double x,y;

		x1=next_x-pos_x;
		y1=next_y-pos_y;
		x=(cos(-pos_dir)*x1)-(sin(-pos_dir)*y1);
		y=(sin(-pos_dir)*x1)+(cos(-pos_dir)*y1);	
		
		dist_point=sqrt(pow((next_x-pos_x),2.0)+pow((next_y-pos_y),2.0));
		dir_point=atan2(y,x);
		//ROS_INFO("dir_point: [%f] pos_dir:%f", dir_point,pos_dir);
		/*ROS_INFO("dist_point:%f",dist_point);
		ROS_INFO("pos_x: %f pos_y: %f",pos_x,pos_y);
		ROS_INFO("next_x: %f next_y: %f",next_x,next_y);*/
		//ROS_INFO("x: %f y: %f",x,y);

		if(dir_point>pi){
			dir_point-=2*pi;
		}else if(dir_point<-pi){
			dir_point+=2*pi;
		}

		if(dist_point<=dist_thres){
			msg_bool.data=true;
			reach_pub.publish(msg_bool);
			vec_i++;
			if(vec_i>8){
				move=0;
			}
			next_x=vec_x[vec_i];
			next_y=vec_y[vec_i];
		}
		
		if(move==1){
			/*if((dir_point-pos_dir)>pi){
				dir_point-= (2*pi);
			}else if((dir_point-pos_dir)<-pi){
				dir_point+= (2*pi);
			}*/
			if((dir_point-0)>(startmove)||(dir_point-0)<-(startmove)){
				dist_point=0;
			}
		}else if(move==2){
			dist_point=-dist_point;
			
			if(dir_point>0){
				dir_point-=pi;
			}else{
				dir_point+=pi;
			}
			
			/*if((dir_point-pos_dir)>pi){
				dir_point-= (2*pi);
			}else if((dir_point-pos_dir)<-pi){
				dir_point+= (2*pi);
			}*/
			
			if((dir_point-0)>(startmove)||(dir_point-0)<-(startmove)){
				dist_point=0;
			}
		}else{
			dist_point=des_dist;
			dir_point=0;
		}
		//ROS_INFO("dir_point: [%f] pos_dir:%f", dir_point,pos_dir);
		ControlPart();
		print_info();
	}
	
	void print_info(){
		ROS_INFO("vel: [%f]", twist.velocity);
 		ROS_INFO("ang_vel: [%f]", twist.angular_vel);
		ROS_INFO("dir_point: [%f] pos_dir:%f", dir_point,pos_dir);
		ROS_INFO("dist_point:%f",dist_point);
		ROS_INFO("pos_x: %f pos_y: %f",pos_x,pos_y);
		ROS_INFO("next_x: %f next_y: %f\n",next_x,next_y);
 		//ROS_INFO("p_vel: %f i_vel:%f  d_vel:%f ",p_vel,i_vel,d_vel);
 		//ROS_INFO("p_ang: %f i_ang:%f  d_ang:%f ",p_ang,i_ang,d_ang);
	}
	
	private:

		nord_messages::MotorTwist twist; 
		
		std::vector<double> vec_x;
		std::vector<double> vec_y;
		//std::vector<double> vec_degree;
		double next_x,next_y;
		int move;
		double pos_x,pos_y;
		double pos_dir;
		double pi;
		double dir_point,des_dist,dist_point;
		double dt;
		double p_vel,i_vel,d_vel; double p_ang,i_ang,d_ang;
		double p_ang_mov, i_ang_mov, d_ang_mov;
		kontroll::pid<double> vel_pid; kontroll::pid<double> ang_pid; kontroll::pid<double> ang_mov_pid;
		ros::Time old;
		ros::Duration duration;
		std_msgs::Bool msg_bool;
		int ang_cont;
		int last_ang_cont;
		
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
		
		loop_rate.sleep(); // go to sleep

	}

	return 0;
};
