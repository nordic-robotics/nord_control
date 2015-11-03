#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "nord_messages/MotorTwist.h"
#include "nord_messages/RelativePoint.h"//VER MESSAGES!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#include "pid.hpp"
class PointControl
{
	public:

	ros::NodeHandle n;

	ros::Publisher twist_pub;
	ros::Subscriber point_sub;
	
	PointControl(char ** argv){
		point_sub=n.subscribe("",1,&PointControl::PointCallback, this);//NAME OF THE TOPIC!!!!
		twist_pub = n.advertise<nord_messages::MotorTwist>("/motor_controller/twist", 1);
		
		twist.velocity=0;
		twist.angular_vel=0; 

		dist_object=0; 
		dir_object =0;

		/*p_vel = std::stod(argv[1]); 	p_ang = std::stod(argv[4]);
		i_vel	= std::stod(argv[2]); 	i_ang = std::stod(argv[5]);
		d_vel = std::stod(argv[3]);   d_ang = std::stod(argv[6]);*/
		
		p_vel=2.1; i_vel=0.2; d_vel=-0.16;
		p_ang=3; i_ang=0.1; d_ang=-0.05;

		pi = 3.141592;
		dt=1.0/10;
		
		vel_pid =kontroll::pid<double>(p_vel, i_vel, d_vel);
		vel_pid.max =  1.5;
		vel_pid.min = -1.5;
		ang_pid =kontroll::pid<double>(p_ang, i_ang, d_ang);
		ang_pid.max =  45*pi/180;
		ang_pid.min = -45*pi/180;

		/*p_vel=4.1 i_vel=0.2 d_vel=-0.16 
		p_ang=4 i_ang=0.1 d_ang=-0.05*/
		// 4.1 0.2 -0.16 4 0.1 -0.05
/*
		for(x=0;x<1000;x++){//testing
			vec_dist[x]=0.2+x*0.1;
			if(x>500){
				vec_degree[x]=25*pi/(180*(x+1));
			}else{
				vec_degree[x]=-25*pi/(180*(x+1));
			}
		}*/
		// vec_dist={0.35,1,2,3,3.1,0.8,2,0.9,0.7,0.2,10,5,6,7,8,9,4.4,3.5,1.6,0.6};
		// vec_degree={-10,1.1,5.6,9.2,10,-8.3,7,-5.6,-25,25.9,3,15.5,-6,-10.6,-7,-20,6.7,10,-7,3};

		// for (auto& e : vec_degree)
		// {
		// 	e *= pi/180;
		// }
		
		ControlPart();
	}
	
	void  ControlPart(){
		
		twist.velocity= vel_pid(des_dist,dist_object , dt);
   		twist.angular_vel = ang_pid(des_dir, dir_object, dt);

	/*	if(real_vel<0.3 && real_vel>0){
			real_vel=0;
		}else if(real_vel>-0.3 && real_vel<0){
			real_vel=0;
		}*/
		
		
		// est_dist +=twist.linear.x*dt;
		// est_dir +=twist.angular.z*dt;
		
		twist_pub.publish(twist);
	}
	
	void PointCallback(const nord_messages::RelativePoint command){
       /* des_dist= 0.50;
        des_dir= 0;
		dist_object = command.distance;
		dir_object  = command.angle;

		if(dist_object >= 1){
			dist_object = 0.50;
			dir_object 	= 0;
		}*/
	}
	
	void print_info(){
		ROS_INFO("vel: [%f]", twist.velocity);
 		ROS_INFO("ang_vel: [%f]", twist.angular_vel);
 		ROS_INFO("des_dist: [%f]", des_dist);
 		ROS_INFO("est_dist: [%f]", dist_object);
 		ROS_INFO("des_dir: [%f]", des_dir);
		ROS_INFO("est_dir: [%f]", dir_object);
 		ROS_INFO("p_vel: %f i_vel:%f  d_vel:%f ",p_vel,i_vel,d_vel);
 		ROS_INFO("p_ang: %f i_ang:%f  d_ang:%f ",p_ang,i_ang,d_ang);
	}
	
	private:

		nord_messages::MotorTwist twist; 
		
		std::vector<double> vec_dist; std::vector<double> vec_degree;
		int x; 
		
		double pi;
		double dt,dir_object,des_dist,dist_object,des_dir;
		double real_vel;
		double p_vel,i_vel,d_vel; double p_ang,i_ang,d_ang;
		kontroll::pid<double> vel_pid; kontroll::pid<double> ang_pid;

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
		run.ControlPart();
		run.print_info();
		loop_rate.sleep(); // go to sleep

	}

	return 0;
};
