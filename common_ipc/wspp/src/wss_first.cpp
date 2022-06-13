#include"wss_ease.h"

using namespace std;

WSS_C::websocket_endpoint teleop_object;
std::string comman;

int a = 0;
void retryCB(const ros::TimerEvent&)
{
	a = WSS_C::websocket_endpoint::I;
	std::cout<<"a from callback : "<<a<<std::endl;
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"wss_teleop");
    ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Timer ret =n.createTimer(ros::Duration (0.5),retryCB);
    while(ros::ok())
	{
        ros::spinOnce();
		std::cout<<WSS_C::websocket_endpoint::DAT<<std::endl;
		std::cout<<WSS_C::websocket_endpoint::N_com<<" I is : "<< WSS_C::websocket_endpoint::I<<std::endl;
		if(WSS_C::websocket_endpoint::N_com==1)//use this in teleop
		{
			comman=WSS_C::websocket_endpoint::DAT;
			WSS_C::websocket_endpoint::N_com=0;
			std::cout<<"here temp"<<std::endl;
		}
		// std::cout<<N_com<<" I is : "<< I<<std::endl;
		// if(N_com==1)//use this in teleop
		// {/
		// 	comman=DAT;
		// 	N_com=0;
		// 	std::cout<<"here temp"<<std::endl;
		// }

		if(comman == "left")
		{
			cout << comman << endl; // test
			ROS_INFO("Left key Pressed");
			std::stringstream SS;
			// SS<<"LEFT";
			// STR.data = SS.str();
			// mov_cmd.publish(STR);
			comman="NULL";
		}

		else if(comman == "forward")
		{
			cout << comman << endl; // test
			ROS_INFO("Forward Key Pressed");
			std::stringstream SS;
			SS<<"FORWARD";
			// STR.data = SS.str();
			// mov_cmd.publish(STR);
			comman="NULL";
		}
		else if(comman == "right")
		{
			cout << comman << endl; // test
			ROS_INFO("Right Key Pressed");
			std::stringstream SS;
			// SS<<"RIGHT";
			// STR.data = SS.str();
			// mov_cmd.publish(STR);
			comman="NULL";
		}
		else if(comman == "backward")
		{
			cout << comman << endl; // test
			ROS_INFO("BackWard Key Pressed");
			std::stringstream SS;
			// SS<<"BACKWARD";
			// STR.data = SS.str();
			// mov_cmd.publish(STR);
			comman="NULL";
		}

		else if(comman == "start_lsf"){
			cout << comman << endl; // test
			ROS_INFO("start_lsf key pressed");
			// std::stringstream SS;
			// SS<<"START_LSF";
			// STR.data = SS.str();
			// start_sf.publish(STR);
			comman="NULL";
		}

		else if(comman == "start_rsf"){
			cout << comman << endl; // test
			ROS_INFO("start_rsf key pressed");
			// std::stringstream SS;
			// SS<<"START_RSF";
			// STR.data = SS.str();
			// start_sf.publish(STR);
			comman="NULL";
		}

		else if(comman == "start_lwf"){
			cout << comman << endl; // test
			ROS_INFO("start_lwf key pressed");
			// std::stringstream SS;
			// SS<<"START_LWF";
			// STR.data = SS.str();
			// start_wf.publish(STR);
			comman="NULL";
		}

		else if(comman == "start_rwf"){
			cout << comman << endl; // test
			ROS_INFO("start_rwf key pressed");
			// std::stringstream SS;
			// SS<<"START_RWF";
			// STR.data = SS.str();
			// start_wf.publish(STR);
			comman="NULL";
		}

		else if(comman == "start_nav_l")
		{
			ROS_INFO("Navigate ZZ L");
			// std::stringstream SS;
			// SS<<"START_L";
			// STR.data = SS.str();
			// nav_zz.publish(STR);
			comman="NULL";
		}

		else if(comman == "start_nav_r")
		{
			ROS_INFO("Navigate ZZ R");
			// std::stringstream SS;
			// SS<<"START_R";
			// STR.data = SS.str();
			// nav_zz.publish(STR);
			comman="NULL";
		}

		else if (comman == "stop" || comman=="stopm")
		{
			ROS_INFO("Stop key pressed");
			// std::stringstream SS;
			// SS<<"STOP";
			// STR.data = SS.str();
			// STOP.publish(STR);
			comman="NULL";
		}
		else if(comman == "pause")
		{
			ROS_INFO("Pause key pressed");
			// std::stringstream SS;
			// SS<<"PAUSE";
			// STR.data = SS.str();
			// PAUSE.publish(STR);
			comman="NULL";
		}
		else if(comman == "resume")
		{
			ROS_INFO("Resume key pressed");
			// std::stringstream SS;
			// SS<<"RESUME";
			// STR.data = SS.str();
			// RESUME.publish(STR);
			comman="NULL";
		}
		loop_rate.sleep();
    }
    ros::spin();
    return 0;
}