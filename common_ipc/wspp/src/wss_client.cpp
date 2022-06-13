#include<ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <ros/console.h>

#include <vector> 
#include <string> 
#include <algorithm> 
#include <sstream> 
#include <iterator> 

#include <iostream>
#include<fstream>
#include <string>

#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>

#include<websocketpp/common/thread.hpp>
#include<websocketpp/common/memory.hpp>
#include <boost/chrono.hpp>
#include<vector>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>
#include <jsoncpp/json/value.h>

using namespace std;
typedef websocketpp::client<websocketpp::config::asio_tls_client> client;
typedef websocketpp::lib::shared_ptr<boost::asio::ssl::context> context_ptr;

int I_id[50],id_it=0;
int I, second = 0, done_here = 0;
vector<float> above_, slant_;

int Active=0,ping_ok=1,init=0,Retry=0,N_com=0, once = 0, send_ping = 1,PING_SENT=0,PING_COUNT=0;	//Active=0 to enable ping
std::string _stat,DAT,x;
std::string uri="wss://central.brsindia.com";
//std::string uri="ws://192.168.170.234:8080";

ros::Timer enc_timer,imu_timer,acc_timer,pose_timer,wifi_timer,ping_timer,recon_timer,ping_timer_check, both_lid_timer;

std::string left_enc_call_message,right_enc_call_message;
std::string RID="5f297ce02b2e2b05d5a7dd78";
//std::string AUTH = "{\"messagetype\":\"robot\",\"data\":\""+RID+"\"}";
std::string AUTH = "";
std::string ping_message="";

std::string enc_message,imu_message,acc_message,motor_left_message;
std::string bothlidar_message,motor_right_message,robspeed_message,pose_message,message9,wifi_message;
std::string PING_MSG;
std::string poseX,poseY,position;
int cache=0;

Json::Value RECV;
Json::Reader reader;
std::string command;

class connection_metadata
{
	public:
		typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

		connection_metadata(int id,websocketpp::connection_hdl hdl,std::string uri)
		: m_id(id)
		, m_hdl(hdl)
		, m_status("connecting")
		, m_uri(uri)
		, m_server("N/A")
		{}
	
		void on_open(client *c, websocketpp::connection_hdl hdl)
		{
			m_status="Open";
			_stat=m_status;
			client::connection_ptr con=c->get_con_from_hdl(hdl);
			m_server=con->get_response_header("Server");
		}
	
		void on_fail(client *c, websocketpp::connection_hdl hdl)
		{
			m_status="Failed";
			_stat=m_status;
		
			client::connection_ptr con=c->get_con_from_hdl(hdl);
			m_server=con->get_response_header("Server");
			m_error_reason=con->get_ec().message();
		}
	
		void on_close(client *c, websocketpp::connection_hdl hdl)
		{
			m_status="Closed";
			_stat=m_status;
			client::connection_ptr con=c->get_con_from_hdl(hdl);
			std::stringstream s;
			s<<"close code: "<< con->get_remote_close_code()<<" ("
			 << websocketpp::close::status::get_string(con->get_remote_close_code())
			 << "), close reason: " <<con->get_remote_close_reason();
			m_error_reason=s.str();
		}

		void on_message(websocketpp::connection_hdl hdl,client::message_ptr msg)
		{
			if(msg->get_opcode()==websocketpp::frame::opcode::text)
			{
				m_message=msg->get_payload();
				bool parseSuccessful = reader.parse(m_message, RECV);
				if ( !parseSuccessful)
				{
					DAT="NULL";
				}
				Json::Value w = RECV;
								
				if (w["message"].isString() and ((w["roboid"].asString() == RID) || (w["robid"].asString() == RID)))
				{
					DAT=w["message"].asString();
				}


				if(DAT==ping_message)
				{
					ping_ok=1;
				}
				else
				{
					N_com=1;
					//std::cout<<"Message:- "<<m_message<<"\n";
				}
//				std::cout<<"Message:- "<<w["message"].asString()<<"\n";
			}
		}
		websocketpp::connection_hdl get_hdl()
		{
			return m_hdl;
		}
		
		std::string get_status()
		{
			return m_status;
		}
		
		int get_id()
		{
			return m_id;
		}
	private:
		int m_id;
		websocketpp::connection_hdl m_hdl;
		std::string m_status;
		std::string m_uri;
		std::string m_server;
		std::string m_error_reason;
		std::string m_message;
};

class websocket_endpoint
{
	public:
	typedef websocket_endpoint type;
	websocket_endpoint() : m_next_id(0)
	{
		m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
		m_endpoint.clear_error_channels(websocketpp::log::elevel::all);

		m_endpoint.init_asio();
		m_endpoint.start_perpetual();
		m_endpoint.set_socket_init_handler(bind(&type::on_socket_init,this,::_1));
		m_endpoint.set_tls_init_handler(bind(&type::on_tls_init,this,::_1));
		m_thread.reset(new websocketpp::lib::thread(&client::run,&m_endpoint));
	}
	
	~websocket_endpoint()
	{
		m_endpoint.stop_perpetual();
		for(con_list::const_iterator it = m_connection_list.begin(); it!=m_connection_list.end(); ++it)
		{
			if(it->second->get_status() != "Open")
			{
				continue;
			}
			ROS_INFO("Websocket : > Closing connection : %d", it->second->get_id());

			websocketpp::lib::error_code ec;
			m_endpoint.close(it->second->get_hdl(), websocketpp::close::status::normal, "", ec);
			if(ec)
			{
				ROS_ERROR("Websocket : > Error closing conection %d : %s", it->second->get_id(), ec.message().c_str());
			}
		}
		m_thread->join();
	}
	void on_socket_init(websocketpp::connection_hdl hdl)
	{
		m_socket_init = std::chrono::high_resolution_clock::now();
	}
	context_ptr on_tls_init(websocketpp::connection_hdl)
	{
		context_ptr ctx(new boost::asio::ssl::context(boost::asio::ssl::context::tlsv12));
		try
		{
			ctx->set_options(boost::asio::ssl::context::default_workarounds |
							boost::asio::ssl::context::no_sslv2 |
							boost::asio::ssl::context::no_sslv3 |
							boost::asio::ssl::context::single_dh_use);
		}
		catch (std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
		return ctx;
	}
	int connect(std::string const & uri)
	{
		websocketpp::lib::error_code ec;
		client::connection_ptr con=m_endpoint.get_connection(uri,ec);
		if(ec)
		{
			ROS_ERROR("Websocket : > Connect initialization error: %s", ec.message().c_str());
			return -1;
		}
	
		int new_id=m_next_id++;
		connection_metadata::ptr metadata_ptr(new connection_metadata(new_id, con->get_handle(),uri));
		m_connection_list[new_id]=metadata_ptr;

		//ROS_INFO("meta : %s", metadata_ptr.c_str());

		con->set_open_handler(websocketpp::lib::bind(
		&connection_metadata::on_open,
		metadata_ptr,&m_endpoint,
		websocketpp::lib::placeholders::_1
		));
	

		con->set_fail_handler(websocketpp::lib::bind(
		&connection_metadata::on_fail,
		metadata_ptr,
		&m_endpoint,
		websocketpp::lib::placeholders::_1
		));

		con->set_message_handler(websocketpp::lib::bind(
		&connection_metadata::on_message,
		metadata_ptr,
		websocketpp::lib::placeholders::_1,
		websocketpp::lib::placeholders::_2
		));

		m_endpoint.connect(con);
		return new_id;
	}
	websocketpp::connection_hdl _find(int id)
	{
		con_list::iterator metadata_it = m_connection_list.find(id);
		websocketpp::connection_hdl hdl = metadata_it->second->get_hdl();
		return hdl;
	}
	void close(int id, websocketpp::close::status::value code,std::string&)
	{
		websocketpp::lib::error_code ec;
		
		con_list::iterator metadata_it=m_connection_list.find(id);
		if(metadata_it==m_connection_list.end())
		{
			ROS_ERROR("Websocket : > No connection found with id : %d", id);
			return;
		}

		m_endpoint.close(metadata_it->second->get_hdl(),code,"",ec);
		if(ec)
		{
			ROS_ERROR("Websocket : > Error initiating close: %s", ec.message().c_str());
		}
	}
	void send(int id, std::string message)
	{
		websocketpp::lib::error_code ec;
		
		con_list::iterator metadata_it=m_connection_list.find(id);
		if(metadata_it==m_connection_list.end())
		{
			ROS_ERROR("Websocket : > No connection found with id : %d", id);
			return;
		}
		m_endpoint.send(metadata_it->second->get_hdl(), message, websocketpp::frame::opcode::text, ec);
		if(ec)
		{
			return;
		}
	}
	connection_metadata::ptr get_metadata(int id) const
	{
		con_list::const_iterator metadata_it=m_connection_list.find(id);
		
		if (metadata_it == m_connection_list.end())
		{
			return connection_metadata::ptr();
		}
		
		else
		{
			return metadata_it->second;
		}
	}
	private:
		typedef std::map<int,connection_metadata::ptr> con_list;
		client m_endpoint;
		websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
		con_list m_connection_list;
		int m_next_id;
		std::chrono::high_resolution_clock::time_point m_tls_init, m_socket_init, m_start;
};

websocket_endpoint endpoint;


//Routine for reconnecting with the server after connection is broken
void retryCB(const ros::TimerEvent&)
{
	if(Retry==1 and Active!=1)
	{
		
		if(_stat=="Open")
		{
			endpoint.send(I,AUTH);
			Active=1;
			ping_ok=1;
			init=1;
			Retry=0;
			ROS_INFO("Websocket : Successfully reconnected to internet");
			I_id[id_it]=I;
			std::cout<<"IDs: "<<I_id[id_it]<<" it: "<<id_it<<"\n";
			id_it++;
			for(int si=0;si<(id_it-1);si++)
			{
				std::string reason= "close";
				endpoint.close(I_id[si],websocketpp::close::status::normal,reason);
			}
		}
		else
		{
			// std::string reason= "close";
			// endpoint.close(I,websocketpp::close::status::normal,reason);
			ROS_INFO("Websocket : Retry connection");
			int id = endpoint.connect(uri);
			I=id;
		}
		//ROS_INFO("Websocket : I - " %d", I);
	}
}

//Routine to send ping & check if connection with server is active or not at fixed interval
void pingCB(const ros::TimerEvent&)
{
	if(init==1)
	{
//		if(send_ping==1 && PING_SENT==0)
		if(send_ping==1)
		{
			PING_MSG="{\"messagetype\":\"Ping\",\"message\":\""+ping_message+"\",\"roboid\":\""+RID+"\"}";
//			PING_MSG="{\"messagetype\":\"Ping\",\"message\":\""+ping_message+"\",\"robid\":\""+RID+"\"}";
			endpoint.send(I,PING_MSG);
			PING_SENT=1;
//			ROS_INFO("%s",PING_MSG.c_str());
		}
	}
}

void pingcheck(const ros::TimerEvent&)
{
	if(init==1)
	{
		if(PING_SENT==1 && PING_COUNT<6 && ping_ok!=1)
		{
			PING_COUNT++;
		}
		else if(ping_ok==1 && PING_SENT==1)
		{
			send_ping = 1;
			ping_ok = 0;	//uncomment to enable ping
			Active = 1;		//uncomment to enable ping
			PING_SENT=0;
			PING_COUNT=0;
		}
		else if(ping_ok==0 && PING_COUNT>=6)
		{
			ROS_ERROR("Websocket : Failed ping");
			Active=0;
			init=0;
			_stat="Failed";
			Retry=1;
			send_ping = 0;
			PING_COUNT=0;
		}

/*
		if(ping_ok==1)
		{
			send_ping = 1;
//			ping_ok = 0;	//uncomment to enable ping
			Active = 1;		//uncomment to enable ping
//			PING_SENT=0;
		}
		else
		{
			ROS_ERROR("Websocket : Failed ping");
			Active=0;
			init=0;
			_stat="Failed";
			Retry=1;
			send_ping = 0;
		}
*/
	}
}

//Routine to send encoder data to server at fixed interval
void enc_s(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+enc_message+"\"}";

	if(init==1 and Active==1 and enc_message!="NULL")
	{
		endpoint.send(I,message);
		enc_message="NULL";
	}
	else
	{
		if(enc_message!="NULL")
		{
			enc_message="NULL";
		}
		cache=1;
	}
}

//Routine to send IMU roll, pitch & yaw data to server at fixed interval
void imu_s(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+imu_message+"\"}";
	if(init==1 and Active==1 and imu_message!="NULL")
	{
//		endpoint.send(I,message);
		imu_message="NULL";
	}
	else
	{
		if(imu_message!="NULL")
		{
			imu_message="NULL";
		}
		cache=1;
	}
}

//Routine to send Acceleration in X, Y & Z direction to server at fixed interval
void acc_s(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+acc_message+"\"}";
	if(init==1 and Active==1 and acc_message!="NULL")
	{
//		endpoint.send(I,message);
		acc_message="NULL";
	}
	else
	{
		if(acc_message!="NULL")
		{
			acc_message="NULL";
		}
		cache=1;
	}
}

void pose_s(const ros::TimerEvent&)
{

	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+pose_message+"\"}";

	if(init==1 and Active==1 and pose_message!="NULL")
	{
		endpoint.send(I,message);
		pose_message="NULL";
	}
	else
	{
		if(pose_message!="NULL")
		{
			pose_message="NULL";
		}
		cache=1;
	}
}

void bothlidar_s(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"bothlid\",\"message\":\""+bothlidar_message+"\",\"roboid\":\""+RID+"\"}";

	if(init==1 and Active==1 and bothlidar_message!="NULL")
	{
		//ROS_INFO(" here : %s", message.c_str());
		//ROS_INFO(" message : %s", message);
		endpoint.send(I,message);
		bothlidar_message="NULL";
	}
	else
	{
		if(bothlidar_message!="NULL")
		{
			bothlidar_message="NULL";
		}
		cache=1;
	}
}

void wifi_s(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+wifi_message+"\"}";

	if(init==1 and Active==1 and wifi_message!="NULL")
	{
		endpoint.send(I,message);
		wifi_message="NULL";
	}
	else
	{
		if(wifi_message!="NULL")
		{
			wifi_message="NULL";
		}
		cache=1;
	}
}

//Left Encoder callback
void lenccb(const std_msgs::Int32::ConstPtr& a)
{
	left_enc_call_message = boost::lexical_cast<std::string>(a->data);
	left_enc_call_message="10";
	enc_message="|robdata|enc|"+left_enc_call_message+"|"+right_enc_call_message;
}

//Right Encoder callback
void renccb(const std_msgs::Int32::ConstPtr& a)
{
	right_enc_call_message = boost::lexical_cast<std::string>(a->data);
	right_enc_call_message="20";
	enc_message="|robdata|enc|"+left_enc_call_message+"|"+right_enc_call_message;
}

//IMU roll, pitch & yaw callback
void imucb(const geometry_msgs::Vector3::ConstPtr& a)
{
	std::string roll,pitch,yaw;
	roll = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
	pitch = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
	yaw = boost::lexical_cast<std::string>((int)((ceil(a->z*100.0))/100.0));
	imu_message="|robdata|imu|"+roll+"|"+pitch+"|"+yaw;
}

//Acceleration in X, Y & Z direction callback
void accelcb(const geometry_msgs::Vector3::ConstPtr& a)
{
	std::string X,Y,Z;
	X = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
	Y = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
	Z = boost::lexical_cast<std::string>((int)((ceil(a->z*100.0))/100.0));
	acc_message="|robdata|acc|"+X+"|"+Y+"|"+Z;
}
void syscb(const geometry_msgs::Vector3::ConstPtr& a)
{
	
}
void wificb(const geometry_msgs::Vector3::ConstPtr& a)
{
	std::string wi_spd,wi_sth;
	wi_spd = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
	wi_sth = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
}
// void spdcb(const geometry_msgs::Vector3::ConstPtr& a)
// {
// 	std::string SPD,R_SPD,L_SPD;
// 	SPD = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
// 	R_SPD = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
// 	L_SPD = boost::lexical_cast<std::string>((int)((ceil(a->z*100.0))/100.0));
// 	robspeed_message="|robdata|botspeed|"+SPD+"|"+R_SPD+"|"+L_SPD;
// }
// void mo_lcb(const geometry_msgs::Vector3::ConstPtr& a)
// {
// 	std::string V_l,A_l;
// 	V_l = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
// 	A_l = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
// 	motor_left_message="|robdata|motdrivL|"+V_l+"|"+A_l;
// }
// void mo_rcb(const geometry_msgs::Vector3::ConstPtr& a)
// {
// 	std::string V_r,A_r;
// 	V_r = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
// 	A_r = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
// 	motor_right_message="|robdata|motdrivR|"+V_r+"|"+A_r;
// }
void amclposecb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

  	double Y =msg->pose.pose.position.x *10.0; // Rviz X psotition publishing in meters
  	double X = msg->pose.pose.position.y *10.0; // Rviz Y psotition

	poseX = boost::lexical_cast<std::string>(X);
	poseY = boost::lexical_cast<std::string>(Y);


	pose_message="|xy|"+poseX+"|"+poseY+"|d";//d is constant robot direction 
	//position = msg->data.c_str();
	ROS_DEBUG("Websocket : Pose message - %s",pose_message.c_str());

}

void bothlidarcb(const std_msgs::String::ConstPtr& msg)
{
	bothlidar_message = msg->data;
	//ROS_INFO("from callback message : %s", bothlidar_message.c_str());
}

void wifi_strength(const std_msgs::StringConstPtr& str)
{
	wifi_message=str->data;//d is constant robot direction 
	wifi_message="1|"+wifi_message;
	//position = msg->data.c_str();
	ROS_DEBUG("Websocket : Wifi strength - %s",wifi_message.c_str());

	wifi_message="|robdata|wifi|"+wifi_message;
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"simple_ws_def");
	ros::NodeHandle n;

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

	ros::Subscriber LENC = n.subscribe<std_msgs::Int32>("/left_enc",10,lenccb);
	ros::Subscriber RENC = n.subscribe<std_msgs::Int32>("/right_enc",10,renccb);
	ros::Subscriber IMU = n.subscribe<geometry_msgs::Vector3>("/brick_imu",200,imucb);
	ros::Subscriber ACC = n.subscribe<geometry_msgs::Vector3>("/accel_imu",200,accelcb);
	ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 100, amclposecb);
	ros::Subscriber sub_sstrength = n.subscribe("signal_strength", 100, wifi_strength);

	ros::Subscriber sub_lidar = n.subscribe("/both_lidar", 1, bothlidarcb);

	ros::Publisher mov_cmd = n.advertise<std_msgs::Int8>("/wss_teleop", 50);	//CMD_VEL
	ros::Publisher mov_cmd2 = n.advertise<std_msgs::Int8>("/wss_alarm", 50);	 

	std::stringstream ss;
	std_msgs::Int8 teleop_command;
	std_msgs::Int8 teleop_command2;

	ros::Rate loop_rate(50); // Higher value results in faster update rates
	
	enc_timer=n.createTimer(ros::Duration (0.50),enc_s);
	imu_timer=n.createTimer(ros::Duration (1.00),imu_s);
	acc_timer=n.createTimer(ros::Duration (1.00),acc_s);
	pose_timer=n.createTimer(ros::Duration (1.00),pose_s);
	wifi_timer=n.createTimer(ros::Duration (0.50),wifi_s);
	ping_timer=n.createTimer(ros::Duration (1.0),pingCB);
	ping_timer_check=n.createTimer(ros::Duration (0.5),pingcheck);
	recon_timer=n.createTimer(ros::Duration (4.00),retryCB);
	both_lid_timer=n.createTimer(ros::Duration (0.25),bothlidar_s);

	ros::param::get("/RID",RID);
	AUTH = "{\"messagetype\":\"robot\",\"data\":\""+RID+"\"}";
	ping_message=RID+"|ping";

	// ROS_INFO(" Websocket : RID is : %s", RID.c_str());
	// ROS_INFO(" Websocket : Auth is : %s", AUTH.c_str());
	// ROS_INFO(" Websocket : ping message is : %s", ping_message.c_str());
	bool done = false;

	//Initial connection with server
	int id = endpoint.connect(uri);
	I=id;
	I_id[id_it]=I;
	id_it++;

	//Authentication message for client
	endpoint.send(I,AUTH);
	
	init=1;
	
	while(ros::ok())
	{
		if(N_com==1)//use this in teleop
		{
			command=DAT;
			N_com=0;
		}

		if(Retry == 1)
		{
			if(once%15 == 0)
			{
				teleop_command.data = 50;
				mov_cmd.publish(teleop_command);
				//std::cout<<"cmd 1 sent:- "<<teleop_command<<"\n";
				teleop_command2.data = 96;
				mov_cmd2.publish(teleop_command2);
				//std::cout<<"cmd 2 sent:- "<<teleop_command2<<"\n";
				command="NULL";
			}
			//std::cout<<"Retry 1 :- "<<once<<"\n";
			once++;
		}
		else if(Retry == 0 and once > 0)
		{
			teleop_command2.data = 97;
			mov_cmd2.publish(teleop_command2);
			//std::cout<<"cmd 3 sent:- "<<teleop_command2<<"\n";
			once = 0;
		}
		if(command == "left")
		{
			teleop_command.data = 40;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}

		else if(command == "forward")
		{
			teleop_command.data = 80;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "right")
		{
			teleop_command.data = 60;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "backward")
		{
			teleop_command.data = 20;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}

		else if (command == "stop" || command=="stopm")
		{
			teleop_command.data = 50;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "pause")
		{
			teleop_command.data = 50;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		 else if(command == "alarmon") 
        {
			teleop_command.data = 96; //Alarm On
			mov_cmd2.publish(teleop_command2);
			command="NULL";
        }
        else if(command == "alarmoff")
        {
			teleop_command.data = 97; //Alarm Off
			mov_cmd2.publish(teleop_command2);
			command="NULL";
        }
		else if(command == "keypop")
		{
			teleop_command.data = 98;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "keyPOP")
		{
			teleop_command.data = 99;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "ssenb")
		{
			teleop_command.data = 15;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "ssdis")
		{
			teleop_command.data = 16;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "obs") //enable
		{
			teleop_command.data = 11;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "OBS") //disable
		{
			teleop_command.data = 12;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "sfl1")
		{
			teleop_command.data = 41;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "sfl2")
		{
			teleop_command.data = 42;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "sfl3")
		{
			teleop_command.data = 43;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "sfl4")
		{
			teleop_command.data = 44;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "sfr1")
		{
			teleop_command.data = 46;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "sfr2")
		{
			teleop_command.data = 47;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "sfr3")
		{
			teleop_command.data = 48;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "sfr4")
		{
			teleop_command.data = 49;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "swfenb")
		{
			teleop_command.data = 13;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "swfdis")
		{
			teleop_command.data = 14;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl1")
		{
			teleop_command.data = 21;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl2")
		{
			teleop_command.data = 22;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl3")
		{
			teleop_command.data = 23;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl4")
		{
			teleop_command.data = 24;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl5")
		{
			teleop_command.data = 25;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl6")
		{
			teleop_command.data = 26;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl7")
		{
			teleop_command.data = 27;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl8")
		{
			teleop_command.data = 28;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl9")
		{
			teleop_command.data = 29;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl10")
		{
			teleop_command.data = 30;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfl11")
		{
			teleop_command.data = 31;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr1")
		{
			teleop_command.data = 61;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr2")
		{
			teleop_command.data = 62;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr3")
		{
			teleop_command.data = 63;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr4")
		{
			teleop_command.data = 64;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr5")
		{
			teleop_command.data = 65;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr6")
		{
			teleop_command.data = 66;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr7")
		{
			teleop_command.data = 67;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr8")
		{
			teleop_command.data = 68;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr9")
		{
			teleop_command.data = 69;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr10")
		{
			teleop_command.data = 70;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		else if(command == "wfr11")
		{
			teleop_command.data = 71;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	if(ros::isShuttingDown())
	{
		std::string reason= "close";
		enc_timer.stop();
		imu_timer.stop();
		acc_timer.stop();
		pose_timer.stop();
		wifi_timer.stop();
		ping_timer.stop();
		both_lid_timer.stop();
		ping_timer_check.stop();
		recon_timer.stop();

	}
	return 0;
}