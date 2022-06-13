#include<ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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

int I, second = 0, done_here = 0;
vector<float> above_, slant_;

int Active=0,ping_ok=1,init=0,Retry=0,N_com=0, once = 0;	//Active=0 to enable ping
std::string _stat,DAT,x;
std::string uri="wss://central.brsindia.com";

ros::Timer enc_timer,imu_timer,acc_timer,pose_timer,wifi_timer,ping_timer,recon_timer;

std::string left_enc_call_message,right_enc_call_message,RID="5f297ce02b2e2b05d5a7dd78";
std::string AUTH = "{\"messagetype\":\"robot\",\"data\":\""+RID+"\"}";

std::string enc_message,imu_message,acc_message,motor_left_message;
std::string bothlidar_message,motor_right_message,robspeed_message,pose_message,message9,wifi_message;
std::string PING_MSG,ping_message=RID+"|ping";
std::string poseX,poseY,position;
int cache=0;

Json::Value RECV;
Json::Reader reader;
std::string command;

std::fstream file;

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
//				std::string temp="{\"messagetype\":\"Ping meassage\",\"message\":\"1|ping\",\"messageSend\":\"open\",\"robid\":\"1\"}";
				m_message=msg->get_payload();
				bool parseSuccessful = reader.parse(m_message, RECV);
				if ( !parseSuccessful)
				{
					DAT="NULL";
				}
				Json::Value w = RECV;
				if (w["message"].isString())
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
				}
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
			std::cout<<"> Closing connection "<<it->second->get_id()<<std::endl;

			websocketpp::lib::error_code ec;
			m_endpoint.close(it->second->get_hdl(), websocketpp::close::status::normal, "", ec);
			if(ec)
			{
				std::cout<<"> Error closing conection "<<it->second->get_id()<<": "
						 <<ec.message()<<std::endl;
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
			std::cout<<"> Connect initialization error: "<<ec.message()<<std::endl;
			return -1;
		}
	
		int new_id=m_next_id++;
		connection_metadata::ptr metadata_ptr(new connection_metadata(new_id, con->get_handle(),uri));
		m_connection_list[new_id]=metadata_ptr;

		std::cout<<"meta "<<metadata_ptr<<std::endl;

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

		boost::chrono::system_clock::time_point nt,start = boost::chrono::system_clock::now();    
		int T=0;
		while(T<1)
		{
			nt=boost::chrono::system_clock::now();
			boost::chrono::duration<double> sec =  nt - start;
			if((sec.count()>2.0))
			{
				T=1;
			}
		}
		return new_id;
	}
	void close(int id, websocketpp::close::status::value code,std::string&)
	{
		websocketpp::lib::error_code ec;
		
		con_list::iterator metadata_it=m_connection_list.find(id);
		if(metadata_it==m_connection_list.end())
		{
			std::cout<<"> No connection found with id "<<id<<std::endl;
			return;
		}

		m_endpoint.close(metadata_it->second->get_hdl(),code,"",ec);
		if(ec)
		{
			std::cout<<"> Error initiating close: "<<ec.message()<<std::endl;
		}
	}
	void send(int id, std::string message)
	{
		websocketpp::lib::error_code ec;
		
		con_list::iterator metadata_it=m_connection_list.find(id);
		if(metadata_it==m_connection_list.end())
		{
			std::cout<<"> No connection found with id "<<id<<std::endl;
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
			file.close();
			file.open("/home/brs/main_ws/src/karcher/src/test.csv",std::fstream::in);
			Active=1;
			ping_ok=1;
			init=1;
			Retry=0;
		}
		else
		{
			std::string reason= "close";
			endpoint.close(I,websocketpp::close::status::normal,reason);
			std::cout<<"stopped\n Creating\n";
			int id = endpoint.connect(uri);
			I=id;
		}
		std::cout<<"I: "<<I<<"\n";
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
		if(!file.eof() && cache==1)
		{
			getline(file,x,'\n');
			if(x.substr(0,3)=="ENC")
			{
				std::string tmp="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+"old_"+x.substr(4)+"\"}";
			}
			else
			{
				cache=0;
			}				
		}
	}
	else
	{
		if(enc_message!="NULL")
		{
			file<<"ENC,"<<enc_message<<"\n";
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
		if(!file.eof() && cache==1)
		{
			getline(file,x,'\n');
			if(x.substr(0,3)=="IMU")
			{
				std::string tmp="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+"old_"+x.substr(4)+"\"}";
			}
			else
			{
				cache=0;
			}				
		}
	}
	else
	{
		if(imu_message!="NULL")
		{
			file<<"IMU,"<<imu_message<<"\n";
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
		if(!file.eof() && cache==1)
		{
			getline(file,x,'\n');
			if(x.substr(0,3)=="ACC")
			{
				std::string tmp="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+"old_"+x.substr(4)+"\"}";
			}
			else
			{
				cache=0;
			}				
		}
	}
	else
	{
		if(acc_message!="NULL")
		{
			file<<"ACC,"<<acc_message<<"\n";
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
		if(!file.eof() && cache==1)
		{
			getline(file,x,'\n');
			if(x.substr(0,3)=="ENC")
			{
				std::string tmp="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+"old_"+x.substr(4)+"\"}";
			}
			else
			{
				cache=0;
			}
		}
	}
	else
	{
		if(pose_message!="NULL")
		{
			file<<"ENC,"<<pose_message<<"\n";
			pose_message="NULL";
		}
		cache=1;
	}
}

void bothlidar_s(const ros::TimerEvent&)
{
	bothlidar_message="|bl|"+bothlidar_message;//d is constant robot direction 
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+bothlidar_message+"\"}";

	if(init==1 and Active==1 and bothlidar_message!="NULL")
	{
		endpoint.send(I,message);
		bothlidar_message="NULL";
	}
	else
	{
		if(bothlidar_message!="NULL")
		{
			file<<"ENC,"<<bothlidar_message<<"\n";
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
		if(!file.eof() && cache==1)
		{
			getline(file,x,'\n');
			if(x.substr(0,3)=="wifi")
			{
				std::string tmp="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+"old_"+x.substr(4)+"\"}";
			}
			else
			{
				cache=0;
			}

		}
	}
	else
	{
		if(wifi_message!="NULL")
		{
			file<<"wifi,"<<wifi_message<<"\n";
			wifi_message="NULL";
		}
		cache=1;
	}
}

//Routine to send ping & check if connection with server is active or not at fixed interval
void pingCB(const ros::TimerEvent&)
{
	if(init==1)
	{
		if(ping_ok==1)
		{
			PING_MSG="{\"messagetype\":\"Ping\",\"robid\":\""+RID+"\",\"message\":\""+ping_message+"\"}";
			endpoint.send(I,PING_MSG);
			ping_ok=0;	//uncomment to enable ping
			Active=1;	//uncomment to enable ping
		}
		else
		{
			std::cout<<"Failed ping\n";
			Active=0;
			file.close();
			file.open("/home/brs/main_ws/src/karcher/src/test.csv",std::fstream::out);
			init=0;
			_stat="Failed";
			Retry=1;
		}
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
  	//poseAMCLa = msg->pose.pose.orientation.w;   
  	//ROS_INFO(msg);

	poseX = boost::lexical_cast<std::string>(X);
	poseY = boost::lexical_cast<std::string>(Y);


	pose_message="|xy|"+poseX+"|"+poseY+"|d";//d is constant robot direction 
	//position = msg->data.c_str();
	ROS_INFO("%s",pose_message.c_str());

}

void bothlidarcb(const std_msgs::String::ConstPtr& msg)
{
	bothlidar_message = msg->data;
}

void wifi_strength(const std_msgs::StringConstPtr& str)
{
	wifi_message=str->data;//d is constant robot direction 
	wifi_message="1|"+wifi_message;
	//position = msg->data.c_str();
	ROS_INFO("%s",wifi_message.c_str());

	wifi_message="|robdata|wifi|"+wifi_message;
}

int main(int argc,char **argv)
{
	bool done = false;

//Initial connection with server
        int id = endpoint.connect(uri);
		I=id;
 
	std::cout<<"Done\n";
//Authentication message for client
	endpoint.send(I,AUTH);
	
	init=1;
   	if(!file)
	{
		std::cout<<"open File Failure \n";
	}

	ros::init(argc,argv,"simple_ws_def");
	ros::NodeHandle n;

	ros::Subscriber LENC = n.subscribe<std_msgs::Int32>("/left_enc",10,lenccb);
	ros::Subscriber RENC = n.subscribe<std_msgs::Int32>("/right_enc",10,renccb);
	ros::Subscriber IMU = n.subscribe<geometry_msgs::Vector3>("/brick_imu",200,imucb);
	ros::Subscriber ACC = n.subscribe<geometry_msgs::Vector3>("/accel_imu",200,accelcb);
	ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 100, amclposecb);
	ros::Subscriber sub_sstrength = n.subscribe("signal_strength", 100, wifi_strength);

	ros::Subscriber sub_lidar = n.subscribe("/both_lidar", 1, &bothlidarcb);

	ros::Publisher mov_cmd = n.advertise<std_msgs::Int8>("/wss_teleop", 50);	//CMD_VEL

	std::stringstream ss;
	std_msgs::Int8 teleop_command;

	ros::Rate loop_rate(50); // Higher value results in faster update rates
	
	enc_timer=n.createTimer(ros::Duration (0.50),enc_s);
	imu_timer=n.createTimer(ros::Duration (1.00),imu_s);
	acc_timer=n.createTimer(ros::Duration (1.00),acc_s);
	pose_timer=n.createTimer(ros::Duration (1.00),pose_s);
	wifi_timer=n.createTimer(ros::Duration (0.50),wifi_s);
	ping_timer=n.createTimer(ros::Duration (2.0),pingCB);
	recon_timer=n.createTimer(ros::Duration (4.00),retryCB);

	while(ros::ok())
	{
		if(N_com==1)//use this in teleop
		{
			command=DAT;
			N_com=0;
		}

		if(Retry == 1 && once == 0)
		{
			teleop_command.data = 50;
			mov_cmd.publish(teleop_command);
			command="NULL";
			once = 1;
		}
		else if(Retry == 0 and once == 1)
		{
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
		else if(command == "resume")
		{
			teleop_command.data = 70;
			mov_cmd.publish(teleop_command);
			command="NULL";
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	if(ros::isShuttingDown())
	{
		std::string reason= "close";
		//endpoint.close(I,websocketpp::close::status::normal,reason);
		enc_timer.stop();
		imu_timer.stop();
		acc_timer.stop();
		pose_timer.stop();
		wifi_timer.stop();
		ping_timer.stop();
		recon_timer.stop();

		file.close();
	}
	return 0;
}