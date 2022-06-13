#include<ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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

int I;
int Active=0,ping_ok=1,init=0,Retry=0,N_com=0;	//Active=0 to enable ping
std::string _stat,DAT,x;
std::string uri="wss://central.brsindia.com";

ros::Timer tim1,tim2,tim3,tim4,tim5,tim6,tim7,ping,recon;

std::string Pl,Pr,RID="5f297ce02b2e2b05d5a7dd78";
std::string AUTH = "{\"messagetype\":\"robot\",\"data\":\""+RID+"\"}";

std::string message1,message2,message3,message4,message5,message6,message7,message8,message9,message10,message11;
std::string PING_MSG,ping_message=RID+"|ping";
std::string message_1,message_2,message_3,message_4,message_5,message_6;
std::string poseX,poseY,position;
int cache=0;

Json::Value RECV;
Json::Reader reader;
std::string command;


float d = 0.0;
std_msgs::Float64 dist;
float thresh2=0.63, thresh1=0.12; //teleop_twist

float DIST = 0.0;
float LEFT_SET = 0.0, RIGHT_SET = 0.0;	// Left wheel & right wheel RPM setpoint 
float LEFT_SET_H = 0.0, RIGHT_SET_H = 0.0;	// Left wheel & right wheel RPM setpoint 
float LEFT_SET_T = 0.0, RIGHT_SET_T = 0.0;	// Left wheel & right wheel RPM setpoint 

geometry_msgs::Twist cmd,setpoint,heading,heading_turn;
std_msgs::Int8 dir,res;
std_msgs::Int32 STRT,STP;
//std_msgs::String msg_startzz,msg_startlsf,msg_stop,msg_pause,msg_resumezz,msg_resumelsf;
std_msgs::String msg_startzz,msg_stop,msg_pause,msg_resumezz,msg_startlwf,msg_startlsf,msg_resumelsf,msg_startsecond,msg_resumelwf;

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
	if(Retry==1)
	{
		if(Active!=1)
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
}

//Routine to send encoder data to server at fixed interval
void tim1CB(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+message1+"\"}";

	if(init==1)
	{
		if(Active==1)
		{
			if(message1!="NULL")
			{
				endpoint.send(I,message);
				message_1=message;
				message1="NULL";
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
		}
	}
	else
	{
		if(message1!="NULL")
		{
			file<<"ENC,"<<message1<<"\n";
			message1="NULL";
		}
		cache=1;
	}
}

//Routine to send IMU roll, pitch & yaw data to server at fixed interval
void tim2CB(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+message2+"\"}";
	if(init==1)
	{
		if(Active==1)
		{
			if(message2!="NULL")
			{

	//			endpoint.send(I,message);
	//			message_2=message;
				message2="NULL";
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
		}
	}
	else
	{
		if(message2!="NULL")
		{
			file<<"IMU,"<<message2<<"\n";
			message2="NULL";
		}
		cache=1;
	}
}

//Routine to send Acceleration in X, Y & Z direction to server at fixed interval
void tim3CB(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+message3+"\"}";
	if(init==1)
	{
		if(Active==1)
		{
			if(message3!="NULL")
			{

	//			endpoint.send(I,message);
	//			message_3=message;
				message3="NULL";
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
		}
	}
	else
	{
		if(message3!="NULL")
		{
			file<<"ACC,"<<message3<<"\n";
			message3="NULL";
		}
		cache=1;
	}
}

void tim4CB(const ros::TimerEvent&)
{
	if(init==1)
	{
		if(Active==1)
		{
		}
	}
	else
	{
//		file<<"  ,"<<message<<"\n";
		cache=1;
	}
}

void tim5CB(const ros::TimerEvent&)
{

	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+message8+"\"}";

	if(init==1)
	{
		if(Active==1)
		{
			if(message8!="NULL")
			{
				endpoint.send(I,message);
				message_4=message;
				message8="NULL";
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
		}
	}
	else
	{
		if(message8!="NULL")
		{
			file<<"ENC,"<<message8<<"\n";
			message8="NULL";
		}
		cache=1;
	}
}

void tim6CB(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+message9+"\"}";

	if(init==1)
	{
		if(Active==1)
		{
			if(message9!="NULL")
			{
				endpoint.send(I,message);
				message_5=message;
				message9="NULL";
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
		}
	}
	else
	{
		if(message9!="NULL")
		{
			file<<"ENC,"<<message9<<"\n";
			message9="NULL";
		}
		cache=1;
	}
}

void tim7CB(const ros::TimerEvent&)
{
	std::string message="{\"messagetype\":\"Input meassage\",\"message\":\""+RID+message10+"\"}";

	if(init==1)
	{
		if(Active==1)
		{
			if(message10!="NULL")
			{
				endpoint.send(I,message);
				message_6=message;
				message10="NULL";
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
		}
	}
	else
	{
		if(message10!="NULL")
		{
			file<<"wifi,"<<message10<<"\n";
			message10="NULL";
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
//			PING_MSG="{\"messagetype\":\"Input meassage\",\"message\":\""+ping_message+"\"}";
			PING_MSG="{\"messagetype\":\"Ping\",\"robid\":\""+RID+"\",\"message\":\""+ping_message+"\"}";
			endpoint.send(I,PING_MSG);
			std::cout<<"coming here"<<std::endl;
			ping_ok=0;	//uncomment to enable ping
			Active=1;	//uncomment to enable ping
		}
		else
		{
			std::cout<<"Failed\n";
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
	Pl = boost::lexical_cast<std::string>(a->data);
	Pl="10";
	message1="|robdata|enc|"+Pl+"|"+Pr;
}

//Right Encoder callback
void renccb(const std_msgs::Int32::ConstPtr& a)
{
	Pr = boost::lexical_cast<std::string>(a->data);
	Pr="20";
	message1="|robdata|enc|"+Pl+"|"+Pr;
}

//IMU roll, pitch & yaw callback
void imucb(const geometry_msgs::Vector3::ConstPtr& a)
{
	std::string roll,pitch,yaw;
	roll = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
	pitch = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
	yaw = boost::lexical_cast<std::string>((int)((ceil(a->z*100.0))/100.0));
	message2="|robdata|imu|"+roll+"|"+pitch+"|"+yaw;
}

//Acceleration in X, Y & Z direction callback
void accelcb(const geometry_msgs::Vector3::ConstPtr& a)
{
	std::string X,Y,Z;
	X = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
	Y = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
	Z = boost::lexical_cast<std::string>((int)((ceil(a->z*100.0))/100.0));
	message3="|robdata|acc|"+X+"|"+Y+"|"+Z;
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
void spdcb(const geometry_msgs::Vector3::ConstPtr& a)
{
	std::string SPD,R_SPD,L_SPD;
	SPD = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
	R_SPD = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
	L_SPD = boost::lexical_cast<std::string>((int)((ceil(a->z*100.0))/100.0));
	message7="|robdata|botspeed|"+SPD+"|"+R_SPD+"|"+L_SPD;
}
void mo_lcb(const geometry_msgs::Vector3::ConstPtr& a)
{
	std::string V_l,A_l;
	V_l = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
	A_l = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
	message5="|robdata|motdrivL|"+V_l+"|"+A_l;
}
void mo_rcb(const geometry_msgs::Vector3::ConstPtr& a)
{
	std::string V_r,A_r;
	V_r = boost::lexical_cast<std::string>((int)((ceil(a->x*100.0))/100.0));
	A_r = boost::lexical_cast<std::string>((int)((ceil(a->y*100.0))/100.0));
	message6="|robdata|motdrivR|"+V_r+"|"+A_r;
}
void amclposecb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

  	double Y =msg->pose.pose.position.x *10.0; // Rviz X psotition publishing in meters
  	double X = msg->pose.pose.position.y *10.0; // Rviz Y psotition
  	//poseAMCLa = msg->pose.pose.orientation.w;   
  	//ROS_INFO(msg);

	poseX = boost::lexical_cast<std::string>(X);
	poseY = boost::lexical_cast<std::string>(Y);


	message8="|xy|"+poseX+"|"+poseY+"|d";//d is constant robot direction 
	//position = msg->data.c_str();
	ROS_INFO("%s",message8.c_str());

}
void cam_feed(const std_msgs::StringConstPtr& str)
{
	message9=str->data;//d is constant robot direction 
	//position = msg->data.c_str();
	ROS_INFO("%s",message9.c_str());

}

void wifi_strength(const std_msgs::StringConstPtr& str)
{
	message10=str->data;//d is constant robot direction 
	message10="1|"+message10;
	//position = msg->data.c_str();
	ROS_INFO("%s",message10.c_str());

	message10="|robdata|wifi|"+message10;
}

int main(int argc,char **argv)
{
	bool done = false;

//Initial connection with server
        int id = endpoint.connect(uri);
		I=id;
		std::cout<<"here"<<std::endl;
 
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

	std::cout<<"2. Done\n";

	ros::Subscriber LENC = n.subscribe<std_msgs::Int32>("/left_enc",10,lenccb);
	ros::Subscriber RENC = n.subscribe<std_msgs::Int32>("/right_enc",10,renccb);
	ros::Subscriber IMU = n.subscribe<geometry_msgs::Vector3>("/brick_imu",200,imucb);
	ros::Subscriber ACC = n.subscribe<geometry_msgs::Vector3>("/accel_imu",200,accelcb);
	ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 100, amclposecb);
	ros::Subscriber cam_live = n.subscribe("cam_stream", 100,cam_feed);
	ros::Subscriber sub_sstrength = n.subscribe("signal_strength", 100, wifi_strength);


	ros::Publisher mov_cmd = n.advertise<std_msgs::String>("/teleop", 50);	//CMD_VEL

	ros::Publisher nav_zz=n.advertise<std_msgs::String>("/NAV_ZZ",10);	//START ZZ L TYPE

	ros::Publisher STOP=n.advertise<std_msgs::String>("/STOP_",10);	//STOP
    ros::Publisher PAUSE = n.advertise<std_msgs::String>("/PAUSE_",10);	//PAUSE
    ros::Publisher RESUME = n.advertise<std_msgs::String>("/RESUME_",10);	//RESUME

	ros::Publisher starting = n.advertise<std_msgs::String>("/starting_", 10);

	ros::Publisher start_sf = n.advertise<std_msgs::String>("/start_sf",10);
	ros::Publisher start_wf = n.advertise<std_msgs::String>("/start_wf",10);

	std::stringstream ss;
	std_msgs::String STR;

	ss<<"STOP";
	msg_stop.data = ss.str();
	ss<<"PAUSE";
	msg_pause.data = ss.str();
	ss<<"START_ZZ";
	msg_startzz.data = ss.str();
	ss<<"RESUME_ZZ";
	msg_resumezz.data = ss.str();
	ss<<"START_LWF";
	msg_startlwf.data = ss.str();
	ss<<"RESUME_LWF";
	msg_resumelwf.data = ss.str();
	ss<<"START_LSF";
	msg_startlsf.data = ss.str();
	ss<<"RESUME_LSF";
	msg_resumelsf.data = ss.str();
	ss<<"STARTSECOND_LAP";
	msg_startsecond.data= ss.str();

	ros::Rate loop_rate(50); // Higher value results in faster update rates
	
	tim1=n.createTimer(ros::Duration (0.50),tim1CB);
	tim2=n.createTimer(ros::Duration (1.00),tim2CB);
	tim3=n.createTimer(ros::Duration (1.00),tim3CB);
	tim4=n.createTimer(ros::Duration (1.00),tim4CB);
	tim5=n.createTimer(ros::Duration (1.00),tim5CB);
	tim6=n.createTimer(ros::Duration (1.00),tim6CB);
	tim7=n.createTimer(ros::Duration (0.50),tim7CB);
	ping=n.createTimer(ros::Duration (2.00),pingCB);
	recon=n.createTimer(ros::Duration (4.00),retryCB);

	
	while(ros::ok())
	{
		if(N_com==1)
		{
			command=DAT;
			N_com=0;
		}

		if(command == "left")
		{
			cout << command << endl; // test
			ROS_INFO("Left key Pressed");
			std::stringstream SS;
			SS<<"LEFT";
			STR.data = SS.str();
			mov_cmd.publish(STR);
			command="NULL";
		}

		else if(command == "forward")
		{
			cout << command << endl; // test
			ROS_INFO("Forward Key Pressed");
			std::stringstream SS;
			SS<<"FORWARD";
			STR.data = SS.str();
			mov_cmd.publish(STR);
			command="NULL";
		}
		else if(command == "right")
		{
			cout << command << endl; // test
			ROS_INFO("Right Key Pressed");
			std::stringstream SS;
			SS<<"RIGHT";
			STR.data = SS.str();
			mov_cmd.publish(STR);
			command="NULL";
		}
		else if(command == "backward")
		{
			cout << command << endl; // test
			ROS_INFO("BackWard Key Pressed");
			std::stringstream SS;
			SS<<"BACKWARD";
			STR.data = SS.str();
			mov_cmd.publish(STR);
			command="NULL";
		}

		else if(command == "start_lsf"){
			cout << command << endl; // test
			ROS_INFO("start_lsf key pressed");
			std::stringstream SS;
			SS<<"START_LSF";
			STR.data = SS.str();
			start_sf.publish(STR);
			command="NULL";
		}

		else if(command == "start_rsf"){
			cout << command << endl; // test
			ROS_INFO("start_rsf key pressed");
			std::stringstream SS;
			SS<<"START_RSF";
			STR.data = SS.str();
			start_sf.publish(STR);
			command="NULL";
		}

		else if(command == "start_lwf"){
			cout << command << endl; // test
			ROS_INFO("start_lwf key pressed");
			std::stringstream SS;
			SS<<"START_LWF";
			STR.data = SS.str();
			start_wf.publish(STR);
			command="NULL";
		}

		else if(command == "start_rwf"){
			cout << command << endl; // test
			ROS_INFO("start_rwf key pressed");
			std::stringstream SS;
			SS<<"START_RWF";
			STR.data = SS.str();
			start_wf.publish(STR);
			command="NULL";
		}

		else if(command == "start_nav_l")
		{
			ROS_INFO("Navigate ZZ L");
			std::stringstream SS;
			SS<<"START_L";
			STR.data = SS.str();
			nav_zz.publish(STR);
			command="NULL";
		}

		else if(command == "start_nav_r")
		{
			ROS_INFO("Navigate ZZ R");
			std::stringstream SS;
			SS<<"START_R";
			STR.data = SS.str();
			nav_zz.publish(STR);
			command="NULL";
		}

		else if (command == "stop" || command=="stopm")
		{
			ROS_INFO("Stop key pressed");
			std::stringstream SS;
			SS<<"STOP";
			STR.data = SS.str();
			STOP.publish(STR);
			command="NULL";
		}
		else if(command == "pause")
		{
			ROS_INFO("Pause key pressed");
			std::stringstream SS;
			SS<<"PAUSE";
			STR.data = SS.str();
			PAUSE.publish(STR);
			command="NULL";
		}
		else if(command == "resume")
		{
			ROS_INFO("Resume key pressed");
			std::stringstream SS;
			SS<<"RESUME";
			STR.data = SS.str();
			RESUME.publish(STR);
			command="NULL";
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	if(ros::isShuttingDown())
	{
		std::string reason= "close";
		endpoint.close(I,websocketpp::close::status::normal,reason);
		tim1.stop();
		tim2.stop();
		tim3.stop();
		tim4.stop();
		tim5.stop();
		tim6.stop();
		tim7.stop();
		ping.stop();
		recon.stop();
		file.close();
	}
	return 0;
}