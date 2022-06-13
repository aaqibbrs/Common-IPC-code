#include"wss_ease.h"

WSS_C::websocket_endpoint ping_endpoint;

bool Retry = false;

using namespace std;

void retryCB(const ros::TimerEvent&)
{
	if(Retry==true)
	{
		if(WSS_C::websocket_endpoint::Active!=true)
		{
		// if(Active!=true)
		// {
			if(WSS_C::websocket_endpoint::_stat=="Open")
			{
			// if(_stat=="Open")
			// {
				ping_endpoint.send(WSS_C::websocket_endpoint::I,AUTH);
				WSS_C::websocket_endpoint::Active=true;
				WSS_C::websocket_endpoint::ping_ok=true;
				WSS_C::websocket_endpoint::init=true;
				// ping_endpoint.send(I,AUTH);
				// Active=true;
				// ping_ok=true;
				// init=true;
				Retry=false;
			}
			else
			{
				std::string reason= "close";
				ping_endpoint.close(WSS_C::websocket_endpoint::I,websocketpp::close::status::normal,reason);
				//ping_endpoint.close(I,websocketpp::close::status::normal,reason);
				std::cout<<"stopped\n Creating\n";
				int id = ping_endpoint.connect(uri);
				WSS_C::websocket_endpoint::I=id;
				//I=id;
			}
			std::cout<<"I: "<<WSS_C::websocket_endpoint::I<<"\n";
			//std::cout<<"I: "<<I<<"\n";
		}
	}
}

void pingCB(const ros::TimerEvent&)
{
	if(WSS_C::websocket_endpoint::init==true)
	{
	// if(init==true)
	// {
		if(WSS_C::websocket_endpoint::ping_ok==true)
		{
		// if(ping_ok==true)
		// {
			std::string PING_MSG="{\"messagetype\":\"Ping\",\"robid\":\""+RID+"\",\"message\":\""+ping_message+"\"}";
			ping_endpoint.send(WSS_C::websocket_endpoint::I,PING_MSG);
			//ping_endpoint.send(:I,PING_MSG);
			std::cout<<"Ping"<<std::endl;
			WSS_C::websocket_endpoint::ping_ok=false;	//uncomment to enable ping
			WSS_C::websocket_endpoint::Active=true;	//uncomment to enable ping
			// ping_ok=false;	//uncomment to enable ping
			// Active=true;	//uncomment to enable ping
		}
		else
		{
			std::cout<<"Failed\n";
			WSS_C::websocket_endpoint::Active=false;

			WSS_C::websocket_endpoint::init=false;
			WSS_C::websocket_endpoint::_stat="Failed";
			// Active=false;

			// init=false;
			// _stat="Failed";
			Retry=true;
		}
	}
}
int main(int argc,char **argv)
{
    bool done = false;

    ping_endpoint.constructor_image();
    int id = ping_endpoint.connect(uri);
    WSS_C::websocket_endpoint::I=id;

    WSS_C::websocket_endpoint::init = false;

    WSS_C::websocket_endpoint::Active = false;
	std::cout<<" id is : "<<id<<std::endl;
	std::cout<<" I from main : "<< WSS_C::websocket_endpoint::I<<std::endl;
	// I=id;

	// std::cout<<" I from main : "<< I<<std::endl;
    // init = false;

    // Active = false;

	std::cout<<"Done\n";
	ping_endpoint.send(WSS_C::websocket_endpoint::I,AUTH);

    WSS_C::websocket_endpoint::init = true;
	// ping_endpoint.send(I,AUTH);

    // init = true;
    ros::init(argc,argv,"wss_ping");
    ros::NodeHandle n;

    ros::Timer ping=n.createTimer(ros::Duration (2.00),pingCB);
	ros::Timer recon=n.createTimer(ros::Duration (4.00),retryCB);

    ros::Rate loop_rate(10);
    while(ros::ok())
	{
        ros::spinOnce();
		std::cout<<" from ping node : "<<WSS_C::websocket_endpoint::N_com<<" I is : "<< WSS_C::websocket_endpoint::I<<std::endl;
		std::cout<<"DAT from ping : "<<WSS_C::websocket_endpoint::DAT<<std::endl;
		if(WSS_C::websocket_endpoint::N_com == 1)
		{
			WSS_C::websocket_endpoint::N_com = 0;
		}
        loop_rate.sleep();
    }
    if(ros::isShuttingDown())
	{
		ping_endpoint.destructor_image();
        recon.stop();
		ping.stop();
	}
    ros::spin();
    return 0;
}