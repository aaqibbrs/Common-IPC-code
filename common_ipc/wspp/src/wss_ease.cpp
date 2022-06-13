#include "wss_ease.h"

using namespace std;

WSS_C::connection_metadata::connection_metadata(int id,websocketpp::connection_hdl hdl,std::string uri)
: m_id(id)
, m_hdl(hdl)
, m_status("connecting")
, m_uri(uri)
, m_server("N/A")
{}

bool WSS_C::websocket_endpoint::N_com{ 0 };
bool WSS_C::websocket_endpoint::Active{ 0 };
bool WSS_C::websocket_endpoint::init{ 0 };
bool WSS_C::websocket_endpoint::ping_ok{ 1 };

int WSS_C::websocket_endpoint::I{ 0 };
std::string WSS_C::websocket_endpoint::DAT{ "" };
std::string WSS_C::websocket_endpoint::_stat{ "" };

// bool WSS_C::N_com = 0;
// bool WSS_C::Active = 0;
// bool WSS_C::init = 0;
// bool WSS_C::ping_ok = 1;

// int WSS_C::I = 0;
// std::string WSS_C::DAT = "";
// std::string WSS_C::_stat = "";

std::string uri="wss://central.brsindia.com";
std::string RID="5f297ce02b2e2b05d5a7dd78";
std::string AUTH = "{\"messagetype\":\"robot\",\"data\":\""+RID+"\"}";
std::string ping_message = RID+"|ping";

Json::Value RECV;
Json::Reader reader;
std::string command;

void WSS_C::connection_metadata::on_open(client *c, websocketpp::connection_hdl hdl)
{
    m_status="Open";
    WSS_C::websocket_endpoint::_stat=m_status;
    //_stat=m_status;
    client::connection_ptr con=c->get_con_from_hdl(hdl);
    m_server=con->get_response_header("Server");
}

void WSS_C::connection_metadata::on_fail(client *c, websocketpp::connection_hdl hdl)
{
    m_status="Failed";
    WSS_C::websocket_endpoint::_stat=m_status;
    //_stat=m_status;
    client::connection_ptr con=c->get_con_from_hdl(hdl);
    m_server=con->get_response_header("Server");
    m_error_reason=con->get_ec().message();
}

void WSS_C::connection_metadata::on_close(client *c, websocketpp::connection_hdl hdl)
{
    m_status="Closed";
    WSS_C::websocket_endpoint::_stat=m_status;
    //_stat=m_status;
    client::connection_ptr con=c->get_con_from_hdl(hdl);
    std::stringstream s;
    s<<"close code: "<< con->get_remote_close_code()<<" ("
    << websocketpp::close::status::get_string(con->get_remote_close_code())
    << "), close reason: " <<con->get_remote_close_reason();
    m_error_reason=s.str();
}

void WSS_C::connection_metadata::on_message(websocketpp::connection_hdl hdl,client::message_ptr msg)
{
    std::cout<<"on message : "<<WSS_C::websocket_endpoint::I<<std::endl;
    if(msg->get_opcode()==websocketpp::frame::opcode::text)
    {
        m_message=msg->get_payload();
        bool parseSuccessful = reader.parse(m_message, RECV);
        if ( !parseSuccessful)
        {
            WSS_C::websocket_endpoint::DAT="NULL";
            //DAT="NULL";
        }
        Json::Value w = RECV;
        if (w["message"].isString())
        {
            WSS_C::websocket_endpoint::DAT=w["message"].asString();
            //DAT=w["message"].asString();
        }

        if(WSS_C::websocket_endpoint::DAT==ping_message)
        {
        // if(DAT==ping_message)
        // {
            WSS_C::websocket_endpoint::ping_ok=true;
            //ping_ok=true;
            std::cout<<"ping message"<<std::endl;
            //std::cout<<N_com<<std::endl;
        }
        else
        {
            WSS_C::websocket_endpoint::N_com=1;
            std::cout<<WSS_C::websocket_endpoint::N_com<<std::endl;
            // N_com=1;
            // std::cout<<N_com<<std::endl;
            std::cout<<"other than ping"<<std::endl;
        }
    }
}
websocketpp::connection_hdl WSS_C::connection_metadata::get_hdl()
{
    return m_hdl;
}
string WSS_C::connection_metadata::get_status()
{
    return m_status;
}
int WSS_C::connection_metadata::get_id()
{
    return m_id;
}

//---websocket endpoint class----------------------------------------------------------------------------
WSS_C::websocket_endpoint::websocket_endpoint(){}

void WSS_C::websocket_endpoint::destructor_image()
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
void WSS_C::websocket_endpoint::on_socket_init(websocketpp::connection_hdl hdl)
{
    m_socket_init = std::chrono::high_resolution_clock::now();
}
void WSS_C::websocket_endpoint::constructor_image()
{
    m_next_id = 0;
    m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
    m_endpoint.clear_error_channels(websocketpp::log::elevel::all);

    m_endpoint.init_asio();
    m_endpoint.start_perpetual();
    m_endpoint.set_socket_init_handler(bind(&type::on_socket_init,this,::_1));
    m_endpoint.set_tls_init_handler(bind(&type::on_tls_init,this,::_1));
    m_thread.reset(new websocketpp::lib::thread(&client::run,&m_endpoint));
}
context_ptr WSS_C::websocket_endpoint::on_tls_init(websocketpp::connection_hdl)
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
int WSS_C::websocket_endpoint::connect(std::string const & uri)
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
void WSS_C::websocket_endpoint::close(int id, websocketpp::close::status::value code,std::string&)
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
void WSS_C::websocket_endpoint::send(int id, std::string message)
{//
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
WSS_C::connection_metadata::ptr WSS_C::websocket_endpoint::get_metadata(int id) const
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
//----------------------------------------------------------------------------------------------