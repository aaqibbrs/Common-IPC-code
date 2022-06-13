#ifndef WSS_EASE
#define WSS_EASE

#include<ros/ros.h>

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

typedef websocketpp::client<websocketpp::config::asio_tls_client> client;
typedef websocketpp::lib::shared_ptr<boost::asio::ssl::context> context_ptr;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

extern std::string uri;
extern std::string RID;
extern std::string AUTH;
extern std::string ping_message;

namespace WSS_C
{
    class connection_metadata
    {
        public:
            typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;
            connection_metadata(int id,websocketpp::connection_hdl hdl,std::string uri);

            void on_open(client *c, websocketpp::connection_hdl hdl);
            void on_fail(client *c, websocketpp::connection_hdl hdl);
            void on_close(client *c, websocketpp::connection_hdl hdl);
            void on_message(websocketpp::connection_hdl hdl,client::message_ptr msg);
            websocketpp::connection_hdl get_hdl();
            std::string get_status();
            int get_id();

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
            websocket_endpoint();
            void destructor_image();
            void constructor_image();
            
            void on_socket_init(websocketpp::connection_hdl hdl);
            context_ptr on_tls_init(websocketpp::connection_hdl);
            int connect(std::string const & uri);
            
            void close(int id, websocketpp::close::status::value code,std::string&);
            void send(int id, std::string message);
            connection_metadata::ptr get_metadata(int id) const;

            static int I;
            static bool init;
            static bool Active;
            static bool N_com;
            static bool ping_ok;
            static std::string DAT, _stat;

        private:
            typedef std::map<int,connection_metadata::ptr> con_list;
            con_list m_connection_list;
            client m_endpoint;
            websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
            int m_next_id;
            std::chrono::high_resolution_clock::time_point m_tls_init, m_socket_init, m_start;
    };
}
#endif