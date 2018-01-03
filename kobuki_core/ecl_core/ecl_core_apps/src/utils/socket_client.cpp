/**
 * @file /src/utils/socket_client.cpp
 *
 * @brief Use to test tcp/ip connections.
 *
 * @date January 2010
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config/ecl.hpp>

#ifdef ECL_IS_POSIX
#ifndef ECL_IS_MAC

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <ecl/command_line.hpp>
#include <ecl/formatters.hpp>
#include <ecl/threads/thread.hpp>
#include <ecl/time/sleep.hpp>
#include <ecl/time/timestamp.hpp>
#include <ecl/devices/socket.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace demos {

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::CmdLine;
using ecl::ValueArg;
using ecl::SwitchArg;
using ecl::ArgException;
using ecl::ConnectionHungUp;
using ecl::SocketClient;
using ecl::Format;
using ecl::Hex;
using ecl::NoAlign;
using ecl::Thread;
using ecl::MilliSleep;
using ecl::TimeStamp;


/*****************************************************************************
** Classes
*****************************************************************************/

class Reader {
public:
	Reader(SocketClient &socket_server, bool timestamps_reqd, bool hex_format_reqd = false) :
		socket(socket_server),
		timestamps(timestamps_reqd),
		hex(hex_format_reqd),
		new_line(true)
	{}

	void loop() {

        Format<unsigned char> hex_format(-1,NoAlign,Hex);
        char c;
        long ch_read;
        MilliSleep sleep;

        if ( hex ) { std::cout << "Hex format" << std::endl; } else { std::cout << "Ascii format" << std::endl; }

        while (1) {
            ch_read = socket.read(c);
            if ( ch_read > 0 ) {
                if ( hex ) {
                    if ( timestamps ) {
                    	if ( new_line ) { timestamp.stamp(); }
    					std::cout << "[" << timestamp << "] : ";
                    }
                    std::cout << hex_format(c) << std::endl;
                } else {
                    if ( timestamps && new_line ) {
    					timestamp.stamp();
    					std::cout << "[" << timestamp << "] : ";
                    }
                    std::cout << c;
                }
                std::cout.flush();
                if ( timestamps ) {
                	if ( new_line ) {
                		new_line = false;
                	} else if ( c == '\n' ) {
                		new_line = true;
                	}
                }
            } else if ( ch_read < 0 ) {
                if ( ch_read == ConnectionHungUp ) {
                    std::cout << "Server Hung Up." << std::endl;
                    break;
                }
            } else {
                // nothing read
            }
            sleep(15);
        }

	}

private:
	SocketClient &socket;
	TimeStamp timestamp;
	bool timestamps;
	bool hex;
	bool new_line;
};

class Writer {
public:
	Writer(SocketClient &socket_server, bool timestamps_reqd) : socket(socket_server), timestamps(timestamps_reqd) {}

	void loop() {
        MilliSleep sleep;
        char buffer[256];
        memset(buffer,0,256);
        char *s_ptr;
        while( 1 ) {
            if ( fgets(buffer,80,stdin) == NULL ) {
                break;
            }
            // fgets always terminates with a null character, even if it manages
            // to read n-1 chars
            s_ptr = buffer;
            while ( *s_ptr != '\0') { ++s_ptr; }
            if ( timestamps ) { timestamp.stamp(); }
            socket.write(buffer,s_ptr - buffer);
            if ( timestamps ) {
            	std::cout << "[" << timestamp << "] : ";
            }
            std::cout << buffer; // already terminated by an endl;
            std::cout.flush();
            sleep(500);
        }

	}

private:
	SocketClient &socket;
	TimeStamp timestamp;
	bool timestamps;
};

} // namespace demos
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::demos;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                 Parsing Command Line" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    int port = 0;
    string hostname;
    bool hex(false);
    bool timestamps(false);

    try {
        CmdLine cmd("This is a simple interface for making a socket client connection.",' ',"0.1");
        ValueArg<string> arg_hostname("n","hostname","Hostname of the server [localhost]",false,"localhost","string");
        ValueArg<int> arg_port("p","port","Port number to connect to [1470].",false,1470,"integer");
        SwitchArg switch_timestamps("t","timestamp","Timestamp incoming/outgoings.",false);
        SwitchArg switch_hex("x","hex","Enable hex output.",false);

        cmd.add(arg_hostname);
        cmd.add(arg_port);
        cmd.add(switch_hex);
        cmd.add(switch_timestamps);
        cmd.parse(argc,argv);

        hostname = arg_hostname.getValue();
        port = arg_port.getValue();
        hex = switch_hex.getValue();
        timestamps = switch_timestamps.getValue();

    } catch ( ArgException &e ) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }

    std::cout << "Connecting to: " << hostname << " [" << port << "]" << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Read and Write Threads   " << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    SocketClient client(hostname,port);
    Writer writer(client,timestamps);
    Reader reader(client,timestamps,hex);
    Thread read_thread(&Reader::loop, reader);
    Thread write_thread(&Writer::loop, writer);

    read_thread.join();
    write_thread.join();


    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	return 0;
}

#endif /* ECL_IS_POSIX */
#endif  /* !ECL_IS_MAC */

#if !defined ECL_IS_POSIX || ECL_IS_MAC

#include <iostream>

int main(int argc, char **argv) {

	std::cout << "This is a posix (not mac) only app." << std::endl;
	return 0;
}

#endif
