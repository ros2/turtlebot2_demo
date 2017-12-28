/**
 * @file /src/utils/serial.cpp
 *
 * @brief Utility for testing a serial port connection.
 *
 * @date September 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <ecl/command_line.hpp>
#include <ecl/errors.hpp>
#include <ecl/ipc.hpp>
#include <ecl/time.hpp>
#include <ecl/threads.hpp>
#include <ecl/devices/serial.hpp>
#include <ecl/formatters.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace utils {

/*****************************************************************************
 * Using
 ****************************************************************************/

using std::string;
using ecl::ArgException;
using ecl::CmdLine;
using ecl::SwitchArg;
using ecl::ValueArg;
using ecl::Serial;
using ecl::BaudRate;
using ecl::BaudRate_9600;
using ecl::BaudRate_38400;
using ecl::BaudRate_115200;
using ecl::DataBits_8;
using ecl::StopBits_1;
using ecl::NoParity;
using ecl::StandardException;
using ecl::Format;
using ecl::Dec;
using ecl::Hex;
using ecl::NoAlign;
using ecl::RightAlign;
using ecl::Thread;
using ecl::TimeStamp;
using ecl::MilliSleep;

/*****************************************************************************
 * Classes
 ****************************************************************************/

class Writer
{
public:
	Writer(Serial *serial_device, bool timestamps = false) :
		display_timestamps(timestamps),
		serial(serial_device),
		thread(&Writer::run,*this)
		{}
	void wait() { thread.join(); }

private:
	bool display_timestamps;
	long current_time;
	TimeStamp timestamp;
	Serial *serial;
	Thread thread;

	void run() {
		MilliSleep sleep;
		string s;
		char buffer[80];
		char *s_ptr;

		/******************************************
		** Read
		*******************************************/
		while (1) {
			// Read a line from standard input
			if ( fgets(buffer,80,stdin) == NULL ) {
				break;
			}
			// fgets always terminates with a null character, even if it manages
			// to read n-1 chars
			s_ptr = buffer;
			while ( *s_ptr != '\0') { ++s_ptr; }
			serial->write(buffer,s_ptr - buffer);
			if ( display_timestamps ) {
				timestamp.stamp();
				std::cout << "[" << timestamp << "]"  << std::endl;
			}
			sleep(5);
		}
	}
};

class Reader
{
public:
	Reader(Serial *serial_device, bool hexFormat = false, bool timestamps = false) :
		serial(serial_device),
		hex(hexFormat),
		display_timestamps(timestamps),
		format(6,RightAlign,Dec),
		hex_format(-1,NoAlign,Hex),
		thread(&Reader::run,*this)
		{}

	void wait() { thread.join(); }

private:
	void run() {
		char s[255];
		unsigned long ch_read;

		if ( hex ) { std::cout << "Hex format" << std::endl; } else { std::cout << "Ascii format" << std::endl; }

		while (1) {
			ch_read = serial->read(s,1);
//            ch_read = serial->read(s,255);
			if ( ch_read > 0 ) {
				if ( display_timestamps ) {
					time.stamp();
					std::cout << "[" << time << "] ";
				}
				for (unsigned int i = 0; i < ch_read; ++i) {
					if ( hex ) {
						std::cout << hex_format(s[i]) << " ";
					} else {
						std::cout << s[i];
					}
				}
				if ( hex ) {
					std::cout << std::endl;
				} else {
					std::cout.flush();
				}
			} else {
//				std::cout << "Timed out." << std::endl;
			}
		}
	}
	Serial *serial;
	bool hex;
	bool display_timestamps;
	long current_time;
	TimeStamp time;
	Format<long> format;
	Format<unsigned char> hex_format;
	Thread thread;

};

} // namespace utils
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::utils;

/*****************************************************************************
** Main program
*****************************************************************************/
int main(int argc, char** argv) {

    /******************************************
     * Parse for the port name
     ******************************************/
    string port;
    BaudRate baud_rate;
    bool hex(false);
    bool timestamps(false);

    try {
        CmdLine cmd("This is a simple interface for reading/echoing from a serial port [115.2k,8N1].",' ',"0.1");
        ValueArg<string> arg_port("p","port","Port to connect to",false,"/dev/ttyS0","string");
        ValueArg<string> arg_baud("b","baud","Baud rate (9600,38400,115200) [115200]",false,"115200","string");
        SwitchArg switch_hex("x","hex","Enable hex output.",false);
        SwitchArg switch_timestamps("t","timestamps","Print timestamps.",false);

        cmd.add(arg_port);
        cmd.add(arg_baud);
        cmd.add(switch_hex);
        cmd.add(switch_timestamps);
        cmd.parse(argc,argv);

        port = arg_port.getValue();
        string baud = arg_baud.getValue();
        hex = switch_hex.getValue();
        timestamps = switch_timestamps.getValue();

        if ( baud == "9600" ) {
        	baud_rate = BaudRate_9600;
        } else if ( baud == "38400" ) {
        	baud_rate = BaudRate_38400;
        } else {
        	baud_rate = BaudRate_115200;
        }

    } catch ( ArgException &e ) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }
    try {
        /******************************************
        ** Open
        *******************************************/
        Serial serial(port,baud_rate,DataBits_8,StopBits_1,NoParity);
        serial.block(5000);
        Reader reader(&serial,hex,timestamps);
        Writer writer(&serial,timestamps);

        reader.wait();
		writer.wait();
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}
