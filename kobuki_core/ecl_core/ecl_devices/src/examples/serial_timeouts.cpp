/**
 * @file /ecl_devices/src/examples/serial_timeouts.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 03/09/2011
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <ecl/time/timestamp.hpp>
#include "../../include/ecl/devices/serial.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace ecl;

/*****************************************************************************
** Functions
*****************************************************************************/

void print_usage() {
	std::cout << "Usage: demo_serial_timeouts [port]" << std::endl;
	std::cout << std::endl;
	std::cout << "  default value for port is /dev/ttyUSB0" << std::endl;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    std::string port("/dev/ttyUSB0");
    if ( argc > 1 ) {
    	std::string arg(argv[1]);
    	if ( arg == "--help" ) {
    		print_usage();
    		return 0;
    	} else {
    		port = argv[1];
    	}
    }
    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Serial Timeouts" << std::endl;
    std::cout << "***********************************************************" << std::endl;
	std::cout << "* For demo'ing low latency read timeouts on posix systems." << std::endl;
	std::cout << "* Timeouts < 100ms use a custom loop, > 100ms use termios." << std::endl;
	std::cout << "* Hook this up to a serial cable to test (actual" << std::endl;
	std::cout << "* connection not important)." << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    Serial serial;
    try {
    	serial.open(port,BaudRate_115200,DataBits_8,StopBits_1,NoParity);
    } catch (StandardException &e ) {
    	std::cout << "[ERROR] : error opening " << port << std::endl;
    	std::cout << std::endl;
    	print_usage();
    	return 0;
    }
    TimeStamp time, pre_read_time;
    char buffer[256];

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                  100 ms timeouts" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "* This will use termios to scan." << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    serial.block(100);
    for ( unsigned int i = 0; i < 10; ++i ) {
    	pre_read_time.stamp();
    	long result = serial.read(buffer,256);
    	time.stamp();
    	if ( result > 0 ) {
    		std::cout << "[INFO] : read " << result << " bytes." << std::endl;
    	} else if ( result == 0 ) {
    		std::cout << "[INFO] : timed out [" << (time - pre_read_time) << "]." << std::endl;
    	} else {
    		std::cout << "[INFO] : error " << result << "." << std::endl;
    	}
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                  50 ms timeouts" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "* This will internally scan with 5ms loops." << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    serial.block(50);
    // uncomment this to test reading without a timeout on a loopback connection.
    // buffer[0] = 'a';
    // buffer[1] = 'b';
    // serial.write(buffer,2);
    for ( unsigned int i = 0; i < 10; ++i ) {
    	pre_read_time.stamp();
    	long result = serial.read(buffer,256);
    	time.stamp();
    	if ( result > 0 ) {
    		std::cout << "[INFO] : read " << result << " bytes." << std::endl;
    	} else if ( result == 0 ) {
    		std::cout << "[INFO] : timed out [" << (time - pre_read_time) << "]." << std::endl;
    	} else {
    		std::cout << "[INFO] : error " << result << "." << std::endl;
    	}
    }
    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                  20 ms timeouts" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "* This will internally scan with 2ms loops." << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    serial.block(20);
    for ( unsigned int i = 0; i < 10; ++i ) {
    	pre_read_time.stamp();
    	long result = serial.read(buffer,256);
    	time.stamp();
    	if ( result > 0 ) {
    		std::cout << "[INFO] : read " << result << " bytes." << std::endl;
    	} else if ( result == 0 ) {
    		std::cout << "[INFO] : timed out [" << (time - pre_read_time) << "]." << std::endl;
    	} else {
    		std::cout << "[INFO] : error " << result << "." << std::endl;
    	}
    }
    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                  5 ms timeouts" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "* This will internally scan with 1ms loops." << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    serial.block(5);
    for ( unsigned int i = 0; i < 10; ++i ) {
    	pre_read_time.stamp();
    	long result = serial.read(buffer,256);
    	time.stamp();
    	if ( result > 0 ) {
    		std::cout << "[INFO] : read " << result << " bytes." << std::endl;
    	} else if ( result == 0 ) {
    		std::cout << "[INFO] : timed out [" << (time - pre_read_time) << "]." << std::endl;
    	} else {
    		std::cout << "[INFO] : error " << result << "." << std::endl;
    	}
    }
    return 0;
}
