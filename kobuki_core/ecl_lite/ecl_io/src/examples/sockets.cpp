/**
 * @file src/examples/sockets.cpp
 *
 * @brief Test the select implementation on different platforms.
 *
 * - http://www.gnu.org/s/libc/manual/html_node/Server-Example.html#Server-Example
 * - make socket example
 *
 * http://www.gnu.org/s/libc/manual/html_node/Inet-Example.html#Inet-Example
 * @date February, 2011.
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../include/ecl/io/sockets.hpp"

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    ecl::SocketError error = ecl::init_sockets();
    if ( error.flag() != ecl::NoError ) {
    	std::cout << error.what() << std::endl;
    } else {
    	std::cout << "Socket subsystem started up sucessfully." << std::endl;
    }
//    ecl::sockets::socket_descriptor = socket(AF_INET, SOCK_STREAM, 0);

    return 0;
}
