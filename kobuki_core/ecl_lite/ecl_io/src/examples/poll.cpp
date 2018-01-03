/**
 * @file /ecl_io/src/examples/poll.cpp
 *
 * @brief Test the cross platform poll implementation.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cstdlib>
#include <ecl/config/ecl.hpp>
#include <ecl/errors/handlers.hpp>
#include "../../include/ecl/io/poll.hpp"
#include "../../include/ecl/io/socketpair.hpp"
#ifndef ECL_IS_WIN32
  #include <unistd.h>
#endif

/*****************************************************************************
** Macros
*****************************************************************************/

#define DATA1 "In Xanadu, did Kublai Khan . . ."
#define DATA2 "A stately pleasure dome decree . . ."

/*****************************************************************************
** Globals
*****************************************************************************/

ecl::socket_descriptor socket_pair[2];

/*****************************************************************************
** Functions
*****************************************************************************/

int socket_error() {
#ifdef ECL_IS_WIN32
	return WSAGetLastError();
#else
	return errno;
#endif
}

void sleep_one_sec() {
#ifdef ECL_IS_WIN32
	Sleep(1000);
#else
	sleep(1);
#endif
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	ecl::SocketError error = ecl::init_sockets();
	if ( error.flag() != ecl::NoError ) {
		std::cout << error.what() << std::endl;
		abort();
	}

	// non blocking mode - need it so the read's don't block below.
	error = ecl::socketpair(socket_pair,true);

	if ( error.flag() != ecl::NoError) {
		std::cout << error.what() << std::endl;
		std::cout << "SocketPair Error: " << socket_error() << std::endl;
		abort();
	}
	ecl::socket_pollfd pfd[2];
	pfd[0].fd = socket_pair[0];
	pfd[0].events = POLLIN;
	pfd[0].revents = 0;
	pfd[1].fd = socket_pair[1];
	pfd[1].events = POLLIN;
	pfd[1].revents = 0;

	int poll_timeout = 500; // ms
	int result = 0;
	unsigned int count = 0;
	while ( count < 10 ) {
		if((result = ecl::poll_sockets(pfd, 2, poll_timeout)) < 0) {
			std::cout << socket_error();
		} else if ( result > 0 ) {
			char receiving_buffer[256];
			int n;
			for ( unsigned int i = 0; i < 2; ++i ) {
				if ( pfd[i].revents == POLLIN ) {
#ifdef ECL_IS_WIN32
					while ( ( n = ::recv(socket_pair[i], reinterpret_cast<char*>(receiving_buffer), 256, 0) ) > 0 ) {
#else
					while( (n = read(socket_pair[i], receiving_buffer, 256)) > 0) {
#endif
						std::cout << receiving_buffer << std::endl;
						if ( i == 0 ) {
#ifdef ECL_IS_WIN32
							if( ::send(socket_pair[0], reinterpret_cast<const char*>(DATA1), sizeof(DATA1), 0) < 0 ) {
#else
							if (write(socket_pair[0], DATA1, sizeof(DATA1)) < 0) {
#endif
								std::cerr << "Failed to write to the socket." << std::endl;
							}
						} else {
#ifdef ECL_IS_WIN32
							if( ::send(socket_pair[1], reinterpret_cast<const char*>(DATA2), sizeof(DATA2), 0) < 0 ) {
#else
							if (write(socket_pair[1], DATA2, sizeof(DATA2)) < 0) {
#endif
								std::cerr << "Failed to write to the socket." << std::endl;
							}
						}
					};
					pfd[i].revents = 0;
				}
			}
			sleep_one_sec();
		} else {
			// it will pass through here once to kickstart things
#ifdef ECL_IS_WIN32
			if( ::send(socket_pair[0], reinterpret_cast<const char*>(DATA1), sizeof(DATA1), 0) < 0 ) {
#else
			if (write(socket_pair[0], DATA1, sizeof(DATA1)) < 0) {
#endif
				std::cerr << "Failed to write to the socket." << std::endl;
			}
			sleep_one_sec();
		}
		++count;
	}


	ecl::close_socket(socket_pair[0]);
	ecl::close_socket(socket_pair[1]);

	return 0;
}


