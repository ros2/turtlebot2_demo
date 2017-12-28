/**
 * @file src/examples/socketpair.cpp
 *
 * @brief Example code for a socket pair.
 *
 * @date February, 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cstdlib>
#include <ecl/config/ecl.hpp>
#ifndef ECL_IS_WIN32
  #include <unistd.h>
#endif
#include <ecl/errors/handlers.hpp>
#include "../../include/ecl/io/socketpair.hpp"

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

#ifdef ECL_IS_WIN32
DWORD WINAPI f(LPVOID args) {
	char buf[1024];
	/* This is the child. */
	while ( 1 ) {
		if( ::send(socket_pair[1], reinterpret_cast<const char*>(DATA1), sizeof(DATA1), 0) < 0 ) {
			std::cerr << "Failed to write to the threaded socket." << std::endl;
		}
		if ( ::recv(socket_pair[1], reinterpret_cast<char*>(buf), 1024, 0) < 0 ) {
			std::cerr << "Failed to read from the threaded socket." << std::endl;
		} else {
			std::cout << buf << std::endl;
		}
		Sleep(1000);
	}
	return 0;
}
#endif

int socket_error() {
#ifdef ECL_IS_WIN32
	return WSAGetLastError();
#else
	return errno;
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

	// blocking mode
	error = ecl::socketpair(socket_pair);
	// non blocking mode
	// error = ecl::socketpair(socket_pair,true);

	if ( error.flag() != ecl::NoError) {
		std::cout << error.what() << std::endl;
		std::cout << "SocketPair Error: " << socket_error() << std::endl;
		abort();
	}
	char buf[1024];
#ifdef ECL_IS_WIN32
	DWORD id;
	char thread_param[3];
	HANDLE thread_handle = CreateThread(
            NULL,                   // default security attributes
            0,                      // use default stack size
            f,       				// thread function name
            &thread_param,			// argument to thread function
            0,                      // use default creation flags
            &id				     	// returns the thread identifier
     );
	/* This is the parent */
	while ( 1 ) {
		if ( ::recv(socket_pair[0], reinterpret_cast<char*>(buf), 1024, 0) < 0 ) {
			std::cerr << "Failed to read from the main socket." << std::endl;
		} else {
			std::cout << buf << std::endl;
		}
		if( ::send(socket_pair[0], reinterpret_cast<const char*>(DATA2), sizeof(DATA2), 0) < 0 ) {
			std::cerr << "Failed to write to the main socket." << std::endl;
		}
		Sleep(1000);
	}
#else
	int child;
	if ((child = fork()) == -1) {
		std::cerr << "Failed to fork." << std::endl;
	} else if (child) {
		/* This is the parent */
		while ( 1 ) {
			if (read(socket_pair[0], buf, 1024) < 0) {
				std::cerr << "Failed to read from the socket." << std::endl;
			} else {
				std::cout << buf << std::endl;
			}
			if (write(socket_pair[0], DATA2, sizeof(DATA2)) < 0) {
				std::cerr << "Failed to write to the socket." << std::endl;
			}
			sleep(1);
		}
	} else {
		/* This is the child. */
		while ( 1 ) {
			if (write(socket_pair[1], DATA1, sizeof(DATA1)) < 0) {
				std::cerr << "Failed to write to the socket." << std::endl;
			}
			if (read(socket_pair[1], buf, 1024) < 0) {
				std::cerr << "Failed to read from the socket." << std::endl;
			} else {
				std::cout << buf << std::endl;
			}
			sleep(1); // 1 sec
		}
	}
#endif
	ecl::close_socket(socket_pair[0]);
	ecl::close_socket(socket_pair[1]);
	return 0;
}
