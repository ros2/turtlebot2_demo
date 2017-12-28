/**
 * @file /src/examples/log_streams.cpp
 *
 * @brief Demo for log streams.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <ecl/threads/thread.hpp>
#include <ecl/streams/log_stream.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::New;
using ecl::LogStream;
using ecl::Thread;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace demos {

/*****************************************************************************
** Enums
*****************************************************************************/

enum LogModes {
    Warning,
    Error,
    Debug,
};

void enableModes( LogStream &log_stream ) {
	log_stream.enableHeader();
	log_stream.enableTimeStamp();
	log_stream.enableMode(Warning,"WARNING");
	log_stream.enableMode(Error,"ERROR");
	log_stream.enableMode(Debug,"DEBUG");
}

void f() {
    LogStream logstream("test.log",New);
    enableModes(logstream);
    LOG(logstream,Debug) << "Debug message from a threaded function.\n";
    FLUSH(logstream);
}

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
    std::cout << "                      LogStream" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "This program will log from main and a parallel thread" << std::endl;
    std::cout << "to a shared log file call 'test.log'." << std::endl;

    LogStream logstream("test.log",New);
    enableModes(logstream);
    Thread thread(f);
    LOG(logstream,Debug) << "Debug message from main.\n";
    FLUSH(logstream);
    thread.join();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	return 0;
}
