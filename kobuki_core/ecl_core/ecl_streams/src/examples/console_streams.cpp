/**
 * @file /src/examples/console_streams.cpp
 *
 * @brief Demo for the console streams.
 *
 * @date October 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <ecl/devices/console.hpp>
#include <ecl/streams/text_stream.hpp>
#include <ecl/streams/console_streams.hpp>
#include <ecl/streams/manipulators/end_of_line.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::OConsole;
using ecl::IConsole;
using ecl::endl;
using ecl::TextStream;
using ecl::IConsoleStream;
using ecl::OConsoleStream;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {

	bool test_istreams = false;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   OConsole Stream" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "These can't really be tested very easily by google tests, so" << std::endl;
    std::cout << "it's just provided here as a demo, but functionally is a" << std::endl;
    std::cout << "unit test (requiring human approval)." << std::endl;
    std::cout << std::endl;

    OConsoleStream ostream;

    std::cout << "Streaming char." << std::endl;
    ostream << 'c' << '\n';
    std::cout << "Streaming char string." << std::endl;
    ostream << "Dude\n";
    std::cout << "Streaming string." << std::endl;
    string dude("dude_string\n");
    ostream << dude;
    std::cout << "Streaming integers." << std::endl;
    short si = 1;
    ostream << si << '\n';
    int i = 2;
    ostream << i << '\n';
    long l = 3;
    ostream << l << '\n';
    long long ll = 4;
    ostream << ll << '\n';
    unsigned short us = 5;
    ostream << us << '\n';
    unsigned int ui = 6;
    ostream << ui << '\n';
    unsigned long ul = 77;
    ostream << ul << '\n';
    unsigned long long ull = 8888;
    ostream << ull << '\n';
    std::cout << "Streaming temporary integers." << std::endl;
    ostream << 77 << '\n';
    std::cout << "Streaming a boolean." << std::endl;
    bool test = true;
    ostream << test << '\n';
    ostream << false << '\n';
    std::cout << "Streaming floating point values." << std::endl;
    float f = 32.1;
    double d = -33.3;
    ostream << f << '\n';
    ostream << d << '\n';

    std::cout << "Flushing" << std::endl;
    ostream.flush();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                OConsole Manipulators" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    ostream << "dude with an ecl::endl" << endl;

    if ( test_istreams ) {
		std::cout << std::endl;
		std::cout << "***********************************************************" << std::endl;
		std::cout << "                   IConsole Stream" << std::endl;
		std::cout << "***********************************************************" << std::endl;
		std::cout << std::endl;

		char c;
		string response;
		IConsoleStream istream;
	//	TextStream<IConsole> istream;

		std::cout << "Enter a char." << std::endl;
		istream >> c;
		std::cout << "Char read: " << c << std::endl;
		std::cout << "Enter a word." << std::endl;
		istream >> response;
		std::cout << "Word read: " << response << std::endl;
		std::cout << "Enter a short: " << std::endl;
		istream >> si;
		if ( istream.fail() ) {
			std::cout << "Short read failed: " << istream.errorStatus().what() << std::endl;
		} else {
			std::cout << "Short read: " << si << std::endl;
		}
		std::cout << "Enter a bool: " << std::endl;
		istream >> test;
		if ( istream.fail() ) {
			std::cout << "Bool read failed: " << istream.errorStatus().what() << std::endl;
		} else {
			std::cout << "Bool read: ";
			if ( test ) {
				std::cout << "true" << std::endl;
			} else {
				std::cout << "false" << std::endl;
			}
		}
		std::cout << "Enter a double:" << std::endl;
		istream >> d;
		if ( istream.fail() ) {
			std::cout << "Double read failed: " << istream.errorStatus().what() << std::endl;
		} else {
			std::cout << "Double read: " << d << std::endl;
		}
    }
    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	return 0;
}
