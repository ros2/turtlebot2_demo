/**
 * @file /src/benchmarks/files.cpp
 *
 * @brief Benchmark comparing various file writing/reading alternatives.
 *
 * @date September 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <fstream>
#include <ecl/threads/priority.hpp>
#include <ecl/time/stopwatch.hpp>
#include <ecl/devices/ofile.hpp>
#include <ecl/devices/shared_file.hpp>
#include <ecl/streams/log_stream.hpp>
#include <ecl/streams/text_stream.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Append;
using ecl::New;
using ecl::OFile;
using ecl::SharedFile;
using ecl::LogStream;
using ecl::TextStream;
using ecl::StandardException;
using ecl::StopWatch;
using ecl::TimeStamp;

/*****************************************************************************
** Enums
*****************************************************************************/

enum LogMode {
	Warning,
	Error
};

/*****************************************************************************
** Main
*****************************************************************************/

int main() {
	try {
		ecl::set_priority(ecl::RealTimePriority4);
	} catch ( StandardException &e ) {
		// dont worry about it.
	}

    StopWatch stopwatch;
	TimeStamp times[22];
	unsigned int lines_to_write = 500;// Buffer::buffer_size/10 + 1;
    float f = 33.54;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   OFile Write" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    OFile o_file("odude.txt",New);
    for ( unsigned int i = 0; i < lines_to_write; ++i ) { // Avoid speedups from caching affecting times.
    	o_file.write("Heya Dude\n",10);
    }
    o_file.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	o_file.write("Heya Dude\n",10);
    }
    o_file.flush();
    times[0] = stopwatch.split();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                  Shared File Write" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    SharedFile s_file("sdude.txt",New);
    for ( unsigned int i = 0; i < lines_to_write; ++i ) { // Avoid speedups from caching affecting times.
    	s_file.write("Heya Dude\n",10);
    }
    s_file.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	s_file.write("Heya Dude\n",10);
    }
    s_file.flush();
    times[1] = stopwatch.split();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                     TextStream<OFile>" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    TextStream<OFile> ostream;
    ostream.device().open("dude_stream.txt",New);
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
        ostream << "Heya Dude\n";
    }
    ostream.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
        ostream << "Heya Dude\n";
    }
    ostream.flush();
    times[2] = stopwatch.split();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
        ostream << f << "\n";
    }
    ostream.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
        ostream << f << "\n";
    }
    ostream.flush();
    times[3] = stopwatch.split();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                     std::ofstream" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::ofstream cpp_ostream("dude_ofstream.txt");
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
        cpp_ostream << "Heya Dude\n";
    }
    cpp_ostream.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
        cpp_ostream << "Heya Dude\n";
    }
    cpp_ostream.flush();
    times[4] = stopwatch.split();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	cpp_ostream << f << "\n";
    }
    cpp_ostream.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	cpp_ostream << f << "\n";
    }
    cpp_ostream.flush();
    times[5] = stopwatch.split();
    cpp_ostream.close();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                     LogStream" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    LogStream log_stream("dude_logstream.txt",New);
	log_stream.disableHeader();
	log_stream.disableTimeStamp();
	log_stream.enableMode(Warning,"WARNING");

    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	LOG(log_stream,Warning) << "Heya Dude\n";
    }
    FLUSH(log_stream);
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	LOG(log_stream,Warning) << "Heya Dude\n";
    }
    FLUSH(log_stream);
    times[9] = stopwatch.split();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	LOG(log_stream,Warning) << f << "\n";
    }
    FLUSH(log_stream);
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	LOG(log_stream,Warning) << f << "\n";
    }
    FLUSH(log_stream);
    times[10] = stopwatch.split();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Times" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Writing Char Strings:" << std::endl;
    std::cout << "   OFile write       : " << times[0].nsec() << " ns" << std::endl;
    std::cout << "   SharedFile write  : " << times[1].nsec() << " ns" << std::endl;
    std::cout << "   OFile stream      : " << times[2].nsec() << " ns" << std::endl;
    std::cout << "   LogStream         : " << times[9].nsec() << " ns" << std::endl;
    std::cout << "   C++ ofstream      : " << times[4].nsec() << " ns" << std::endl;
    std::cout << "Streaming Floats:" << std::endl;
    std::cout << "   OFileStream       : " << times[3].nsec() << " ns" << std::endl;
    std::cout << "   Log stream        : " << times[10].nsec() << " ns" << std::endl;
    std::cout << "   C++ ofstream      : " << times[5].nsec() << " ns" << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	return 0;
}
