/**
 * @file /src/benchmarks/string_conversions.cpp
 *
 * @brief Benchmarks the performance of various string conversions.
 *
 * Benchmarks the performance of various string converters. Namely the
 * c sprintf, the c++ ostringstream and ecl's converters (refer to
 * package ecl_utilities).
 *
 * @date May 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <sstream>
#include <ecl/threads/priority.hpp>
#include <ecl/time/stopwatch.hpp>
#include <ecl/time/timestamp.hpp>
#include <ecl/converters.hpp>

//#include <fastformat/fastformat.hpp>
//#include <fastformat/sinks/char_buffer.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using std::ostringstream;
using ecl::StopWatch;
using ecl::TimeStamp;
using ecl::Converter;
using ecl::StandardException;

/*****************************************************************************
** Main
*****************************************************************************/


int main()
{
	try {
		ecl::set_priority(ecl::RealTimePriority4);
	} catch ( StandardException &e ) {
		// dont worry about it.
	}
    StopWatch stopwatch;
    TimeStamp time_converters[10], time_sprintf[10], time_iostreams[10];

    string str;
    int i = -11;
    unsigned int ui = 123;
    long l = -211123;
    float f = -16.123f;
    double d = -2316.1234;
    char buffer[30];
    char* char_string;
    Converter<char*> toCharString;
//    Converter<string> toString;
    ostringstream ostream;

    for (int j = 0; j < 50; ++j ) {
        sprintf(buffer,"%d",i);         // First sprintf is always slow, it caches some crap I think.
        sprintf(buffer,"%f",f);         // Like above, but for floats.
        ostream << i;                   // Ditto
        ostream << l;                   // Ditto
    }

    /*********************
    ** Sprintf
    **********************/
    stopwatch.restart();
    sprintf(buffer,"%d",i);
    time_sprintf[0] = stopwatch.split();
    sprintf(buffer,"%d",ui);
    time_sprintf[1] = stopwatch.split();
    sprintf(buffer,"%ld",l);
    time_sprintf[2] = stopwatch.split();
    sprintf(buffer,"%f",f);
    time_sprintf[3] = stopwatch.split();
    sprintf(buffer,"%lf",d);
    time_sprintf[4] = stopwatch.split();

    /*********************
    ** IOStreams
    **********************/
    stopwatch.restart();
    ostream << i;
    time_iostreams[0] = stopwatch.split();
    ostream << ui;
    time_iostreams[1] = stopwatch.split();
    ostream << l;
    time_iostreams[2] = stopwatch.split();
    ostream << f;
    time_iostreams[3] = stopwatch.split();
    ostream << d;
    time_iostreams[4] = stopwatch.split();

//    /*********************
//    ** FastFormat
//    **********************/
//    char char_buffer[250];
//    TimeStamp time_ff[10];
//    fastformat::sinks::char_buffer_sink ff_char_buffer(250,char_buffer);
//    fastformat::write(ff_char_buffer, i);
//    fastformat::write(ff_char_buffer, l);
//    stopwatch.restart();
//    fastformat::write(ff_char_buffer, i);
//    time_ff[0] = stopwatch.split();
//    fastformat::write(ff_char_buffer, l);
//    time_ff[1] = stopwatch.split();
////    fastformat::write(ff_char_buffer, f);
//    time_ff[2] = stopwatch.split();
////    fastformat::write(ff_char_buffer, d);
//    time_ff[3] = stopwatch.split();
//
    /*********************
    ** Converter
    **********************/
    char_string = toCharString(f);
    char_string = toCharString(f);
    char_string = toCharString(f);
    char_string = toCharString(f);
    stopwatch.restart();
    char_string = toCharString(i);
    time_converters[0] = stopwatch.split();
    char_string = toCharString(ui);
    time_converters[1] = stopwatch.split();
    char_string = toCharString(l);
    time_converters[2] = stopwatch.split();
    char_string = toCharString(f);
    time_converters[3] = stopwatch.split();
    char_string = toCharString(d);
    time_converters[4] = stopwatch.split();


    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "  Performance comparison of char string conversion apis" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Converter<char*>(int)          " << " Time: " << time_converters[0] <<  std::endl;
    std::cout << "Converter<char*>(unsigned int) " << " Time: " << time_converters[1] <<  std::endl;
    std::cout << "Converter<char*>(long)         " << " Time: " << time_converters[2] <<  std::endl;
    std::cout << "Converter<char*>(float)        " << " Time: " << time_converters[3] << std::endl;
    std::cout << "Converter<char*>(double)       " << " Time: " << time_converters[4] << std::endl;
    std::cout << std::endl;
    std::cout << "sprintf(int)                   " << " Time: " << time_sprintf[0] <<  std::endl;
    std::cout << "sprintf(unsigned int)          " << " Time: " << time_sprintf[1] <<  std::endl;
    std::cout << "sprintf(long)                  " << " Time: " << time_sprintf[2] <<  std::endl;
    std::cout << "sprintf(float)                 " << " Time: " << time_sprintf[3] <<  std::endl;
    std::cout << "sprintf(double)                " << " Time: " << time_sprintf[4] <<  std::endl;
    std::cout << std::endl;
    std::cout << "ostringstream(int)             " << " Time: " << time_iostreams[0] <<  std::endl;
    std::cout << "ostringstream(unsigned int)    " << " Time: " << time_iostreams[1] <<  std::endl;
    std::cout << "ostringstream(long)            " << " Time: " << time_iostreams[2] <<  std::endl;
    std::cout << "ostringstream(float)           " << " Time: " << time_iostreams[3] <<  std::endl;
    std::cout << "ostringstream(double)          " << " Time: " << time_iostreams[4] <<  std::endl;
    std::cout << std::endl;
//    std::cout << "fastformat(int)                " << " Time: " << time_ff[0] <<  std::endl;
//    std::cout << "fastformat(long)               " << " Time: " << time_ff[1] <<  std::endl;
//    std::cout << "fastformat(float)              " << " Time: " << time_ff[2] <<  std::endl;
//    std::cout << "fastformat(double)             " << " Time: " << time_ff[3] <<  std::endl;
//    std::cout << std::endl;

    return 0;
}

