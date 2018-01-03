/**
 * @file /src/examples/exception_tracer.cpp
 *
 * @brief Backtracing glibc style.
 *
 * This is an interesting exception tracer that provides functions
 * and memory addresses of the backtrace where thrown.
 *
 * Not really useful as is, could use additional help to show
 * line numbers as well. See here for a complicated example that
 * does this:
 *
 * http://cairo.sourcearchive.com/documentation/1.9.4/backtrace-symbols_8c-source.html
 *
 * @date March, 2011
 **/
/*****************************************************************************
** Cross Platform
*****************************************************************************/

#include <ecl/config/ecl.hpp>

#ifndef ECL_IS_WIN32

/*****************************************************************************
** Includes
*****************************************************************************/

#include <execinfo.h> // backtrace
#include <signal.h>
#include <cstdlib>
#include <exception>
#include <iostream>

/*****************************************************************************
** Exceptions
*****************************************************************************/

/**
 * @cond DO_NOT_DOXYGEN
 */
class ExceptionTracer {
public:
	 ExceptionTracer() {
         void * array[25];
         int nSize = backtrace(array, 25);
         char ** symbols = backtrace_symbols(array, nSize);

         for (int i = 0; i < nSize; i++) {
             std::cout << symbols[i] << std::endl;
         }
         free(symbols);
     }
};

void f() {
	std::cout << "f()" << std::endl;
	throw ExceptionTracer();
}
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	std::cout << "Dude" << std::endl;
	f();

	return 0;
}
#else

#include <iostream>

int main(int argc, char **argv) {

	std::cout << "No backtrace support in windows (that I know of yet)." << std::endl;
	return 0;
}
#endif
/**
 * @endcond
 */
