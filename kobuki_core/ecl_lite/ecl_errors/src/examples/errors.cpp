/**
 * @file /src/examples/errors.cpp
 *
 * @brief Unit Test for ecl error functions and macros.
 *
 * @date April, 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../include/ecl/errors/compile_time_assert.hpp"
#include "../../include/ecl/errors/handlers.hpp"
#include "../../include/ecl/errors/run_time_functions.hpp"
#include "../../include/ecl/errors/macros.hpp"


/*****************************************************************************
** Globals
*****************************************************************************/
/**
 * @cond IGNORE_THIS
 */
template <int M, int N>
class Test {
public:
	Test() {
		ecl_compile_time_assert( M >= 0 );
		ecl_compile_time_assert( N >= 0 );
	}
};
/**
 * @endcond
 */
/*****************************************************************************
** Main
*****************************************************************************/

int main() {

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                       Error functions" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Constructing an error of type 'Unknown Error'" << std::endl;
    std::cout << std::endl;
    ecl::Error error(ecl::UnknownError);
    std::cout << "Error::what() " << std::endl;
    std::cout << error.what() << std::endl;
    std::cout << "Error::print(LOC) " << std::endl;
    error.print(LOC);

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                       Asserts and Aborts" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    ecl::Error configuration_error(ecl::ConfigurationError);
    Test<3,1> test;
//    ecl_compile_time_assert(1 >= 3);
//    ecl_verbose_compile_time_assert( 1 >= 0, BAD_INEQUALITY );
//    ecl_run_time_abort ( LOC,  ConfigurationError );
//    ecl_run_time_abort ( LOC,  "Custom error report." );
    ecl_run_time_assert ( 2 < 1, LOC, configuration_error.flag() );
//    ecl_run_time_assert ( 2 < 1, LOC, ConfigurationError );
//    ecl_run_time_assert ( 2 < 1, LOC, "Inequality broken." );

    return 0;
}

