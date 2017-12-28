/**
 * @file /src/examples/exceptions.cpp
 *
 * @brief Unit test for ecl exceptions.
 *
 * Can't really gtest these, so this is more of a coverage test per se as
 * well as an example of its usage.
 *
 * @sa @ref errorsExceptions "Exceptions Guide".
 *
 * @date April, 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdio> // fopen
#include <iostream>
#include <ecl/config/ecl.hpp>
#include "../../include/ecl/exceptions/exception.hpp"
#include "../../include/ecl/exceptions/data_exception.hpp"
#include "../../include/ecl/exceptions/macros.hpp"
#include "../../include/ecl/exceptions/standard_exception.hpp"

/*****************************************************************************
** Disable check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#ifndef ECL_DISABLE_EXCEPTIONS

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl;

/*****************************************************************************
** Functions
*****************************************************************************/

void f(int i) ecl_debug_throw_decl(StandardException)
{
    if ( i > 5 ) {
        ecl_debug_throw(StandardException(LOC,InvalidInputError,"Debug throw test failed, input integer exceeded."));
    } else {
//        cout << "Valid integer supplied." << endl;
    }
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                  Standard Exceptions" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    try {
        throw StandardException(LOC, NoError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, UnknownError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, OutOfRangeError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, ConstructorError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, DestructorError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, ConversionError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, OpenError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, CloseError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, InvalidArgError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, ConfigurationError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, ConnectionError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, ReadError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, WriteError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, NotInitialisedError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, PermissionsError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, MemoryError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, UsageError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, RaiiError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, ArgNotSupportedError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, NotSupportedError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, BusyError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, OutOfResourcesError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }
    try {
        throw StandardException(LOC, InterruptedError);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "       Standard Exceptions - Custom Message" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    try {
        throw StandardException(LOC, OutOfRangeError,"Custom detailed message.");
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "       Standard Exceptions - Custom Message" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;
    try {
        throw StandardException(LOC, OutOfRangeError,"Custom detailed message.");
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "         Assert Exception Macro" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    try {
        ecl_assert_throw(2<1, StandardException(LOC, InvalidInputError,"Two is not smaller than one!"));
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "             Debug Throw Macro" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    try {
        f(7);
    } catch ( StandardException &e ) {
        std::cout << e.what() << std::endl;
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                 Rethrowing" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    try {
        throw StandardException(LOC, ConfigurationError,"Starting spot for a rethrow.");
    } catch ( StandardException &e1 ) {
        try {
            throw StandardException(LOC, e1);
        } catch ( StandardException &e2 ) {
            std::cout << e2.what() << std::endl;
        }
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Try Catch" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;


    std::cout << "An ecl try-catch block." << std::endl;
    std::cout << std::endl;
    StandardException e_original(LOC,UnknownError);
	ecl_try {
		std::cout << "Try" << std::endl;
		ecl_throw(StandardException(LOC,e_original));
	} ecl_catch( StandardException &e ) {
		std::cout << "Catch" << std::endl;
		std::cout << e.what() << std::endl;
	}
    std::cout << "An debug try-catch block." << std::endl;
    std::cout << std::endl;
	ecl_debug_try {
		std::cout << "Try" << std::endl;
		ecl_debug_throw(StandardException(LOC,e_original));
	} ecl_debug_catch( StandardException &e ) {
		std::cout << "Catch" << std::endl;
		// std::cout << e.what() << std::endl; // This fails in release
	}


    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                 Data Exception" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    try {
        throw DataException<int>(LOC, ConfigurationError,"Throwing a data exception with an int.", 7);
    } catch ( DataException<int> &e ) {
        std::cout << "Data: " << e.data() << std::endl;
        std::cout << e.what() << std::endl;
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    return 0;
}

#else
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
	std::cout << "Ecl exceptions are unavailable (ecl was compiled with ECL_DISABLE_EXCEPTIONS)." << std::endl;

	return 0;
}

#endif /* ECL_DISABLE_EXCEPTIONS */
