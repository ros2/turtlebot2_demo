/**
 * @file /src/lib/standard_exception.cpp
 *
 * @brief Implementation for standard exceptions.
 *
 * @date April, 2009
 **/
/*****************************************************************************
** Disable check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#ifndef ECL_DISABLE_EXCEPTIONS

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/exceptions/standard_exception.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
 * Implementation [Standard Exception]
 ****************************************************************************/

StandardException::StandardException(const char* loc, ErrorFlag error ) :
    Exception(loc),
    error_flag(error)
{}

StandardException::StandardException(const char* loc, ErrorFlag error, const std::string &msg ) :
    Exception(loc),
    error_flag(error),
    detailed_message(msg)
{}

StandardException::StandardException(const char* loc, const StandardException &e ) :
    Exception(),
    error_flag(e.flag()),
    detailed_message(e.detailed_message)
{
    location = std::string(loc) + "\n         : " + e.location;
}

const char* StandardException::what() const throw() {

    std::string what_msg;

    what_msg = what_msg + "\nLocation : " + this->location + "\n" + "Flag     : " + Error(error_flag).what() + "\n";
    if ( detailed_message.size() > 0 ) {
        what_msg  = what_msg + "Detail   : " + detailed_message + "\n";
    }
    return what_msg.c_str();
}

}; // namespace ecl

#endif /* ECL_DISABLE_EXCEPTIONS */
