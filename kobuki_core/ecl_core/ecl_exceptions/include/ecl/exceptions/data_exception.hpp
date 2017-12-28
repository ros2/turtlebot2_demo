/**
 * @file /include/ecl/exceptions/data_exception.hpp
 *
 * @brief Custom ecl exceptions that carry bundled data.
 *
 * Exceptions for try-catch handling. These exceptions
 * allow bundling of some data with the exception.
 *
 * @date August 2008
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_EXCEPTIONS_DATA_EXCEPTION_HPP_
#define ECL_EXCEPTIONS_DATA_EXCEPTION_HPP_

/*****************************************************************************
** Disable check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#ifndef ECL_DISABLE_EXCEPTIONS

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <sstream>
#include <ecl/errors/handlers.hpp>
#include "exception.hpp"
#include <ecl/errors/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [DataException<Data>]
*****************************************************************************/
/**
 * @brief Extended exception class that bundles location, message and data.
 *
 * This exception class extends the StandardException to include an additional
 * component, a single data object. Usually its a good idea to keep the data
 * object small - it is copied into the exception so that it cannot be lost
 * if the original object should die.
 *
 * The bundled data object must be streamable to standard output if this class
 * is going to utilise the generic what() output.
 *
 * @sa Exception, StandardException, @ref errorsExceptions "Exceptions Guide".
 **/
template <typename Data>
class DataException : public Exception
{
    public:
        DataException(const char* loc, ErrorFlag error, Data &d );
        DataException(const char* loc, ErrorFlag error, const std::string &msg, const Data &d );
        DataException(const char* loc, const DataException<Data> &e );

        virtual ~DataException() throw() {}

        const char* what() const throw();

        const ErrorFlag& flag() const { return error_type; } /**< @brief Flag enumerating the type of exception thrown. **/
        const Data& data() const { return error_data; } /**< @brief The bundled data object. **/

    private:
        ErrorFlag error_type;
        Data error_data;
        std::string message;
};

/*****************************************************************************
 * Implementation
 ****************************************************************************/
/**
 * Default constructor for data exceptions.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 * @param error : enumerated exception error type.
 * @param d : the data bundled with the exception.
 **/
template <typename Data>
DataException<Data>::DataException(const char* loc, ErrorFlag error, Data &d ) :
    Exception(loc),
    error_type(error),
    error_data(d)
{}
/**
 * Constructor for data exceptions with a custom message.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 * @param error : enumerated exception error type.
 * @param msg : extra detail message.
 * @param d : the data bundled with the exception.
 **/
template <typename Data>
DataException<Data>::DataException(const char* loc, ErrorFlag error, const std::string &msg, const Data &d ) :
    Exception(loc),
    error_type(error),
    error_data(d),
    message(msg)
{}
/**
 * Constructor for data exceptions that enables rethrowing of an existing exception up
 * the heirarchy with a new code location stamp.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 * @param e : a caught standard exception that is to be rethrown.
 **/
template <typename Data>
DataException<Data>::DataException(const char* loc, const DataException<Data> &e ) :
    Exception(loc),
    error_type(e.flag()),
    error_data(e.data()),
    message(e.message)
{
    location = std::string(loc) + "\n         : " + e.location;
}
/**
 * Default exception handling output function.
 *
 * @return char const* : the output message.
 */
template <typename Data>
const char* DataException<Data>::what() const throw() {

    std::string what_msg;

    std::ostringstream stream;
    stream << "\n" << "Location : " << this->location << "\n";
    stream << "Flag     : " << Error(error_type).what() << "\n";
    if ( message.size() > 0 ) {
        stream << "Detail   : " << message << "\n";
    }
    stream << "Data     : " << error_data << "\n";
    what_msg = stream.str();
    return what_msg.c_str();
}

}; // namespace ecl

#endif /* ECL_DISABLE_EXCEPTIONS */
#endif /* ECL_EXCEPTIONS_DATA_EXCEPTION_HPP_*/
