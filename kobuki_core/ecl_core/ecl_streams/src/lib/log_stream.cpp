/**
 * @file /src/lib/log_stream.cpp
 *
 * @brief Customised logging implementation.
 *
 * @date December 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <utility>
#include "../../include/ecl/streams/log_stream.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;

/*****************************************************************************
** Implementation [LogStream]
*****************************************************************************/

void LogStream::enableHeader() { write_header = true; }
void LogStream::disableHeader() { write_header = false; }
void LogStream::enableTimeStamp() { write_stamp = true; }
void LogStream::disableTimeStamp() { write_stamp = false; }

void LogStream::enableMode(int mode, std::string header) { modes.insert( std::make_pair(mode,header) ); }
void LogStream::disableMode(int mode) { modes.erase(mode); }

bool LogStream::isEnabled() {
    if ( modes.size() > 0 ) {
        return true;
    } else {
        return false;
    }
}

bool LogStream::isModeEnabled(int mode) {

    if( modes.count(mode) > 0 ) {
        return true;
    } else {
        return false;
    }
}

LogStream& LogStream::log(int mode) {
    if ( write_stamp ) {
        (*this) << timestamp.stamp() << " ";
    }
    if ( write_header ) {
        (*this) << "[" << modes[mode] << "] ";
    }
    if ( write_stamp || write_header ) {
    	(*this) << ": ";
    }
    return (*this);
}

} // namespace ecl

