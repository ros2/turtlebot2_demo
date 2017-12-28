/**
 * @file /src/lib/shared_file_pos.cpp
 *
 * @brief Posix shared file implementation.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/
// Only because there is only ofile_pos support so far.

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <map>
#include <string>
#include <ecl/errors/handlers.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/exceptions/macros.hpp>
#include "../../include/ecl/devices/shared_file.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace devices {

/****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::WriteMode;
using ecl::CloseError;
using ecl::StandardException;
using ecl::Mutex;

/*****************************************************************************
** Implementation [SharedFileCommon]
*****************************************************************************/

SharedFileCommon::SharedFileCommon(const std::string &name, ecl::WriteMode mode) ecl_throw_decl(StandardException) :
	count(1),
	error_handler(NoError)
{
	ecl_try {
		if ( !file.open(name,mode) ) {
			error_handler = file.error();
		}
	} ecl_catch( StandardException &e ) {
		error_handler = file.error();
		ecl_throw(StandardException(LOC,e));
	}
}

/*****************************************************************************
** Static Variable Initialisation [SharedFileManager]
*****************************************************************************/

Mutex SharedFileManager::mutex;
std::map<string,SharedFileCommon*> SharedFileManager::opened_files;

/*****************************************************************************
** Implementation [SharedFileManager]
*****************************************************************************/

SharedFileCommon* SharedFileManager::RegisterSharedFile(const std::string& name, ecl::WriteMode mode) ecl_throw_decl(StandardException) {

	mutex.lock();
	std::map<std::string,SharedFileCommon*>::iterator iter = opened_files.find(name);
	SharedFileCommon* shared_instance;
	if ( iter != opened_files.end() ) {
        /******************************************
        ** File exists - do not open
        *******************************************/
        iter->second->count += 1;
        shared_instance = iter->second;
    } else {
        /******************************************
        ** File does not exist - open it
        *******************************************/
    	ecl_try {
    		shared_instance = new SharedFileCommon(name,mode);
    		opened_files.insert(std::pair<string,SharedFileCommon*>(name,shared_instance));
    	} ecl_catch ( StandardException &e ) {
    		shared_instance = NULL;
    		ecl_throw(StandardException(LOC,e));
    	}
    }
    mutex.unlock();
    return shared_instance;
}
/**
 * Don't have to worry about this too much, its only called when closing.
 * @param name
 * @return
 */
bool SharedFileManager::DeRegisterSharedFile(const std::string& name) ecl_throw_decl(StandardException) {

	mutex.lock();
	std::map<std::string,SharedFileCommon*>::iterator iter = opened_files.find(name);

	if ( iter == opened_files.end() ) {
		ecl_throw(StandardException(LOC,CloseError,"The specified shared object file could not be closed - was not found."));
		return false;
	}
    if ( iter->second->count == 1 ) {
    	delete iter->second;
        opened_files.erase(iter);
    } else {
        iter->second->count -= 1;
    }
    mutex.unlock();
    return true;
}

}; // namespace Interfaces

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;

/*****************************************************************************
** Implementation [SharedFile]
*****************************************************************************/

SharedFile::SharedFile(const std::string &name, WriteMode mode) ecl_throw_decl(StandardException) :
	shared_instance(NULL)
{
	ecl_try {
		open(name,mode);
	} ecl_catch( StandardException &e ) {
		ecl_throw(StandardException(LOC,e));
	}
}

SharedFile::~SharedFile() {
	ecl_try {
		devices::SharedFileManager::DeRegisterSharedFile( shared_instance->file.filename() );
	} ecl_catch( StandardException &e ) {
		// Never throw from a destructor!
		// use some other mechanism!!!
		// throw StandardException(LOC,e);
	}
}

bool SharedFile::open(const std::string &name, WriteMode mode) ecl_throw_decl(StandardException) {
	ecl_try {
		shared_instance = devices::SharedFileManager::RegisterSharedFile(name,mode);
		if ( shared_instance == NULL ) {
			shared_instance->error_handler = OpenError;
			return false;
		} else {
			shared_instance->error_handler = NoError;
			return true;
		}
	} ecl_catch ( StandardException &e ) {
		shared_instance->error_handler = OpenError;
		ecl_throw(StandardException(LOC,e));
	}
}

long SharedFile::write(const char &c) ecl_debug_throw_decl(StandardException) {
	long n = buffer.append(c);
	if ( buffer.full() ) {
		if ( !flush() ) {
			return -1;
		}
	}
	return n;
}

long SharedFile::write(const char* s, unsigned long n) ecl_debug_throw_decl(StandardException) {
	unsigned int no_written = 0;
	while ( no_written < n ) {
		no_written += buffer.append(s+no_written,n-no_written);
		if ( buffer.full() ) {
			if ( !flush() ) {
				return -1;
			}
		}
	}
	return n;
}

bool SharedFile::flush() ecl_debug_throw_decl(StandardException) {
	long written;
	ecl_debug_try {
		written = shared_instance->file.write(buffer.c_ptr(), buffer.size() );
	} ecl_debug_catch(const StandardException &e) {
		shared_instance->error_handler = shared_instance->file.error();
		ecl_debug_throw(StandardException(LOC,e));
	}
	buffer.clear();
	// fallback for no exceptions
	shared_instance->error_handler = shared_instance->file.error();
	if ( written > 0 ) {
		return true;
	} else {
		return false;
	}
}


}; // namespace ecl
