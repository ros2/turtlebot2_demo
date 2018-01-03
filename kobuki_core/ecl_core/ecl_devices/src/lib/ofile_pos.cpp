/**
 * @file /src/lib/ofile_pos.cpp
 *
 * @brief Posix synchronous file implementation.
 *
 * @date September 2009
 **/

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/devices/ofile_pos.hpp"
#include "../../include/ecl/devices/detail/error_handler.hpp"
#include <iostream>
#include <ecl/exceptions/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [OFile]
*****************************************************************************/

OFile::OFile() :
	file(NULL),
	error_handler(NoError)
{}

OFile::OFile(const std::string &file_name, const WriteMode &write_mode) ecl_throw_decl(StandardException) :
	file(NULL),
	error_handler(NoError)
{
	ecl_try {
		open(file_name,write_mode);
	} ecl_catch( StandardException &e ) {
		ecl_throw(StandardException(LOC,e));
	}
}

OFile::~OFile() {
	if ( open() ) {
		// This flushes and closes the file descriptor.
		if ( fclose(file) != 0 ) {
			// implement some mechanism if ever needed, but no
			// exceptions allowed in destructors, just have to assume the best.
		}
		file = NULL;
	}
}
/*****************************************************************************
** Implementation [OFile][open/close]
*****************************************************************************/

bool OFile::open(const std::string &file_name, const WriteMode &write_mode) ecl_throw_decl(StandardException) {
	name = file_name;
    switch(write_mode) {
        case(New) : {
        	file_descriptor = ::open(name.c_str(), O_WRONLY|O_CREAT, S_IWUSR|S_IRUSR|S_IRGRP|S_IROTH);
            if ( file_descriptor == -1 ) {
                ecl_throw(devices::open_exception(LOC,file_name));
                error_handler = devices::open_error();
                return false;
            }
            file = fdopen(file_descriptor,"w");
            break;
        }
        case(Append) : {
        	file_descriptor = ::open(name.c_str(),O_WRONLY|O_APPEND|O_CREAT, S_IWUSR|S_IRUSR|S_IRGRP|S_IROTH);
            if ( file_descriptor == -1 ) {
                ecl_throw(devices::open_exception(LOC,file_name));
                error_handler = devices::open_error();
                return false;
            }
            file = fdopen(file_descriptor,"a");
            break;
        }
        default : break;
    }
    if ( file == NULL ) {
        ecl_throw(devices::open_exception(LOC,file_name));
        error_handler = devices::open_error();
        return false;
    }
	error_handler = NoError;
    return true;
}

bool OFile::close() ecl_throw_decl(StandardException) {
	if ( open() ) {
		// This flushes and closes the file descriptor.
		if ( fclose(file) != 0 ) {
			ecl_throw(devices::close_exception(LOC,name));
			error_handler = devices::close_error();
			return false;
		}
		file = NULL;
	}
	error_handler = NoError;
	return true;
}
/*****************************************************************************
** Implementation [OFile][write]
*****************************************************************************/

long OFile::write(const char &c) ecl_debug_throw_decl(StandardException)
{
	if ( !open() ) {
		ecl_debug_throw(StandardException(LOC, OpenError, std::string("File ") + name + std::string(" is not open for writing.")));
		error_handler = OpenError;
    	return -1;
	}
    size_t written = fwrite(&c,1,1,file);
    if ( written <= 0 ) {
    	ecl_debug_throw(StandardException(LOC, WriteError, std::string("Could not write to ") + name + std::string(".")));
    	error_handler = WriteError;
    	return -1;
    }
	error_handler = NoError;
    // fwrite returns the number of 'items' written, not bytes!
    return written;
}

long OFile::write(const char* s, unsigned long n) ecl_debug_throw_decl(StandardException)
{
	if ( !open() ) {
		ecl_debug_throw(StandardException(LOC, OpenError, std::string("File ") + name + std::string(" is not open for writing.")));
		error_handler = OpenError;
    	return -1;
	}
	size_t written = fwrite(s,n,1,file);
    if ( written <= 0 ) {
    	ecl_debug_throw(StandardException(LOC, WriteError, std::string("Could not write to ") + name + std::string(".")));
    	error_handler = WriteError;
    	return -1;
    }
	error_handler = NoError;
    // fwrite returns the number of 'items' written, not bytes!
    return n*written;
}

bool OFile::flush() ecl_debug_throw_decl(StandardException) {
	// This flushes userland buffers to the kernel buffers...not sure why,
	// but you can still read the file in real time, so its good enough and
	// thus better than a more expensive fsync here.
	int result = fflush(file);
	if ( result != 0 ) {
		ecl_debug_throw ( StandardException(LOC, UnknownError, std::string("Could not fflush ") + name + std::string(".")));
		error_handler = UnknownError;
		return false;
	}
	error_handler = NoError;
	return true;
}

}; // namespace ecl

#endif /* ECL_IS_POSIX */
