/**
 * @file /src/lib/console.cpp
 *
 * @brief Standard input-output device implementation.
 *
 * @date October 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdio>
#include <string>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/devices/console.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [OConsole]
*****************************************************************************/

long OConsole::write(const char &c) ecl_assert_throw_decl(StandardException)
{
	long n = buffer.append(c);
	if ( buffer.full() ) {
		flush();
	}
	return n;
}

long OConsole::write(const char* s, unsigned long n) ecl_assert_throw_decl(StandardException)
{
	unsigned int no_written = 0;
	while ( no_written < n ) {
		no_written += buffer.append(s+no_written,n-no_written);
		if ( buffer.full() ) {
			flush();
		}
	}
	return n;
}

void OConsole::flush() ecl_assert_throw_decl(StandardException) {
    fputs(buffer.c_str(),stdout);
    buffer.clear();
	int result = fflush(stdout);
	ecl_assert_throw ( result == 0, StandardException(LOC, WriteError, std::string("Could not flush to the standard output device.")));
}

/*****************************************************************************
** Implementation [EConsole]
*****************************************************************************/

long EConsole::write(const char &c) ecl_assert_throw_decl(StandardException)
{
	long n = buffer.append(c);
	if ( buffer.full() ) {
		flush();
	}
	return n;
}

long EConsole::write(const char* s, unsigned long n) ecl_assert_throw_decl(StandardException)
{
	unsigned int no_written = 0;
	while ( no_written < n ) {
		no_written += buffer.append(s+no_written,n-no_written);
		if ( buffer.full() ) {
			flush();
		}
	}
	return n;
}

void EConsole::flush() ecl_assert_throw_decl(StandardException) {
    fputs(buffer.c_str(),stderr);
    buffer.clear();
	int result = fflush(stderr);
	ecl_assert_throw ( result == 0, StandardException(LOC, WriteError, std::string("Could not flush to the standard output device.")));
}

/*****************************************************************************
** Implementation [IConsole]
*****************************************************************************/

long IConsole::read(char &c) ecl_assert_throw_decl(StandardException)
{
	// fgets is a problem, it doesn't read the newline with the character,
	// which means the next read from stdin will catch that newline first.
    c = static_cast<char>(fgetc(stdin));
    if ( c == EOF ) {
    	ecl_assert_throw( c != EOF, StandardException(LOC, ReadError, "Failed to read from standard input."));
    	return 0; // fallover if not in debug mode.
    } else {
//    	while ( fgetc(stdin) != '\n' ) {}
    	return 1;
    }
}

long IConsole::read(char* s, const unsigned long &n) ecl_assert_throw_decl(StandardException)
{
    char *result = fgets(s,n,stdin);
    if ( result == NULL ) {
        ecl_debug_throw( StandardException(LOC, ReadError, "Failed to read from standard input."));
    	return 0; // Fallover if not in debug mode.
    } else {
        // fgets always terminates with a null character, even if it manages
        // to read n-1 chars. We also need to make sure we drop the newline
    	// character that was returned to trigger the input.
    	size_t length = strlen(s); // This doesn't count the \0 character.
//    	*(s+length-1) = '\0'; // drop the newline
//    	return length-1;
    	return length;
    }
}

} // namespace ecl
