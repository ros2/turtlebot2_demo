/**
 * @file /ecl_devices/src/lib/serial_w32.cpp
 *
 * @brief Win32 implementation for serial ports.
 *
 * @date May 2010
 **/

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>
#ifdef ECL_IS_WIN32

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/devices/serial_w32.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [Serial][C&D]
*****************************************************************************/

Serial::Serial(const std::string& port_name, const BaudRate &baud_rate, const DataBits &data_bits,
		const StopBits &stop_bits, const Parity &parity ) throw(StandardException) :
				port(port_name), is_run(false), file_descriptor(INVALID_HANDLE_VALUE)
{
	try {
		open(port_name, baud_rate, data_bits, stop_bits, parity);
	} catch ( StandardException &e ) {
		throw StandardException(LOC,e);
	}
}

Serial::~Serial() {
	close();
}

/*****************************************************************************
** Implementation [Serial][Open]
*****************************************************************************/

void Serial::open(const std::string& port_name, const BaudRate &baud_rate, const DataBits &data_bits,
		const StopBits &stop_bits, const Parity &parity ) throw(StandardException) {

	if ( open() ) {
		close();
	}

	if (strstr(port_name.c_str(), "\\\\.\\"))
		port = port_name;
	else
		port = std::string("\\\\.\\") + port_name;

    /******************************************
     * Reset Timeouts
     ******************************************/
    m_osRead.Offset = 0;
    m_osRead.OffsetHigh = 0;
    if (! (m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL))) {
    	throw StandardException(LOC, OpenError, "Serial port failed to open - could not configure offsets.");
    }
    m_osWrite.Offset = 0;
    m_osWrite.OffsetHigh = 0;
    if (! (m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL))) {
    	throw StandardException(LOC, OpenError, "Serial port failed to open - could not configure offsets.");
    }

    /******************************************
     * Open the port handle
     ******************************************/
    /*
     * CreateFileA is the ascii function. CreateFileW is the unicode function.
     * FILE_FLAG_OVERLAPPED - do not block operations (use events to return)
     * If the above flag is not specified, it defaults to non-overlapped mode.
     * When in non-overlapped mode you can't sit in a thread trying to read the
     * port and simultaneously try to write from another.
     */
    file_descriptor = CreateFileA( port.c_str(),
                            GENERIC_READ | GENERIC_WRITE,
                            0,
                            NULL,
                            OPEN_EXISTING, // Serial ports already exist
                            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                            NULL);

    /******************************************
     * Open - error handling
     ******************************************/
    if (file_descriptor == INVALID_HANDLE_VALUE ) {
        int error = GetLastError();
        switch (error) {
            case (ERROR_PATH_NOT_FOUND) : {
                throw StandardException(LOC,NotFoundError);
                break;
            }
            case (ERROR_FILE_NOT_FOUND) : {
                throw StandardException(LOC,NotFoundError);
                break;
            }
            default : {
                throw StandardException(LOC,OpenError);
            }
        }
    }

    /******************************************
	** Configuration
	*******************************************/
    static const int baud_rate_flags[] = { CBR_110, CBR_300, CBR_600, CBR_1200, CBR_2400, CBR_4800, CBR_9600, CBR_19200, CBR_38400, CBR_57600, CBR_115200 };
    static const int stop_bits_flags[] = { ONESTOPBIT, TWOSTOPBITS }; // Windoze has a ONE5STOPBITS as well
    static const int parity_types_flags[] = { NOPARITY, ODDPARITY, EVENPARITY }; // There are others but we dont need
    static const int data_bits_flags[] = { 5, 6, 7, 8 };

    /******************************************
     * Initialise events to monitor
     ******************************************/
    // EV_RXCHAR : A character was received and placed in the input buffer.
    SetCommMask( file_descriptor, EV_RXCHAR );
	
    /******************************************
     * Initialise the input/output buffer's size
     ******************************************/
    SetupComm( file_descriptor, 2048, 2048 ); // 4096,4096

    /******************************************
     * Clear
     ******************************************/
    PurgeComm( file_descriptor, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR );

	/******************************************
	* Dont care about write timeouts (could be dangerous!?)
	******************************************/
	COMMTIMEOUTS    timeouts;
	GetCommTimeouts( file_descriptor, &timeouts );
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts( file_descriptor, &timeouts );

	/******************************************
	* Setup the Configuration Structure
	******************************************/
	DCB dcb;

	FillMemory(&dcb, sizeof(dcb), 0);

	// MS recommends just grabbing the current state and modifying the required parameters on the
	// off chance the structure may change in the future.
	if ( !GetCommState( file_descriptor, &dcb) ) {
	 throw StandardException(LOC,ConfigurationError,"There was an error retrieving the state of the port.");
	};

	/******************************************
	* Protocol Configuration
	******************************************/
	dcb.DCBlength = sizeof(dcb);
	dcb.BaudRate = baud_rate_flags[baud_rate];
	dcb.ByteSize = data_bits_flags[data_bits];
	dcb.Parity = parity_types_flags[parity];
	dcb.StopBits = stop_bits_flags[stop_bits];
	dcb.fNull= FALSE; // If true, discard null bytes if they are received

	// dcb.fAbortOnError = TRUE; // Need?

	/******************************************
	* Flow Control
	******************************************/
	dcb.fInX = FALSE; // XON/XOFF flow control during reception.
	dcb.fOutX = FALSE; // XON/XOFF flow control during transmission.
	dcb.fDtrControl = DTR_CONTROL_DISABLE; // Data-Terminal-Ready flow control
	dcb.fRtsControl = RTS_CONTROL_DISABLE; // Request-To-Send flow control.

	dcb.fOutxCtsFlow = FALSE; // don't monitor the Clear-To-Send output flow control signal.
	dcb.fOutxDsrFlow = FALSE; // don't monitor the Data-Set-Ready output flow control signal.

	// dcb.XonLim = 1000; // Number of bytes in the input buffer before flow control is activated to inhibit the sender.
	// dcb.XoffLim = 1000; // Similar situation as above...
	// dcb.XonChar = ASCII_XON;
	// dcb.XoffChar = ASCII_XOFF;

    /******************************************
     * Save and Handle Errors
     ******************************************/
    if (!SetCommState( file_descriptor, &dcb)) {
        int error = GetLastError();
        switch ( error ) {
            case ( ERROR_INVALID_PARAMETER ) : {
                throw StandardException(LOC,ConfigurationError,"A parameter was configured incorrectly (ERROR_INVALID_PARAMETER).");
                break;
            }
            default: {
                throw StandardException(LOC,ConfigurationError);
                break;
            }
        }
    }

	/******************************************
	* Prepare thread for receiving event generated by communication port
	******************************************/
    is_run = true;
	event_receiver.cancel();
    event_receiver.start(generateFunctionObject(event_proc, this));

    block(5000);
    is_open = true;
}

void Serial::close() {
	if ( open() ) {

		// release thread for receiving event
		is_run = false;
		event_receiver.join();
		event_receiver.cancel();

		if (file_descriptor != INVALID_HANDLE_VALUE) {
			// These return values, but assume it works ok, not really critical.
			SetCommMask(file_descriptor, 0);
			PurgeComm(file_descriptor, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR );
			CloseHandle(file_descriptor);
			file_descriptor = INVALID_HANDLE_VALUE;
		}

	    is_open = false;
	}
}

/*****************************************************************************
** Implementation [Serial][Threading Procedure]
*****************************************************************************/
void event_proc(void* arg) {
	Serial* owner = (Serial*)arg;
	if (!owner)
		return;
	while (owner->is_run) {
		DWORD mask;
		if (!WaitCommEvent(owner->file_descriptor, &mask, NULL)) {
			if (0x000003e3 == GetLastError()) {
				break;
			}
		}
	}
	owner->file_descriptor = INVALID_HANDLE_VALUE;
	owner->is_open = false;
}

/*****************************************************************************
** Implementation [Serial][Writing]
*****************************************************************************/

long Serial::write(const char &c) ecl_assert_throw_decl(StandardException) {
	return write(&c, 1);
}

long Serial::write(const char *s, unsigned long n) ecl_assert_throw_decl(StandardException) {
	DWORD   written, error, error_flags;
	COMSTAT comstat;
	int    result;

	result = WriteFile( file_descriptor, s, n, &written, &m_osWrite);

	if (!result) {
		if (GetLastError() == ERROR_IO_PENDING) {
			while (!GetOverlappedResult(file_descriptor, &m_osWrite, &written, TRUE)) {
				error = GetLastError();
				if (error != ERROR_IO_INCOMPLETE) {
					ClearCommError( file_descriptor, &error_flags, &comstat);
					break;
				}
			}
		} else {
			written = 0;
			ClearCommError(file_descriptor, &error_flags, &comstat);
		}
	}
	ecl_assert_throw( written != 0, StandardException(LOC,WriteError) );
	return written;
}

/*****************************************************************************
** Implementation [Serial][Reading Modes]
*****************************************************************************/

void Serial::block(const long &timeout) ecl_assert_throw_decl(StandardException) {

	ecl_assert_throw( timeout >= 0, StandardException(LOC, InvalidInputError, "Serial port timeouts must be greater than 0ms.") );
    COMMTIMEOUTS    timeouts;
    GetCommTimeouts( file_descriptor, &timeouts );

    // See http://msdn2.microsoft.com/en-us/library/ms885171.aspx for details
    // about this combination of settings.
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = static_cast<DWORD>(timeout);
    SetCommTimeouts( file_descriptor, &timeouts );
}

void Serial::unblock() {
    COMMTIMEOUTS    timeouts;
    GetCommTimeouts( file_descriptor, &timeouts );

    // See http://msdn2.microsoft.com/en-us/library/ms885171.aspx for details
    // about this combination of settings.
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    SetCommTimeouts( file_descriptor, &timeouts );
}

/*****************************************************************************
** Implementation [Serial][Reading]
*****************************************************************************/
long Serial::remaining() {
    unsigned long error;
    _COMSTAT status;
    if ( ClearCommError(file_descriptor,&error,&status) ) {
        return status.cbInQue;
    } else {
        return -1;
    }
}

long Serial::read(char &c) ecl_assert_throw_decl(StandardException) {
	return read(&c,1);
}

long Serial::read(char *s, const unsigned long &n) ecl_assert_throw_decl(StandardException)
{
    COMSTAT comstat;
    DWORD   read=0, error, error_flags;
    DWORD dwRes;

    /*********************
    ** Windows Hack
    **********************/
    // If its blocking for a minimum number of characters,
    /*********************
    ** Reading Serial
    **********************/
    if (file_descriptor == INVALID_HANDLE_VALUE) {
    	return 0;
    }

    if (!ReadFile( file_descriptor, s, n, &read, &m_osRead) ) {
        error = GetLastError();

        if( error != ERROR_IO_PENDING ) {
        	if (error != ERROR_ACCESS_DENIED)
        		ClearCommError(file_descriptor, &error_flags, &comstat);

            return 0;
        } else {
            dwRes = WaitForSingleObject( m_osRead.hEvent, INFINITE );

            switch( dwRes ) {
            case WAIT_OBJECT_0:
                if( !GetOverlappedResult( file_descriptor, &m_osRead, &read, FALSE) ) {
                    ClearCommError(file_descriptor, &error_flags, &comstat);
                    return 0;
                } else {
                    ClearCommError(file_descriptor, &error_flags, &comstat);
                    return read;
                }
                break;
            default:
                {
                    ClearCommError(file_descriptor, &error_flags, &comstat);
                    return 0;
                }
                break;
            }
        }
    }
    return read;
}
} // namespace ecl

#endif /* ECL_IS_WIN32 */
