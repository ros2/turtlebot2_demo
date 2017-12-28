/**
 * @file /ecl_devices/include/ecl/devices/serial_w32.hpp
 *
 * @brief Win32 interface for serial (RS232) devices.
 *
 * @date May 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_SERIAL_W32_HPP_
#define ECL_THREADS_SERIAL_W32_HPP_

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>
#ifdef ECL_IS_WIN32

/*****************************************************************************
** Includes
*****************************************************************************/

#include <windows.h>
#include <string>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/utilities/parameter.hpp>
#include <ecl/threads/thread.hpp>
#include "serial_parameters.hpp"
#include "traits.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [Serial]
*****************************************************************************/
/**
 * @brief Win32 implementation for a serial (RS232) device.
 *
 * This device is a c++ wrapper around the win32 api for serial (RS232 devices). It
 * configures the serial device as a non-seekable source-sink (read-write),
 * device.
 *
 * Just a quick note about the serial termios functions. They can
 * be configured in a variety of ways for a variety of
 * situations (modems, terminals, login consoles). Most of these are
 * redundant for control, so we just keep it simple here
 * and use a simple wrapper around a useful configuration for
 * control. This keeps
 * options and settings to a minimum and lets us get on with the job
 * of control.
 *
 * <b>Read Modes</b>
 *
 * In posix, there are several available read modes. As mentioned above,
 * some of these are for all intents and purposes, redundant for control.
 * One particular example is the message
 * buffering configuration that uses both a packet size limit as well
 * as an inter-byte timeout when reading. Unfortunately the minimum inter-byte
 * timeout is a massive 200ms, which makes it all but useless for
 * control.
 *
 * Subsequently, the default mode of operation is using
 * a timeout - as this one will automatically return you whenever any
 * new data comes in. Great for control! An alternative non-blocking option is
 * also provided.
 *
 * <b>Usage</b>:
 *
 * The serial device does both input and output, using the usual read/write
 * interface with your preference of RAII/non-RAII style interface. In either
 * case, the serial class does the cleanup for you in the destructor.
 *
 * Protocol configuration is done either in the constructor or the open()
 * command by passing a port name and a set of enumerated
 * protocol configuration parameters - @ref BaudRate "baud rate",
 * @ref DataBits "data bits", @ref StopBits "stop bits",
 * and @ref Parity "parity" are defined via enums.
 *
 * @code
 * Serial serial_raii("/dev/ttyS0",BaudRate_115200,DataBits_8,StopBits_1,NoParity); // RAII
 * Serial serial_nonraii;
 * serial_nonraii.open("/dev/ttyS0",BaudRate_115200,DataBits_8,StopBits_1,NoParity); // non-RAII
 * @endcode
 *
 * Reading with various timeouts.
 *
 * @code
 * int n;
 * char buffer[256];
 * serial.block(500); // timeout of 500ms
 * n = serial.read(buffer,256);
 * serial.unblock();
 * n = serial.read(buffer,256); // always returns, even if nothing is there.
 * @endcode
 *
 * Writing:
 *
 * @code
 * serial.write("Dude\n",5);
 * @endcode
 *
 * @todo Another option in future would be to utilise the asynchronous
 * configuration mode for serial ports (see
 * http://www.faqs.org/docs/Linux-HOWTO/Serial-Programming-HOWTO.html#AEN144).
 * This might get around some bottlenecking io performance problems.
 */
class ecl_devices_PUBLIC Serial {
public:
	/*********************
	** C&D
	**********************/
	/**
	 * @brief Non-RAII style constructor, doesn't make a connection.
	 *
	 * Sometimes its more convenient to open manually rather than inside
	 * constructors. Use this with the open() command to do so. You can
	 * check for open status with via the open accessor.
	 */
	Serial() : is_open(false), is_run(false), file_descriptor(INVALID_HANDLE_VALUE), error_handler(NoError) {};
	/**
	 * @brief Constructs and opens the connection, RAII style.
	 *
	 * Configures and opens the serial device (RAII style) with the specified
	 * parameters.
	 *
	 * By default, the port is configured to read in blocking mode
	 * (with no timeouts) - use the block(), block(timeout_ms) and unbock()
	 * methods to alter its behaviour.
	 * @param port_name : the device name.
	 * @param baud_rate : baud rate.
	 * @param data_bits : the number of bits in a single message byte.
	 * @param stop_bits : the number of bits after the data used for error checking.
	 * @param parity : the parity used for checksums.
	 * @exception StandardException : throws if the connection failed to open.
	 */
	Serial(const std::string& port_name, const BaudRate &baud_rate = BaudRate_115200, const DataBits &data_bits = DataBits_8,
			const StopBits &stop_bits = StopBits_1, const Parity &parity = NoParity ) ecl_throw_decl(StandardException);

	/**
	 * @brief Cleans up the file descriptor.
	 *
	 * Cleans up the file descriptor.
	 */
	virtual ~Serial();

	/*********************
	** Open/Close
	**********************/
	/**
	 * @brief Opens the connection.
	 *
	 * Configures and opens the serial device with the specified
	 * parameters. You don't need to do this if you constructed this class with the RAII
	 * style constructor.
	 *
	 * If the device is already open, it will first close and then reopen the device.
	 *
	 * @param port_name : the device name.
	 * @param baud_rate : baud rate.
	 * @param data_bits : the number of bits in a single message byte.
	 * @param stop_bits : the number of bits after the data used for error checking.
	 * @param parity : the parity used for checksums.
	 * @exception StandardException : throws if the connection failed to open.
	 */
	void open(const std::string& port_name, const BaudRate &baud_rate = BaudRate_115200, const DataBits &data_bits = DataBits_8,
			const StopBits &stop_bits = StopBits_1, const Parity &parity = NoParity ) ecl_throw_decl(StandardException);

	/**
	 * @brief Closes the port connection.
	 *
	 * The destructor automatically handles this, so in most cases its redundant.
	 * However there are times when a serial port must be temporarily closed.
	 * This enables you to do that.
	 */
	void close();
	/**
	 * @brief Status flag indicating if the serial port is open/closed.
	 *
	 * True if the serial port is open and connected, false otherwise.
	 */
	bool open() const { return is_open; }

	/*********************
	** Writing
	**********************/
	/**
	 * @brief Write a character to the serial port.
	 *
	 * Write a character to the serial port.
	 *
	 * @param c : the character to write.
	 * @exception StandardException : throws if writing returned an error [debug mode only].
	 **/
	long write(const char &c) ecl_assert_throw_decl(StandardException);

	/**
	 * @brief Write a character string to the serial port.
	 *
	 * Write a character string to the serial port.
	 * @param s : points to the beginning of the character string.
	 * @param n : the number of characters to write.
	 * @exception StandardException : throws if writing returned an error [debug mode only].
	 **/
	long write(const char *s, unsigned long n) ecl_assert_throw_decl(StandardException);

	/**
	 * @brief A dummy flush function, not used, but needed by streams.
	 *
	 * This is unused - but is included so as to implement streaming functionality
	 * on top of this device.
	 */
	void flush() {}

	/*********************
	** Reading Modes
	**********************/
	/**
	 * @brief Switch to blocking mode with a timeout for reading.
	 *
	 * Switched to blocking mode with the specified timeout in milliseconds.
	 *
	 * @param timeout : timeout measured in ms.
	 *
	 * @exception StandardException : throws if a timeout < 0 is specified [debug mode only].
	 */
	void block(const long &timeout = 500) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Switch to unbocked mode for reading.
	 *
	 * This causes the serial port to automatically return from a read, even
	 * if there is no data available.
	 */
	void unblock();

	/*********************
	** Reading
	**********************/
	/**
	 * @brief Get the number of bytes remaining in the buffer, but do not read.
	 *
	 * Check the serial port's buffer to determine how many bytes are left in the buffer for reading.
	 *
	 * @return int : the number of bytes in the buffer
	 **/
	long remaining();
	/**
	 * @brief Read a character from the port.
	 *
	 * Uses the current device configuration
	 * for reading. The blocking policy is defined by the current parameters
	 * of the port.
	 *
	 * @param c : character to read into from the serial port's buffer.
	 * @exception StandardException : throws if reading returned an error [debug mode only].
	 **/
    long read(char &c) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Read a character string from the port.
	 *
	 * Uses the current device configuration
	 * for reading. The blocking policy is defined by the current parameters
	 * of the port.
	 *
	 * @param s : character string to read into from the serial port's buffer.
	 * @param n : the number of bytes to read.
	 * @exception StandardException : throws if reading returned an error [debug mode only].
	 **/
    long read(char *s, const unsigned long &n) ecl_assert_throw_decl(StandardException);


    /*********************
	** Serial Specific
	**********************/
	/**
	 * @brief Clear both input and output buffers.
	 *
	 * Serial input and output buffers are managed by the system. This clears
	 * them both.
	 */
    void clear() {
        PurgeComm( file_descriptor, PURGE_RXCLEAR );
        PurgeComm( file_descriptor, PURGE_TXCLEAR );
    }
    /**
     * @brief Clear the input buffer.
     *
     * The serial input buffer are managed by the system. This clears it.
    */
    void clearInputBuffer() { PurgeComm( file_descriptor, PURGE_RXCLEAR ); }
    /**
     * @brief Clear the output buffer.
     *
     * The serial output buffer are managed by the system. This clears it.
    */
    void clearOutputBuffer() { PurgeComm( file_descriptor, PURGE_TXCLEAR ); }

    /**
     * @brief Return the latest error state for this serial object.
     *
     * Return the error result from the last used open/read/write
     * method in this class.
     */
    const Error& error() const { return error_handler; }

private:
	/*********************
	** Variables
	**********************/
	HANDLE file_descriptor;
	OVERLAPPED  m_osRead, m_osWrite; // Offsets
    std::string port;
    bool is_open;
    ecl::Error error_handler;
    Thread event_receiver;
    bool is_run;

private:
    /**
     * @brief Threading procedure for serial port.
     *
     * Receiving event generated by communication port.
     */
    friend void event_proc(void* arg);

};

/*****************************************************************************
** Traits [Serial]
*****************************************************************************/
/**
 * @brief Serial sink (output device) trait.
 *
 * Specialisation for the serial sink (output device) trait.
 */
template <>
class is_sink<Serial> : public True {};

/**
 * @brief Serial sink (input device) trait.
 *
 * Specialisation for the serial sink (input device) trait.
 */
template <>
class is_source<Serial> : public True {};

/**
 * @brief Serial sourcesink (input-output device) trait.
 *
 * Specialisation for the serial sourcesink (input-output device) trait.
 */
template <>
class is_sourcesink<Serial> : public True {};

} // namespace ecl

#endif /* ECL_IS_WIN32 */
#endif /* ECL_THREADS_SERIAL_W32_HPP_ */
