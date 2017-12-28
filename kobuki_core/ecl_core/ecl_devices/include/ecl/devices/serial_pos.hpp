/**
 * @file /include/ecl/devices/serial_pos.hpp
 *
 * @brief Posix interface for serial (RS232) devices.
 *
 * @date September 2009
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ECL_DEVICES_SERIAL_POS_HPP_
#define ECL_DEVICES_SERIAL_POS_HPP_

/*****************************************************************************
 ** Platform Check
 *****************************************************************************/

#include <ecl/config.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <termios.h>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/errors/compile_time_assert.hpp>
#include <ecl/time/snooze.hpp>
#include <ecl/utilities/parameter.hpp>
#include <ecl/type_traits/fundamental_types.hpp>
#include "detail/error_handler.hpp"
#include "detail/exception_handler_pos.hpp"
#include "serial_parameters.hpp"
#include "traits.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl
{

  /*****************************************************************************
   ** Interface [Serial]
   *****************************************************************************/
  /**
   * @brief Posix implementation for a serial (RS232) device.
   *
   * This device is a c++ wrapper around the posix termios functions. It
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
   * unsigned char buffer[256];
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
   * <b>Concepts</b>:
   *
   * The serial device handles reading and writing for all byte types, so it
   * supports both the InputOutputCharDevice and InputOutputByteDevice concepts
   * (refer to ecl_concepts).
   *
   * <b>Notes on Serial Port Configuration</b>:
   *
   * It's configuration is customised for robot control type devices. Given
   * the variation in configuration that can be set for serial devices, a list
   * of assumptions about the device is given below:
   *
   * - it is locked for the one process, if already locked it will throw.
   * - flow control is disabled.
   * - starts in blocking mode with timeouts, but can be switched.
   * - buffers are cleared on opening.
   */
  class Serial
  {
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
    Serial() : is_open(false), error_handler(NoError)
    {};
    /**
     * @brief Constructs and opens the connection, RAII style.
     *
     * Configures and opens the serial device (RAII style) with the specified
     * parameters.
     *
     * By default, the port is configured to read in blocking mode
     * (with overly large timeout of 5s) - use the block(timeout_ms) and unblock()
     * methods to alter its behaviour.
     *
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
     * This function will either throw, or return false and set the error() flag
     * if exceptions are disabled. Error flag values for this function include:
     *
     * - InvalidArgError
     * - PermissionsError
     * - OutOfResourcesError
     * - InterruptedError
     * - MemoryError
     * - UsageError
     * - SystemFailureError
     * - InvalidObjectError
     * - IsLockedError
     *
     * If successful, it will configure the port with a blocking policy and a
     * timeout of 5s. Use the block() method after opening to alter the behaviour.
     *
     * @sa devices::open_error, devices::open_exception
     *
     * @param port_name : the device name.
     * @param baud_rate : baud rate.
     * @param data_bits : the number of bits in a single message byte.
     * @param stop_bits : the number of bits after the data used for error checking.
     * @param parity : the parity used for checksums.
     * @exception StandardException : throws if the connection failed to open.
     */
    bool open(const std::string& port_name, const BaudRate &baud_rate = BaudRate_115200, const DataBits &data_bits = DataBits_8,
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
     * This goes beyond just maintaining some internal state, it will also
     * check if the device file (e.g. /dev/ttyUSB0) is still available and
     * has access permissions for the user. If it's not, it will automatically
     * close the serial device.
     */
    bool open();

    /*********************
     ** Writing
     **********************/
    /**
     * @brief Write a character to the serial port.
     *
     * This function gives a compile time error if the type is not a byte (refer to ecl_mpl's)
     * is_byte type traits.
     *
     * @param byte : the character to write.
     * @exception StandardException : throws reading returned an error [debug mode only].
     **/
    template <typename Byte>
    long write(const Byte &byte) ecl_debug_throw_decl(StandardException);

    /**
     * @brief Write a character string to the serial port.
     *
     * This function gives a compile time error if the type is not a byte (refer to ecl_mpl's)
     * is_byte type traits.
     *
     * @param bytes : points to the beginning of the character string.
     * @param n : the number of characters to write.
     * @exception StandardException : throws reading returned an error [debug mode only].
     **/
    template <typename Byte>
    long write(const Byte *bytes, const unsigned long &n) ecl_debug_throw_decl(StandardException);

    /**
     * @brief A dummy flush function, not used, but needed by streams.
     *
     * This is unused - but is included so as to implement streaming functionality
     * on top of this device.
     */
    void flush()
    {}

    /*********************
     ** Reading Modes
     **********************/
    /**
     * @brief Switch to blocking mode with a timeout for reading.
     *
     * Switched to blocking mode with the specified timeout in milliseconds.
     * Note that there is a valid minimum value set by termios for serial
     * timeouts - 100ms. We go beyond this though - if a timeout < 100ms is set
     * this serial class will implement its own blocking loop with the specified
     * timeout. If the timeout is > 100ms, then it will default to typical termios
     * behaviour.
     *
     * Low latency blocking has its own looping logic (keeps the interface simple):
     *
     *  - <5ms  : loops at 1ms intervals.
     *  - <20ms : loops at 2ms intervals.
     *  - 20-100ms : loops at 5ms intervals.
     *
     * @param timeout : timeout measured in ms.
     */
    void block(const unsigned long &timeout = 500);
    /**
     * @brief Switch to unblocked mode for reading.
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
     * This function gives a compile time error if the type is not a byte (refer to ecl_mpl's)
     * is_byte type traits.
     *
     * @param byte : character to read into from the serial port's buffer.
     * @exception StandardException : throws reading returned an error [debug mode only].
     **/
    template <typename Byte>
    long read(Byte &byte) ecl_debug_throw_decl(StandardException);
    /**
     * @brief Read a character string from the port.
     *
     * Uses the current device configuration
     * for reading. The blocking policy is defined by the current parameters
     * of the port.
     *
     * This function gives a compile time error if the type is not a byte (refer to ecl_mpl's)
     * is_byte type traits.
     *
     * @param bytes : character string to read into from the serial port's buffer.
     * @param n : the number of bytes to read.
     * @exception StandardException : throws reading returned an error [debug mode only].
     **/
    template <typename Byte>
    long read(Byte *bytes, const unsigned long &n) ecl_debug_throw_decl(StandardException);

    /*********************
     ** Serial Specific
     **********************/
    /**
     * @brief Clear both input and output buffers.
     *
     * Serial input and output buffers are managed by the system. This clears
     * them both.
     */
    void clear()
    { tcflush(file_descriptor,TCIOFLUSH);}
    /**
     * @brief Clear the input buffer.
     *
     * The serial input buffer are managed by the system. This clears it.
     */
    void clearInputBuffer()
    { tcflush(file_descriptor,TCIFLUSH);}
    /**
     * @brief Clear the output buffer.
     *
     * The serial output buffer are managed by the system. This clears it.
     */
    void clearOutputBuffer()
    { tcflush(file_descriptor,TCOFLUSH);}

    /**
     * @brief Return the latest error state for this serial object.
     *
     * Return the error result from the last used open/read/write
     * method in this class.
     */
    const Error& error() const
    { return error_handler;}
  private:
    /*********************
     ** Constants
     **********************/
    enum {
      NonBlocking = -1
    };
    /*********************
     ** Variables
     **********************/
    int file_descriptor;
    termios options;
    std::string port;
    unsigned long read_timeout_ms;
    ecl::Snooze fake_snooze;
    unsigned int fake_loop_count;
    bool is_open;
    ecl::Error error_handler;
  };

  /*****************************************************************************
   ** Template Implementations
   *****************************************************************************/

  template <typename Byte>
  long Serial::write(const Byte &byte) ecl_debug_throw_decl(StandardException)
  {
    ecl_compile_time_assert( is_byte<Byte>::value );
    if ( !is_open )  // internal check only, don't worry about doing the full device filename check here (we need speed)
    {
      ecl_debug_throw( StandardException(LOC, OpenError, std::string("Port ") + port + std::string(" is not open.")));
      error_handler = OpenError;
      return -1;
    }
    ssize_t no_written = ::write(file_descriptor, &byte, 1);
    if ( no_written < 0 )
    {
      ecl_debug_throw(devices::write_exception(LOC));
      error_handler = devices::write_error();
      return -1;
    }
    error_handler = NoError;
    return no_written;
  }

  template <typename Byte>
  long Serial::write(const Byte *bytes, const unsigned long &n) ecl_debug_throw_decl(StandardException)
  {
    ecl_compile_time_assert( is_byte<Byte>::value );
    if ( !is_open )  // internal check only, don't worry about doing the full device filename check here (we need speed)
    {
      ecl_debug_throw( StandardException(LOC, OpenError, std::string("Port ") + port + std::string(" is not open.")));
      error_handler = OpenError;
      return -1;
    }
    ssize_t no_written = ::write(file_descriptor, bytes, n);
    if ( no_written < 0 )
    {
      ecl_debug_throw(devices::write_exception(LOC));
      error_handler = devices::write_error();
      return -1;
    }
    error_handler = NoError;
    return no_written;
  }

  template <typename Byte>
  long Serial::read(Byte &byte) ecl_debug_throw_decl(StandardException)
  {
    ecl_compile_time_assert( is_byte<Byte>::value );
    if ( !is_open )  // internal check only, don't worry about doing the full device filename check here (we need speed)
    {
      ecl_debug_throw( StandardException(LOC, OpenError, std::string("Port ") + port + std::string(" is not open.")));
      error_handler = OpenError;
      return -1;
    }
    ssize_t no_read;
    if ( ( read_timeout_ms != NonBlocking ) && ( read_timeout_ms < 100 ) )
    {
      fake_snooze.initialise();
      for ( unsigned int i = 0; i < fake_loop_count; ++i )
      {
        no_read = ::read(file_descriptor, &byte, 1);
        if ( no_read != 0 )
        {
          break;
        }
        fake_snooze();
      }
    }
    else
    {
      no_read = ::read(file_descriptor, &byte, 1);
    }
    if ( no_read < 0 )
    {
      ecl_debug_throw(devices::read_exception(LOC));
      error_handler = devices::read_error();
      return -1;
    }
    error_handler = NoError;
    return no_read;

  }

  template <typename Byte>
  long Serial::read(Byte *bytes, const unsigned long &n) ecl_debug_throw_decl(StandardException)
  {
    ecl_compile_time_assert( is_byte<Byte>::value );
    if ( !is_open )  // internal check only, don't worry about doing the full device filename check here (we need speed)
    {
      ecl_debug_throw( StandardException(LOC, OpenError, std::string("Port ") + port + std::string(" is not open.")));
      error_handler = OpenError;
      return -1;
    }
    ssize_t no_read = 0;
    if ( ( read_timeout_ms != NonBlocking ) && ( read_timeout_ms < 100 ) )
    {
      fake_snooze.initialise();
      for ( unsigned int i = 0; i < fake_loop_count; ++i )
      {
        no_read = ::read(file_descriptor, bytes, n);
        if ( no_read != 0 )
        {
          break;
        }
        fake_snooze();
      }
    }
    else
    {
      no_read = ::read(file_descriptor, bytes, n);
    }
    if ( no_read < 0 )
    {
      ecl_debug_throw(devices::read_exception(LOC));
      error_handler = devices::read_error();
      return -1;
    }
    error_handler = NoError;
    return no_read;
  }

  /*****************************************************************************
   ** Traits [Serial]
   *****************************************************************************/
  /**
   * @brief Serial sink (output device) trait.
   *
   * Specialisation for the serial sink (output device) trait.
   */
  template <>
  class is_sink<Serial> : public True
  {};

  /**
   * @brief Serial sink (input device) trait.
   *
   * Specialisation for the serial sink (input device) trait.
   */
  template <>
  class is_source<Serial> : public True
  {};

  /**
   * @brief Serial sourcesink (input-output device) trait.
   *
   * Specialisation for the serial sourcesink (input-output device) trait.
   */
  template <>
  class is_sourcesink<Serial> : public True
  {};

} // namespace ecl

#endif /* ECL_IS_POSIX */
#endif /* ECL_DEVICES_SERIAL_POS_HPP_ */
