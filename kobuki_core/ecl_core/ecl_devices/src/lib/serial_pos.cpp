/**
 * @file /src/lib/serial_pos.cpp
 *
 * @brief Posix implementation for serial ports.
 *
 * @date September 2009
 **/

/*****************************************************************************
 ** Platform Check
 *****************************************************************************/

#include <ecl/config/ecl.hpp>
#ifdef ECL_IS_POSIX

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <cstdlib> // div
#include <unistd.h> // access
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/exceptions/macros.hpp>
#include "../../include/ecl/devices/serial_pos.hpp"
#include "../../include/ecl/devices/detail/error_handler.hpp"

/*****************************************************************************
 ** Platform Checks
 *****************************************************************************/

#include <ecl/config.hpp>
// B921600 baud rate macro is not part of the posix specification (see
// http://digilander.libero.it/robang/rubrica/serial.htm#3_1_1). However
// FreeBSD has had it for at least four years and linux quite some time too.
// Apple Macs however do not currently have it, so we help it here.
#if defined(ECL_IS_APPLE)
  #ifndef B460800
    #define B460800 460800
  #endif
  #ifndef B921600
    #define B921600 921600
  #endif
#endif

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl
{

/*****************************************************************************
 ** Using
 *****************************************************************************/

using std::string;
using ecl::InvalidInputError;
using ecl::OpenError;
using ecl::StandardException;

/*****************************************************************************
 ** Implementation [Serial][C&D]
 *****************************************************************************/

Serial::Serial(const std::string& port_name, const BaudRate &baud_rate, const DataBits &data_bits,
               const StopBits &stop_bits, const Parity &parity) ecl_throw_decl(StandardException) :
    port(port_name), read_timeout_ms(5000), error_handler(NoError)
{
  ecl_try
  {
    open(port_name, baud_rate, data_bits, stop_bits, parity);
  } ecl_catch( const StandardException &e ) {
    ecl_throw(StandardException(LOC,e));
  }
}

Serial::~Serial()
{
  close();
}

/*****************************************************************************
 ** Implementation [Serial][Open]
 *****************************************************************************/

bool Serial::open(const std::string& port_name, const BaudRate &baud_rate, const DataBits &data_bits,
                  const StopBits &stop_bits, const Parity &parity) ecl_throw_decl(StandardException)
{

  /*********************
   ** Input Checks
   **********************/
  if (stop_bits == StopBits_15)
  {
    ecl_throw(
        StandardException(LOC,ConfigurationError,"Standard serial device does not accept StopBits_15 as valid (used in ftdi)."));
    error_handler = InvalidArgError;
    is_open = false;
    return false;
  }

  if (open())
  {
    close();
  }
  port = port_name;
  // One curious thing here is O_NONBLOCK. If you do not specify this, it will try
  // and wait till the line has an 'other' end. Which it can't possibly know, so
  // it will effectively block here forever.
  file_descriptor = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (file_descriptor == -1)
  {
    ecl_throw(devices::open_exception(LOC,port_name));
    error_handler = devices::open_error();
    is_open = false;
    return false;
  }

  static const int baud_rate_flags[] = {B110, B300, B600, B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200,
                                        B230400, B460800, B921600};
  if (baud_rate >= (sizeof(baud_rate_flags) / sizeof(B110)))
  {
    ecl_throw(StandardException(LOC,ConfigurationError,"Selected baudrate is not supported."));
    error_handler = InvalidArgError;
    is_open = false;
    return false;
  }
  static const int data_bits_flags[] = {CS5, CS6, CS7, CS8};

  /*********************
   ** Current Settings
   **********************/
//    tcgetattr(file_descriptor,&options); // Need?
  /******************************************
   ** Flow Control
   *******************************************/
  // Turn off the O_NONBLOCK we set back when opening (we want by default
  // blocking and timeouts etc to occur).
  // Using 0 as the argument here resets the file status flags to defaults.
  fcntl(file_descriptor, F_SETFL, 0);
  // this came from ros' hokoyu node code for setting a lock on the device file descriptor
  // useful if you want to make sure other devices dont mess with your device!
  struct flock file_lock;
  file_lock.l_type = F_WRLCK;
  file_lock.l_whence = SEEK_SET;
  file_lock.l_start = 0;
  file_lock.l_len = 0;
  file_lock.l_pid = getpid();
  if (fcntl(file_descriptor, F_SETLK, &file_lock) != 0)
  {
    ecl_throw_decl(
        StandardException(LOC,OpenError,std::string("Device is already locked. Try 'lsof | grep ") + port + std::string("' to find other processes that currently have the port open (if the device is a symbolic link you may need to replace the device name with the device that it is pointing to) [posix error in case it is something else: " + std::to_string(errno))));
    error_handler = IsLockedError;
    is_open = false;
    return false;
  }

  /******************************************
   ** Initialise flags
   *******************************************/
  options.c_cflag = 0;
  options.c_iflag = 0;
  options.c_lflag = 0;
  options.c_oflag = 0;

  /*********************
   ** Baud rates
   **********************/
  if (cfsetspeed(&options, baud_rate_flags[baud_rate]) < 0)
  {
    ecl_throw(StandardException(LOC,ConfigurationError,"Setting speed failed."));
    error_handler = InvalidArgError;
    is_open = false;
    return false;
  }

  /*********************
   ** Ownership
   **********************/
  /*
   * CLOCAL and CREAD should always be set.
   * They make sure the owner of the port is
   * not changed by the program.
   */
  options.c_cflag |= CLOCAL;
  options.c_cflag |= CREAD;

  /*********************
   ** Disable Flow control
   **********************/
#if defined(CRTSCTS)
  options.c_cflag &= ~CRTSCTS; // Disable hardware flow control (old)
#elif defined (CNEW_RTSCTS)
  options.c_cflag &= ~CNEW_RTSCTS; // Disable hardware flow control (new)
#endif

  /*********************
   ** StopBits
   **********************/
  if (stop_bits == StopBits_1)
  {
    options.c_cflag &= ~CSTOPB; // One stop bit only
  }
  else
  {
    options.c_cflag |= CSTOPB; // Two stop bits.
  }

  /******************************************
   ** DataBits
   *******************************************/
  options.c_cflag &= ~CSIZE; // Mask (i.e. reset) character size bits
  options.c_cflag |= data_bits_flags[data_bits]; // Now set the # data bits

  /******************************************
   ** Processing
   *******************************************/
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw, no echoing or waiting till a flush or signals
  // options.c_lflag |= (ICANON | ECHO | ECHOE); // like a chat session

  /******************************************
   ** Input Options
   *******************************************/
  options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control

  /******************************************
   ** Output Options
   *******************************************/
//    options.c_oflag |= OPOST; // Perform a little post processing
//    options.c_oflag |= ONLCR; // Man NL to NL-CR
  /*********************
   ** Parity
   **********************/
  if (parity == NoParity)
  {
    options.c_cflag &= ~PARENB; // No Parity
  }
  else if (parity == EvenParity)
  {
    options.c_iflag |= (INPCK | ISTRIP); // Enable parity check and strip parity bit
    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
  }
  else
  { // OddParity
    options.c_iflag |= (INPCK | ISTRIP); // Enable parity check and strip parity bit
    options.c_cflag |= PARENB;
    options.c_cflag |= PARODD;
  }

  /******************************************
   ** Enable
   *******************************************
   * TCSANOW - make changes immediately
   * TCSADRAIN - wait till everything is transmitted, don't do anything with unread data.
   * TCSAFLUSH - wait till everything is transmitted, also clear unread data.
   *******************************************/
  tcsetattr(file_descriptor, TCSAFLUSH, &options);

  /******************************************
  ** Reset Status
  *******************************************/
  if ( read_timeout_ms == NonBlocking ) {
    unblock();
  } else {
    block(read_timeout_ms);
  }
  clear();

  is_open = true;
  error_handler = NoError;
  return true;
}

void Serial::close()
{
  if (is_open)
  {
    // should check return values here, it does have some, EBADF/EINTR/EIO
    ::close(file_descriptor);
    is_open = false;
  }
}

bool Serial::open()
{
  if ( is_open ) {
    if ( access(port.c_str(), F_OK ) == -1 ) {
      close();
    }
  }
  return is_open;
}

/*****************************************************************************
 ** Implementation [Serial][Reading Modes]
 *****************************************************************************/

void Serial::block(const unsigned long &timeout)
{
  if (timeout < 100)
  {
    if (timeout < 5)
    {
      fake_snooze.period(ecl::Duration(0.001));
      fake_loop_count = timeout;
    }
    else if (timeout < 20)
    {
      fake_snooze.period(ecl::Duration(0.002));
      div_t d = div(timeout, 2);
      if (d.rem == 0)
      {
        fake_loop_count = d.quot;
      }
      else
      {
        fake_loop_count = d.quot + 1;
      }
    }
    else
    {
      fake_snooze.period(ecl::Duration(0.005));
      div_t d = div(timeout, 5);
      if (d.rem == 0)
      {
        fake_loop_count = d.quot;
      }
      else
      {
        fake_loop_count = d.quot + 1;
      }
    }
    this->unblock();
  }
  else
  {
    options.c_cc[VMIN] = 0;
    // VTIME is in tenths of a second
    if (timeout < 100)
    { // just in case we're not running in debug mode or exceptions are disabled.
      options.c_cc[VTIME] = static_cast<unsigned char>(1); // 100/100
    }
    else
    {
      options.c_cc[VTIME] = static_cast<unsigned char>(timeout / 100);
    }
    tcsetattr(file_descriptor, TCSAFLUSH, &options);
  }
  read_timeout_ms = timeout;
}

void Serial::unblock()
{
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  tcsetattr(file_descriptor, TCSAFLUSH, &options);
  read_timeout_ms = NonBlocking;
}

/*****************************************************************************
 ** Implementation [Serial][Reading]
 *****************************************************************************/
long Serial::remaining()
{
  long bytes = 0;
  ioctl(file_descriptor, FIONREAD, &bytes);
  return bytes;
}

} // namespace ecl
#endif /* ECL_IS_POSIX */
