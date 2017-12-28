/**
 * @file /include/ecl/streams/log_stream.hpp
 *
 * @brief Macro enabled, fast logging stream.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_LOG_STREAM_HPP_
#define ECL_STREAMS_LOG_STREAM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <map>
#include <string>
#include <ecl/config/macros.hpp>
#include <ecl/devices/shared_file.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/time/timestamp.hpp>
#include "text_stream.hpp"
#include "macros.hpp"

/*****************************************************************************
** Macros
*****************************************************************************/

/**
 * @brief Enables log streaming.
 *
 * @ingroup Macros
 *
 * Simply use this alongside LogStream's << operator - it will provide the usual
 * text stream functionality, but additionally provides the ability to
 * insert header/timestamp information automatically into the beginning of the stream.
 *
 * @sa @ref ecl::LogStream "LogStream".
 */
#define LOG(logStream,mode) \
  if ( !logStream.isModeEnabled(mode) ) {} \
  else logStream.log(mode)   // << rest of stream input will fill out here

/**
 * @brief Enables flushing of log streams.
 *
 * @ingroup Macros
 *
 * Use this with LOG and LogStream to flush the stream to the shared file.
 *
 * @sa @ref ecl::LogStream "LogStream".
 */
#define FLUSH(logStream) \
  if ( !logStream.isEnabled() ) {} \
  else { logStream.flush(); }

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [LogStream]
*****************************************************************************/
/**
 * @brief A customised textstream for fast, multithreaded logging to file.
 *
 * Together with the LOG and FLUSH macros, this class enables fast,
 * multithreaded logging with multiple modes, customisable headers and timestamps.
 *
 * <b>Usage</b>:
 *
 * Multiple modes are most conveniently utilsed via customised enums. For example:
 *
 * @code
 * enum LogModes {
 *    Warning,
 *    Error,
 *    Debug,
 * };
 * @endcode
 *
 * Note that this is an advantage over alot of other loggers in that it gives you
 * the freedom to define your own error logging levels. Each mode can then be
 * associated inside the log stream with its own customised header.
 *
 * @code
 * LogStream log_stream("test.log")
 * log_stream.enableMode(Warning,"WARNING");
 * log_stream.enableMode(Error,"ERROR");
 * log_stream.enableMode(Debug,"DEBUG");
 * @endcode
 *
 * This process can be repeated from multiple threads each with its own instance
 * of the log stream attached to the single file. Using the log stream is then
 * done via the macros LOG and FLUSH
 *
 * @code
 * LOG(log_stream, Warning) << "This is a log message from main().\n";
 * FLUSH(log_stream)
 * @endcode
 *
 * By default this will automatically add header and timestamp information. You
 * can manually disable these if you prefer.
 */
class ecl_streams_PUBLIC LogStream : public TextStream<SharedFile> {
public:
	/**
	 * @brief Default constructor, underlying device must be manually opened.
	 *
	 * This must open the underlying shared file device manually
	 * via device().open() as you would do if using a TextStream.
	 */
	LogStream() {};
	/**
	 * @brief Convenience constructor for logstreams.
	 *
	 * This constructor enables RAII style construction of the underlying
	 * shared file device.
	 *
	 * @param file_name : output file name.
	 * @param mode : mode for writing (New, Append).
	 *
	 * @exception StandardException : throws if the connection failed to open.
	 */
	LogStream(const std::string &file_name, const WriteMode &mode = New) ecl_throw_decl(StandardException) :
		write_header(true),
		write_stamp(true)
	{
		ecl_try {
			if ( !this->device().open(file_name, mode) ) {
				error = this->device().error();
			}
		} ecl_catch(StandardException &e) {
			ecl_throw(StandardException(LOC,e));
		}
	}

	virtual ~LogStream() {};

	/**
	 * @brief Enable header information.
	 *
	 * This prints the header at the beginning of any logging message. The header is
	 * a string that is specified when calling the enableMode() function.
	 **/
    void enableHeader();
	/**
	 * @brief Disables header information.
	 *
	 * This turns off printing of the header at the beginning of any logging message.
	 **/
    void disableHeader();
	/**
	 * @brief Enable time stamps.
	 *
	 * This prints a timestamp at the beginning of any logging message. The timestamp
	 * is in the format seconds.nanoseconds (think unix time).
	 **/
    void enableTimeStamp();
	/**
	 * @brief Disable time stamps.
	 *
	 * This disables timestamps at the beginning of any logging message. The timestamp
	 * is in the format seconds.nanoseconds (think unix time).
	 **/
    void disableTimeStamp();

    /**
     * @brief Enable the given mode and associate the specified header.
     *
     * Enable the given log mode and associate it with the specified header string.
     * @param mode : the log mode.
     * @param header : the string to precede the log message if required.
     **/
    void enableMode(int mode, const std::string header = "");
    /**
     * @brief Disable the given log mode.
     *
     * Disable the given log mode.
     * @param mode : the log mode.
     **/
    void disableMode(int mode);
    /**
     * @brief Check to see if any modes are enabled.
     *
     * Check to see if any modes are enabled.
     * @return bool : true if any modes are enabled, false otherwise.
     **/
    bool isEnabled();
    /**
     * @brief Check to see if a particular mode is enabled.
     *
     * Check to see if the specified mode is enabled.
     * @return bool : true if the modes is enabled, false otherwise.
     **/
    bool isModeEnabled(int mode);
    /**
     * @brief Log streaming function.
     *
     * Do not use this directly, rather it is indirectly utilised by
     * the LOG macro, which should only reach here once isModeEnabled() is checked.
     * It adds a header and timestamp if it is configured to do so and then passes the
     * stream to the program for further additions.
     *
     * @param mode : log mode that is being logged.
     *
     * @return OutputTextStream : log stream's output streaming parent.
     **/
    LogStream& log(int mode);

    using TextStream<SharedFile>::operator<<;

private:
    bool write_header;
    bool write_stamp;
    std::map<int,std::string> modes;
    TimeStamp timestamp;

};

} // namespace ecl

#endif /* ECL_STREAMS_LOG_STREAM_HPP_ */
