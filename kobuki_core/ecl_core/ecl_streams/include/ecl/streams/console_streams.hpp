/**
 * @file /include/ecl/streams/console_streams.hpp
 *
 * @brief Convenience handles for console textstreams.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_CONSOLE_STREAM_HPP_
#define ECL_STREAMS_CONSOLE_STREAM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "text_stream.hpp"
#include <ecl/devices/console.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Typedefs
*****************************************************************************/
/**
 * @brief Streams to standard output.
 *
 * This class operates similar to, but posesses several advantages to the
 * usual cout c++ mechanism.
 *
 * - Faster.
 * - Can be instantiated separately, each with its own internal buffer.
 *
 * The ecl could provide a global mechanism very much like the cout handle,
 * but instantiating separately stops getting standard output messages
 * confused and is a useful property for multi-threaded
 * control applications. If you wish to use a global mechanism, simply
 * instantiate a global instance in your program.
 *
 * <b>Usage:</b>
 *
 * There is no need to open this device, it automatically connects to
 * stdout.
 * @code
 * OConsoleStream ostream;
 * ostream << "Hey dude";
 * ostream.flush();
 * @endcode
 *
 * @sa @ref ecl::TextStream "TextStream".
 */
typedef TextStream<OConsole> OConsoleStream;
/**
 * @brief Streams to standard input.
 *
 * Streams to standard input.
 *
 * <b>Usage:</b>
 *
 * There is no need to open this device, it automatically connects to
 * stdin.
 * @code
 * IConsoleStream istream;
 * string str;
 * istream << str;
 * @endcode
 */
typedef TextStream<IConsole> IConsoleStream;
/**
 * @brief Streams to standard error.
 *
 * Streams to standard error.
 *
 * <b>Usage:</b>
 *
 * There is no need to open this device, it automatically connects to
 * stderr.
 * @code
 * EConsoleStream estream;
 * estream << "Boom";
 * estream.flush();
 * @endcode
 */
typedef TextStream<EConsole> EConsoleStream;

} // namespace ecl

#endif /* ECL_STREAMS_CONSOLE_STREAM_HPP_ */
