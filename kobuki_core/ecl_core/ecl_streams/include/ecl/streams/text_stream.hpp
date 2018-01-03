/**
 * @file /include/ecl/streams/text_stream.hpp
 *
 * @brief Convenience header for various text streams.
 *
 * Convenience header for various text streams.
 *
 * @date October 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_TEXT_STREAM_HPP_
#define ECL_STREAMS_TEXT_STREAM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/devices/traits.hpp>
#include "text_streams/base_text_stream.hpp"
#include "text_streams/output_text_stream.hpp"
#include "text_streams/input_text_stream.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [TextStream]
*****************************************************************************/

/**
 * @brief A text streaming interface.
 *
 * This connects to an underlying device and enables text streaming to and
 * from the device. The device type (determined by input and output
 * device concept checks) determines the type of interface that is
 * brought into the textstream instance, either input, output or both.
 *
 * <b>Usage</b>:
 *
 * <i>Instantiating</i>
 *
 * To open the underlying device,
 *
 * @code
 * TextStream<OFile> ofstream;
 * ofstream.device().open("dudes.txt",New);
 * @endcode
 *
 * <i>Streaming</i>
 *
 * Usage follows very similar to the standard c++ streams.
 *
 * @code
 * ofstream << "dudes " << 32.36;
 * ofstream.flush();
 * @endcode
 *
 * <i>Formatting</i>
 *
 * These streams are usable with the format classes in ecl_formatters.
 *
 * @code
 * Format<double> format; format.width(5); format.precision(2);
 * double d = 1.0/3.0;
 * ostream << format(d);  // This will send 0.33 to the stream.
 * ostream.flush();
 * @endcode
 *
 * <i>Error Checking</i>
 *
 * Output streams can generate errors that are not so easily checked compared with handling devices directly.
 * To check for failure, ecl streams use a mechanism similar to that of the standard cout stream.
 *
 * @code
 * ostream << 32.1;
 * if ( ostream.fail() ) {
 *     std::cout << ostream.errorMessage() << std::endl;
 * }
 * @endcode
 *
 * You can also use the errorStatus() method to retrieve the exact error flag (enumeration).
 *
 * @sa interfaces::InputTextStream<Device,true>, interfaces::OutputTextStream<Device,true>, interfaces::BaseTextStream.
 */
template <typename Device>
class ECL_PUBLIC TextStream : public interfaces::InputTextStream<Device, is_source<Device>::value >,
				   public interfaces::OutputTextStream<Device, is_sink<Device>::value > {
public:
	virtual ~TextStream() {};

};

} // namespace ecl

#endif /* ECL_STREAMS_TEXT_STREAM_HPP_ */
