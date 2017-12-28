/**
 * @file /include/ecl/streams/text_streams/base_text_stream.hpp
 *
 * @brief Base interface for text streams.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_BASE_TEXT_STREAM_HPP_
#define ECL_STREAMS_BASE_TEXT_STREAM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/errors/handlers.hpp>
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace interfaces {

/*****************************************************************************
** Interface [BaseTextStream]
*****************************************************************************/
/**
 * @brief Parent class for text stream functionality.
 *
 * This brings the common io functionality together for both input and
 * output text streams, namely the device and
 * its handlers.
 *
 * @tparam Device : the underlying device type.
 */
template <typename Device>
class ECL_PUBLIC BaseTextStream {
public:
	BaseTextStream() : error(ecl::NoError) {};
	virtual ~BaseTextStream() {}
    Device& device();
    bool fail() const;
    const ecl::Error& errorStatus() const;

protected:
    ecl::Error error;
	Device io_device;
};

/*****************************************************************************
** Implementation [BaseTextStream]
*****************************************************************************/
/**
 * @brief Provides access to the underlying output device.
 *
 * Provides access to the underlying output device.
 * @return Device : reference to the underlying output device.
 */
template <typename Device>
Device& BaseTextStream<Device>::device() { return io_device; }

/**
 * @brief Denotes the status of the last device operation.
 *
 * This returns true if the last (read/write) operation failed. This can be due to the
 * input being incorrect (e.g. letters in a value when reading a digit), or
 * because it tried to read past the end of the stream, or the stream is in a
 * bad state.
 * return bool : true if the last stream operation failed, false otherwise.
 **/
template <typename Device>
bool BaseTextStream<Device>::fail() const {
	if ( error.flag() != ecl::NoError ) {
		return true;
	} else {
		return false;
	}
}
/**
 * @brief Enumeration indicating the exit status of the last read operation.
 *
 * This returns an enumeration that indicates the exit status of the last
 * read operation.
 *
 * @return ErrorFlag : the error status indicator (NoError is a valid state).
 */
template <typename Device>
const ecl::Error& BaseTextStream<Device>::errorStatus() const {
	return error;
}

} // namespace interfaces
} // namespace ecl

#endif /* ECL_STREAMS_BASE_TEXT_STREAM_HPP_ */
