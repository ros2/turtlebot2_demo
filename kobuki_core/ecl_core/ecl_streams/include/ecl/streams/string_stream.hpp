/**
 * @file /include/ecl/streams/string_stream.hpp
 *
 * @brief Convenience handle for virtual string streams.
 *
 * Convenience handle for virtual string streams.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_STRING_STREAM_HPP_
#define ECL_STREAMS_STRING_STREAM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ecl/config/macros.hpp>
#include <ecl/devices/string.hpp>
#include "text_stream.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/
/**
 * @brief Convenience wrapper for string device based textstreams.
 *
 * This is a wrapper around what could also be used as a
 * TextStream<String> object. It provides a more convenient handle and direct
 * interfaces to some of the string device functions (otherwise you would
 * normally be required to use TextStream's device() method).
 */
class ecl_streams_PUBLIC StringStream : public TextStream<String> {
public:
	/**
	 * @brief Character string representation of the underlying device's contents.
	 *
	 * This returns a null terminates string representing the contents of the
	 * underlying device's internal buffer. Note that this returns the
	 * full buffer, not just the unread parts.
	 *
	 * @return const char* : pointer to the device's internal buffer.
	 */
    const char* c_str() { return this->io_device.c_str(); }

    /**
	 * @brief String representation of the underlying device's contents.
	 *
	 * This generates a c++ style string representing the contents of the
	 * underlying device's internal buffer. Note that this returns the
	 * full buffer, not just the unread parts.
	 *
	 * @return string : string representation of the underlying device's internal buffer.
	 */
    std::string str() { return this->io_device.str(); }

	/**
	 * @brief Clears the underlying device's internal buffers.
	 *
	 * Clears the underlying device's internal character buffer and resets
	 * read/write location pointers.
	 */
    void clear() { io_device.clear(); }

    virtual ~StringStream() {}
};

} // namespace ecl




#endif /* ECL_STREAMS_STRING_STREAM_HPP_ */
