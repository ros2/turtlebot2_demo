/**
 * @file /include/ecl/streams/file_streams.hpp
 *
 * @brief Convenience handles for file textstreams.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_FILE_STREAM_HPP_
#define ECL_STREAMS_FILE_STREAM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ecl/config/macros.hpp>
#include <ecl/devices/ofile.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "text_stream.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces [OFileStream]
*****************************************************************************/
/**
 * @brief Convenience class for OFile TextStream definitions.
 *
 * This class provides a convenient handle for both writing and
 * opening TextStream<OFile> objects. It has no further functionality.
 *
 * @sa @ref ecl::TextStream "TextStream".
 */
class ecl_streams_PUBLIC OFileStream : public TextStream<OFile> {
public:
	/**
	 * @brief Default constructor, underlying device must be manually opened.
	 *
	 * This must open the device manually via device().open() as you would do
	 * if using a TextStream.
	 */
	OFileStream() {};

	/**
	 * @brief Convenience constructor for output file text streams.
	 *
	 * This constructor enables RAII style construction of the underlying
	 * device (this makes it distinct from a generic TextStream<OFile> object).
	 *
	 * @param file_name : output file name.
	 * @param mode : mode for writing (New, Append).
	 *
	 * @exception StandardException : throws if the connection failed to open.
	 */
	OFileStream(const std::string &file_name, const WriteMode &mode = New) ecl_throw_decl(StandardException) {
		ecl_try {
			if( !this->device().open(file_name, mode) ) {
				error = this->device().error();
			}
		} ecl_catch(StandardException &e) {
			ecl_throw(StandardException(LOC,e));
		}
	}

	virtual ~OFileStream() {};
};

} // namespace ecl

#endif /* ECL_STREAMS_FILE_STREAM_HPP_ */
