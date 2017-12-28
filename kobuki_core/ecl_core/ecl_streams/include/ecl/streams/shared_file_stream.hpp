/**
 * @file /include/ecl/streams/shared_file_stream.hpp
 *
 * @brief Convenience handles for shared file textstreams.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_SHARED_FILE_STREAMS_HPP_
#define ECL_STREAMS_SHARED_FILE_STREAMS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ecl/config/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/devices/shared_file.hpp>
#include "text_stream.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces [SharedFileStream]
*****************************************************************************/
/**
 * @brief Convenience class for SharedFile TextStream definitions.
 *
 * This class provides a convenient handle for both writing and
 * opening TextStream<SharedFile> objects. It has no further functionality.
 *
 * @sa @ref ecl::TextStream "TextStream".
 */
class ecl_streams_PUBLIC SharedFileStream : public TextStream<SharedFile> {
public:
	/**
	 * @brief Default constructor, underlying device must be manually opened.
	 *
	 * This must open the device manually via device().open() as you would do
	 * if using a TextStream.
	 */
	SharedFileStream() {};
	/**
	 * @brief Convenience constructor for shared file text streams.
	 *
	 * This constructor enables RAII style construction of the underlying
	 * device (this makes it distinct from a generic TextStream<SharedFile> object).
	 *
	 * @param file_name : output file name.
	 * @param mode : mode for writing (New, Append).
	 *
	 * @exception StandardException : throws if the connection failed to open.
	 */
	SharedFileStream(const std::string &file_name, const WriteMode &mode = New) ecl_throw_decl(StandardException) {
		ecl_try {
			if ( !this->device().open(file_name, mode) ) {
				error = this->device().error();
			}
		} ecl_catch(StandardException &e) {
			ecl_throw(StandardException(LOC,e));
		}
	}
	virtual ~SharedFileStream() {};
};

} // namespace ecl


#endif /* ECL_STREAMS_SHARED_FILE_STREAMS_HPP_ */
