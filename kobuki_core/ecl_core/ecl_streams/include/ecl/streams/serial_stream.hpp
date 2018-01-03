/**
 * @file /include/ecl/streams/serial_stream.hpp
 *
 * @brief Convenience handle for serial textstreams.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_SERIAL_STREAM_HPP_
#define ECL_STREAMS_SERIAL_STREAM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ecl/config/macros.hpp>
#include <ecl/devices/serial.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "text_stream.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces [SerialStream]
*****************************************************************************/
/**
 * @brief Convenience class for Serial TextStream definitions.
 *
 * This class provides a convenient handle for both writing and
 * opening TextStream<Serial> objects. It has no further functionality.
 *
 * @sa @ref ecl::TextStream "TextStream".
 */
class ecl_streams_PUBLIC SerialStream : public TextStream<Serial> {
public:
	/**
	 * @brief Default constructor, underlying device must be manually opened.
	 *
	 * This must open the device manually via device().open() as you would do
	 * if using a TextStream.
	 */
	SerialStream() {};
	/**
	 * @brief Convenience constructor for serial text streams.
	 *
	 * This constructor enables RAII style construction of the underlying
	 * device (this makes it distinct from a generic TextStream<Serial> object).
	 *
	 * @param port_name : the device name.
	 * @param baud_rate : baud rate.
	 * @param data_bits : the number of bits in a single message byte.
	 * @param stop_bits : the number of bits after the data used for error checking.
	 * @param parity : the parity used for checksums.
	 *
	 * @exception StandardException : throws if the connection failed to open.
	 */
	SerialStream(const std::string& port_name, const BaudRate &baud_rate = BaudRate_115200, const DataBits &data_bits = DataBits_8,
			const StopBits &stop_bits = StopBits_1, const Parity &parity = NoParity) ecl_throw_decl(StandardException)
	{
		ecl_try {
			this->device().open(port_name, baud_rate, data_bits, stop_bits, parity);
			if ( !this->device().open() ) {
				error = this->device().error();
			}
		} ecl_catch(StandardException &e) {
			ecl_throw(StandardException(LOC,e));
		}
	}
	virtual ~SerialStream() {};

};

} // namespace ecl


#endif /* ECL_STREAMS_SERIAL_STREAM_HPP_ */
