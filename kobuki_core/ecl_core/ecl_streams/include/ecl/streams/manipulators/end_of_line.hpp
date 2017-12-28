/**
 * @file /include/ecl/streams/manipulators/end_of_line.hpp
 *
 * @brief The familiar end of line manipulator, but as a c++ class.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_END_OF_LINE_HPP_
#define ECL_STREAMS_END_OF_LINE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include "../manipulators.hpp"
#include "../text_stream.hpp"
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [EndOfLine]
*****************************************************************************/
/**
 * @brief Replicates the well known end of line/flush combination.
 *
 * This replicates the well known std::endl function as an ecl class based
 * manipulator. The library instantiates a variable of this class under
 * the name 'endl', so you need only refer to ecl::streams::endl to
 * directly use it with your streams.
 *
 * @sa Manipulator.
 */
class ecl_streams_PUBLIC EndOfLine : public Manipulator<EndOfLine> {
public:
	/**
	 * @brief Forces an end of line character/flush combination.
	 *
	 * Forces an end of line character/flush combination.
	 *
	 * @tparam OutputStream : the output stream type.
	 *
	 * @param ostream : the stream to use for manipulation.
	 */
	template <typename ODevice>
	void action (interfaces::OutputTextStream<ODevice,true>& ostream) {
		ostream << "\n";
		ostream.flush();
	}

	virtual ~EndOfLine() {}
};

/*****************************************************************************
** Global Variables
*****************************************************************************/

extern EndOfLine endl;

} // namespace ecl

#endif /* ECL_STREAMS_END_OF_LINE_HPP_ */
