/**
 * @file /include/ecl/streams/manipulators/clear_screen.hpp
 *
 * @brief Really crude hack to clear the screen.
 *
 * Clears the screen just by spamming new lines at it - really crude! But
 * at least it avoids any platform specific code.
 *
 * @date Dec 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_CLEAR_SCREEN_HPP_
#define ECL_STREAMS_CLEAR_SCREEN_HPP_

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
** Interface [ClearScreen]
*****************************************************************************/
/**
 * @brief Manipulator that clears a terminal screen the c++ (hard) way.
 *
 * Function object manipulator that clears an output terminal
 * screen the hard (c++) way by sending 80 new lines to it.
 *
 * The library instantiates a variable of this class under
 * the name 'clrscr', so you need only refer to ecl::streams::clrscr to
 * directly use it with your streams.
 *
 * @sa Manipulator.
 **/
class ecl_streams_PUBLIC ClearScreen : public Manipulator<ClearScreen> {
public:
	/**
	 * @brief Clears a terminal screen the c++ (hard) way.
	 *
	 * Not very fast on most systems - hammers the terminal
	 * with alot of newlines. In windows this causes a problem since
	 * terminal screens are so slow and on screens with large
	 * resolutions, the 80 lines is not enough. Might actually
	 * implement system specific code for this at some time in the future.
	 *
	 * @tparam OutputStream : the output stream type.
	 *
	 * @param ostream : the stream to use for manipulation.
	 */
	template <typename ODevice>
	void action (interfaces::OutputTextStream<ODevice,true>& ostream) {
	    for (int i = 0; i < 80; ++i ) {
	        ostream << "\n";
	    }
	}
	virtual ~ClearScreen() {}
};

/*****************************************************************************
** Global Variables
*****************************************************************************/

extern ClearScreen clrscr;

} // namespace ecl


#endif /* ECL_STREAMS_CLEAR_SCREEN_HPP_ */
