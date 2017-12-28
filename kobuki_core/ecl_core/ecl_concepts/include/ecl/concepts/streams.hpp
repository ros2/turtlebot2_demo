/**
 * @file /include/ecl/concepts/streams.hpp
 *
 * @brief Defines and validates functionality for the <i>streams</i> concept.
 *
 * @date July 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONCEPTS_STREAMS_HPP_
#define ECL_CONCEPTS_STREAMS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Concept [Stream]
*****************************************************************************/

/**
 * @brief Defines validating functionality for the @ref streamsConcept "streams concept".
 *
 * This only requires very minimal level functionality for streams, most std
 * io streams as well as the ecl streams will work with it.
 **/
template <typename Implementation>
class StreamConcept {
    public:

        /**
         * @brief Implements a concept test for streams.
         *
         * The following conditions are required by streams:
         *
         * - operator<< : a streaming operator which works on at least, chars.
         * - flush()    : a method for flushing the stream to the underlying device.
         */
		ecl_compile_time_concept_test(StreamConcept)
        {
        	stream << 'a';
        	stream.flush();
        }

    private:
        // Putting instantiations here actually saves instantiation (which can cause a
        // problem if there is no default constructor).
        Implementation stream;
};

}; // namespace ecl

#endif /* ECL_CONCEPTS_STREAMS_HPP_ */
