/**
 * @file /ecl_containers/include/ecl/containers/array/formatters.hpp
 *
 * @brief Text formatters for streaming array classes.
 *
 * @date September 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_COMMON_FORMATTERS_HPP_
#define ECL_CONTAINERS_COMMON_FORMATTERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include <ecl/config/macros.hpp>
#include <ecl/formatters/common.hpp>
#include <ecl/formatters/number.hpp>
#include <ecl/formatters/floats.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace formatters {

/*****************************************************************************
** Using
*****************************************************************************/

/*****************************************************************************
** Interface [FloatContainerFormatter]
*****************************************************************************/

/**
 * @brief Parent interface for formatters of fixed size float/double containers.
 *
 * This defines the common interface to be shared by fixed size float/double
 * containers. It only allows configuration of width and precision to be used to
 * format the elements of the container. Note that if the width is set to -1 (the default),
 * this class automatically
 * configures an appropriate width by scanning the elements within and setting it to the minimum
 * width necessary to display all elements properly.
 *
 * Do not use this class directly, rather call it via either the
 * Container type or Format classes explicitly. This makes the code more readable. e.g.
 *
 * @code
 * Array<float>::Formatter format_array_1;
 * Format< Array<float,4> > format_array_2;    // same thing
 * @endcode
 *
 * @tparam Container : the underlying fixed size float or double array.
 *
 * @sa ecl::formatters::Format< Array<float,N>, ecl::formatters::Format<Array<double,N>>.
 *
 * Note, the above don't link properly to the partial specialisations. Look for them
 * in the ecl::formatters namespace.
 *
 */
template <typename Container >
class ECL_LOCAL FloatContainerFormatter {
    public:
        /******************************************
        ** Typedefs
        *******************************************/
        typedef typename Container::value_type value_type; /**< @brief Catches the container element's type. **/

        /**
         * Constructor - can be used to define the precision required for the formatter's elements.
         *
         * @param p : the number of decimal places of precision [default : 2].
         * @param w : width [default : no width constraint]
         **/
        FloatContainerFormatter(const unsigned int p = 2, const int w=-1) :
            old_precision(-1),
            tmp_width(-1),
            prm_width(w),
            width_ptr(&prm_width),
            format(w,p,ecl::RightAlign,ecl::Fixed),
            container(NULL),
            ready_to_format(false)
        {}

		virtual ~FloatContainerFormatter() {}

        /**
         * @brief Sets the precision format parameter.
         *
         * Use this to set the precision of the displayed output.
         *
         * @param p : the number of decimal places of precision.
         * @return FloatContainerFormatter& : this formatter readied for use with a stream.
         **/
        FloatContainerFormatter& precision(const unsigned int p) { format.precision(p); return *this; } // Permanent
        /**
         * @brief Sets the width format parameter.
         *
         * Use this to configure the width of the displayed output. The formatter can be
         * instructed to automatically determine a width suitable for all elements by setting
         * this value to -1.
         *
         * @param w : the width to use for the inserted float [-1 for automatic resizing of cells].
         * @return FloatContainerFormatter& : this formatter readied for use with a stream.
         **/
        FloatContainerFormatter& width(const int w) { prm_width = w; return *this; } // Permanent
        /**
         * @brief Access the current precision used by this formatter.
         *
         * Returns the current precision setting.
         * @return int : the number of decimal places of precision.
         */
        unsigned int precision() const { return format.precision(); }
        /**
         * Permanently sets the stream's precision format. This is simply a convenient
         * version of the precision(p) method.
         * @param p : the number of decimal places of precision.
         * @param w : the width to use for the inserted float.
         * @return FloatContainerFormatter& : this formatter readied for use with a stream.
         */
        FloatContainerFormatter& operator()(const unsigned int p, const int w) { format.precision(p); prm_width = w; return *this; } // Permanent
        /**
         * Convenient stream formatter. This function directly readies the formatter
         * with the specified container and the stored (permanent) settings.
         * @code
         * Format< Array<float,N> > format;
         * cout << format(array) << endl;
         * @endcode
         * @param c : the input container to be formatted.
         * @return FormatFloat& : this formatter readied for use with a stream.
         **/
        FloatContainerFormatter& operator()(const Container &c) { container = &c; ready_to_format = true; return *this; }
        /**
         * Convenient stream formatter. This function directly readies the formatter
         * with the specified container and configures a temporary setting for
         * the precision (one-shot).
         * @code
         * Format< Array<float,N> > format;
         * cout << format(array,2) << endl;
         * @endcode
         * @param c : the input container to be formatted.
         * @param p : the number of decimal places of precision.
         * @param w : the width to use for the inserted float.
         * @return FormatFloat& : this formatter readied for use with a stream.
         **/
        FloatContainerFormatter& operator()(Container &c, const unsigned int p, const int w) {
            container = &c;
            ready_to_format = true;
            old_precision = format.precision();
            format.precision(p);
            tmp_width = w;
            width_ptr = &tmp_width;
            return *this;
        } //temporary

        /**
         * @brief Insertion operator for sending the formatter to an output stream.
         *
         * @param ostream : the output stream.
         * @param formatter : the formatter to be inserted.
         * @return OutputStream : continue streaming with the updated output stream.
         *
         * @exception StandardException : throws if the formatter is used multiply in one stream operation [debug mode only].
         */
        template <typename OutputStream, typename Container_>
        friend OutputStream& operator << (OutputStream& ostream, FloatContainerFormatter< Container_ > &formatter) ecl_assert_throw_decl(StandardException);

    private:
        long digits(value_type min_coeff, value_type max_coeff);

        int old_precision;
        int tmp_width, prm_width;
        int* width_ptr;
        Format< value_type > format;
        const Container *container;
        bool ready_to_format;
};

/**
 * Compute the integral width. This accepts the minimum and maximum values in the array
 * from which a minimum width that provides a common width for every element in the array
 * to be fully displayed is determined.
 * @param min_coeff : the minimum value in the array.
 * @param max_coeff : the maximum value in the array.
 * @return long : the width to use for the integral part of the co-efficients.
 */
template <typename Container >
long FloatContainerFormatter<Container>::digits(value_type min_coeff, value_type max_coeff) {
    long min_digits, max_digits;
    long actual_digits;
    min_digits = static_cast<long> ( ceil(log10(fabs(min_coeff)+1)) );
    max_digits = static_cast<long> ( ceil(log10(fabs(max_coeff)+1)) );
    if ( min_digits < max_digits ) {
        actual_digits = max_digits;
    } else {
        if ( ( min_digits == 0 ) && ( max_digits == 0 ) ) {
            min_digits = 1;
        }
        actual_digits = min_digits;
    }
    if ( min_coeff < 0 ) { actual_digits += 1; }
    actual_digits += 1 + format.precision();
    return actual_digits;
}

/******************************************
** Streamer
*******************************************/

template <typename OutputStream, typename Container_>
OutputStream& operator << (OutputStream& ostream, FloatContainerFormatter< Container_ > &formatter) ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw(formatter.ready_to_format, StandardException(LOC,UsageError,"The formatter cannot print any data - "
            "either there is no data available, or you have tried to use the "
            "formatter more than once in a single streaming operation. "
            "C++ produces unspecified results when functors are used multiply "
            "in the same stream sequence, so this is not permitted here.") );

    if ( *(formatter.width_ptr) == -1 ) {
        /******************************************
        ** Compute min,max coefficients
        *******************************************/
        typename Container_::value_type min_coeff, max_coeff;
        const typename Container_::value_type *cur_coeff;
        min_coeff = (*formatter.container)[0];
        max_coeff = min_coeff;

//        for (size_t i = 1; i < N; ++i ) {
        for ( unsigned int i = 1; i < formatter.container->size(); ++i) {
            cur_coeff = &(*formatter.container)[i];
            if ( *cur_coeff > max_coeff ) { max_coeff = *cur_coeff; }
            if ( *cur_coeff < min_coeff ) { min_coeff = *cur_coeff; }
        }
        /*********************
        ** Set format width
        **********************/
        formatter.format.width(formatter.digits(min_coeff,max_coeff));
    } else {
        formatter.format.width(*(formatter.width_ptr));
    }

    /*********************
    ** Stream
    **********************/
    typename Container_::const_iterator iter;
    ostream << "[ ";
    for ( iter = formatter.container->begin(); iter != formatter.container->end(); ++iter ) {
        ostream << formatter.format(*iter) << " ";
    }
    ostream << "]";

    /*********************
    ** Reset if temporary
    **********************/
    if ( formatter.old_precision != -1 ) {
        formatter.format.precision(formatter.old_precision);
        formatter.old_precision = -1;

    }
    formatter.width_ptr = &(formatter.prm_width);
    formatter.ready_to_format = false;

    ostream.flush();
    return ostream;
}

} // namespace formatters
} // namespace ecl

#endif /* ECL_CONTAINERS_COMMON_FORMATTERS_HPP_ */
