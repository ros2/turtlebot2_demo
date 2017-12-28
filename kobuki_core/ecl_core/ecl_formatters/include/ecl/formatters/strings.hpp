/**
 * @file /include/ecl/formatters/strings.hpp
 *
 * @brief String type formatter.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_FORMATTERS_STRINGS_HPP_
#define ECL_FORMATTERS_STRINGS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ecl/exceptions/standard_exception.hpp>
#include "common.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Format Interface [String types]
*****************************************************************************/

/**
 * @brief  Formatter for string types.
 *
 * Default formatter for string types.
 * This is pretty slow since we're using strings, but its convenient when
 * speed is not an issue.
 **/
template <>
class ecl_formatters_PUBLIC Format<std::string> {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * Default constructor. Initialises the format tags for width, alignment and base.
         * @param w : width (default - no width constraints)
         * @param a : the textual alignment (default - no alignment constraints)
         **/
        Format(int w = -1, Alignment a = NoAlign) : width_(w),alignment_(a) {}
		virtual ~Format() {}

        /******************************************
        ** Set
        *******************************************/
        /**
         * Sets the width format parameter. Setting this to -1 will turn off width formatting constraints.
         * @param w : the total width of the text output.
         **/
        Format<std::string>& width(int w) { width_ = w; return *this; }
        /**
         * Sets the alignment format parameter. This aligns the text output in the text window which has a width specified by the width parameter.
         * @param a : the aligning property for the output text.
         * @sa Alignment
         **/
        Format<std::string>& align(Alignment a) { alignment_ = a;  return *this; }

        /******************************************
         * Set Format Combinations
         ******************************************/
        /**
         * Convenient stream format parameter setter. This one permanently configures the width and alignment as specfied.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @return Format<String>& : this formatter (for use with streams).
         **/
        Format<std::string>& operator ()(int w, Alignment a);

        /******************************************
        ** Format a value
        *******************************************/
        /**
         * Insert an input value to be formatted when passed to a stream.
         * @param input_string : the string to be formatted.
         * @return Format<string> : formatting object to be passed back to the stream.
         **/
        Format<std::string>& operator() (const std::string &input_string);

        /******************************************
        ** Common format usages
        *******************************************/
        /**
         * Temporarily configure the formatter with a width, base combination along with a value to be formatted.
         * @param input_string : the string to be formatted.
         * @param w : the width to be used.
         * @return Format<string> : formatting object to be passed back to the stream.
         **/
        Format<std::string>& operator() (const std::string &input_string, int w);
        /**
         * Temporarily configure the formatter with a complete parameter set along with a value to be formatted.
         * @param input_string : the string to be formatted.
         * @param w : the width to be used.
         * @param a : the alignment to be used.
         * @return Format<string> : formatting object to be passed back to the stream.
         *
         **/
        Format<std::string>& operator() (const std::string &input_string, int w, Alignment a);

        /******************************************
        ** Insert the formatter into a stream
        *******************************************/
        /**
         * Insertion operator for formatters. It takes a customised formatter and
         * inserts its current context into the stream. If two formatters are used
         * in succession, it will throw an error (c++ cannot handle two in a single
         * line of streaming). This works on normal c++ streams
         * as well as TextStreams.
         * @param ostream : the stream to insert the formatted object into.
         * @param formatter : the format context.
         * @return OutputStream : a handle to the resultant stream.
         *
         * @exception StandardException : throws if the formatter is used multiply in one stream operation [debug mode only].
         **/
        template <typename OutputStream> friend OutputStream& operator << (OutputStream& ostream, Format<std::string>& formatter) ecl_assert_throw_decl(StandardException);

    private:
        /******************************************
        ** Parameters
        *******************************************/
        int width_;
        Alignment alignment_;
        bool ready_to_format;
        std::string s;

        /******************************************
        ** Padding
        *******************************************/
        /**
         * Internally used to pad the output with spaces when requested.
         * @param n : the number of spaces to pad.
         * @param ostream : the streaming operator the formatter is conencted to.
         **/
        template <typename OutputStream> void pad(int n, OutputStream &ostream) const;
        /**
         * Prepad the output with spaces when requested.
         * @param n : the number of spaces to pad.
         * @param ostream : the streaming operator the formatter is conencted to.
         **/
        template <typename OutputStream> void prePad(int n, OutputStream &ostream) const;
        /**
         * Postpad the output with spaces when requested.
         * @param n : the number of spaces to pad.
         * @param ostream : the streaming operator the formatter is conencted to.
         **/
        template <typename OutputStream> void postPad(int n, OutputStream &ostream) const;

        /******************************************
        ** Formatter Functions
        *******************************************/
        /**
         * Internal function that does the actual formatting of a format object into a stream.
         * @param ostream : the stream to be inserted into.
         **/
        template <typename OutputStream> void format(OutputStream &ostream) const;
};

/*****************************************************************************
** Implementation [Format<string>][Templates]
*****************************************************************************/

template <typename OutputStream>
void Format<std::string>::format(OutputStream &ostream) const
{
    int size = s.size();

    prePad(width_ - (size),ostream); // previously had (size+2) here...why?
    ostream << s;
    postPad(width_ - (size),ostream);
}

template <typename OutputStream>
void Format<std::string>::prePad(int n, OutputStream &ostream) const
{
    if ( n <= 0 ) { return; }
    switch ( alignment_ )
    {
        case ( NoAlign ) : { break; }
        case ( LeftAlign ) : { break; }
        case ( RightAlign ) : { pad(n,ostream); break; }
        case ( CentreAlign ) : { pad(n/2+ n%2,ostream); break; } // Add the remainder
        default : break;
    }
}
template <typename OutputStream>
void Format<std::string>::postPad(int n, OutputStream &ostream) const
{
    if ( n <= 0 ) { return; }
    switch ( alignment_ )
    {
        case ( NoAlign ) : { break; }
        case ( LeftAlign ) : { pad(n,ostream); break; }
        case ( RightAlign ) : { break; }
        case ( CentreAlign ) : { pad(n/2,ostream); break; }  // Do not add the remainder
        default : break;
    }
}
template <typename OutputStream>
void Format<std::string>::pad(int n, OutputStream &ostream) const
{
    for (int i = n; i > 0; --i )
    {
        ostream << ' ';
    }
}

/*****************************************************************************
* Implementation [streaming]
*****************************************************************************/

template <typename OutputStream>
OutputStream& operator << (OutputStream &ostream, Format<std::string>& formatter ) ecl_assert_throw_decl(StandardException)
{
    bool ready = formatter.ready_to_format;

    ecl_assert_throw(ready, StandardException(LOC,UsageError,"The formatter cannot print any data - "
            "either there is no data available, or you have tried to use the "
            "formatter more than once in a single streaming operation. "
            "C++ produces unspecified results when functors are used multiply "
            "in the same stream sequence, so this is not permitted here.") );

    if ( ready ) {
        formatter.format(ostream);
        formatter.ready_to_format = false;
    }

    return ostream;
}

}; // namespace ecl

#endif /*ECL_FORMATTERS_STRINGS_HPP_*/
