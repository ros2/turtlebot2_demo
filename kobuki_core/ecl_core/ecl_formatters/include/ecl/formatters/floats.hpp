/**
 * @file /include/ecl/formatters/floats.hpp
 *
 * @brief Interface for the float (float,double) formatter.
 *
 * @todo add code for the scientific formatting of a float.
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_FORMATTERS_FLOATS_HPP_
#define ECL_FORMATTERS_FLOATS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdio>   // snprintf
#include <cstring>  // strlen
#include "common.hpp"
#include <ecl/exceptions/standard_exception.hpp>

#if _MSC_VER
#define snprintf _snprintf
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Format Tags
*****************************************************************************/
/**
 * @brief Base format tags for the float formatters.
 *
 * Base format tags for text formatting of floats.
 **/
enum FloatBase
{
    Fixed,  /**< @brief Fixed formatting for floats (i.e. normal decimal representation). **/
    Sci,    /**< @brief Scientific formatting for floats (i.e. exponential representation). **/
};

namespace interfaces {

/*****************************************************************************
** FormatFloat
*****************************************************************************/

/**
 * @brief  Formatter for float types.
 *
 * Default formatter for float types. For convenience, use the individual
 * Format<float|double> classes that inherit this one instead.
 **/
template <typename Number>
class FormatFloat {
public:
	/******************************************
	** C&D's
	*******************************************/
	/**
	 * Default constructor. Initialises the format tags for width, precision, alignment and base.
	 * @param w : width (default - no width constraints)
	 * @param p : the number of decimal places of precision (default - 4)
	 * @param a : the textual alignment (default - no alignment constraints)
	 * @param b : the base format for the floating point representation (default - fixed)
	 **/
	FormatFloat(const int w = -1, const unsigned int p = 4, const ecl::Alignment a = NoAlign, const ecl::FloatBase b = Fixed) :
		prm_width(w),
		prm_precision(p),
		prm_alignment(a),
		prm_base(b),
		width_(&prm_width),
		precision_(&prm_precision),
		alignment_(&prm_alignment),
		base_(&prm_base),
		ready_to_format(false) {}

	virtual ~FormatFloat() {}

	/******************************************
	** Set Format Parameters
	*******************************************/
	/**
	 * @brief Sets the precision format parameter.
	 *
	 * Sets the precision format parameter.
	 *
	 * @param p : the number of decimal places of precision.
	 * @return FormatFloat& : this formatter readied for use with a stream.
	 **/
	FormatFloat<Number>& precision(const unsigned int p) { *precision_ = p; return *this; }
	/**
	 * @brief Sets the width format parameter.
	 *
	 * Sets the width format parameter.
	 *
	 * @param w : the width to use for the inserted float (-1 is no width constraint).
	 * @return FormatFloat& : this formatter readied for use with a stream.
	 **/
	FormatFloat<Number>& width(const int w);
	/**
	 * @brief Sets the alignment format parameter.
	 *
	 * Sets the alignment format parameter.
	 *
	 * @param a : the textual alignment.
	 * @return FormatFloat& : this formatter readied for use with a stream.
	 **/
	FormatFloat<Number>& align(const ecl::Alignment a);
	/**
	 * @brief Sets the base notation to use.
	 *
	 * Sets the base notation to use (fixed or scientific).
	 *
	 * @param b : the base format for the floating point representation.
	 * @return FormatFloat& : this formatter readied for use with a stream.
	 **/
	FormatFloat<Number>& base(const ecl::FloatBase b);

	/******************************************
	** Get Format Parameters
	*******************************************/
	/**
	 * @brief Returns the current precision setting.
	 * @return int : the precision value.
	 */
	int precision() { return *precision_; }

	/**
	 * @brief Returns the current precision setting.
	 *
	 * @return int : the width value (-1 for no constraint).
	 */
	int width() { return *width_; }

	/******************************************
	 * Set Format Combinations
	 ******************************************/
	/**
	 * Convenient stream format parameter setter. This one permanently configures the width and precision as specified.
	 * @param p : the number of decimal places of precision.
	 * @param w : the total width of the text output.
	 * @return FormatFloat& : this formatter readied for use with a stream.
	 **/
	FormatFloat<Number>& operator()(unsigned int p, const int w);
	/**
	 * Convenient stream format parameter setter. This one permanently configures all parameters.
	 * @param p : the number of decimal places of precision.
	 * @param w : the total width of the text output.
	 * @param align : the aligning property for the output text.
	 * @param b : the base representation used for the output.
	 * @return FormatFloat& : this formatter readied for use with a stream.
	 **/
	FormatFloat<Number>& operator()(const unsigned int p, const int w, const ecl::Alignment align, const ecl::FloatBase b);

	/******************************************
	** Format a value
	*******************************************/
	/**
	 * Convenient stream formatter. This function directly formats
	 * the specified input value with the stored settings.
	 * @code
	 * Format<float> format;
	 * cout << format(3,4)(3.5215233) << endl;
	 * @endcode
	 * @param n : the input value to be formatted.
	 * @return FormatFloat& : this formatter readied for use with a stream.
	 **/
	FormatFloat<Number>& operator() (const Number n);

	/******************************************
	** Format for streaming
	*******************************************/
	/**
	 * Convenient stream formatter. This member directly formats the specified input value with the supplied/temporary parameters.
	 * @param n : the input value to be formatted.
	 * @param p : the number of decimal places of precision.
	 * @param w : the total width of the text output.
	 **/
	FormatFloat<Number>& operator() (const Number n, const unsigned int p, const int w);
	/**
	 * Convenient stream formatter. This member directly formats the specified input value with the supplied/temporary parameters.
	 * @param n : the input value to be formatted.
	 * @param p : the number of decimal places of precision.
	 * @param w : the total width of the text output.
	 * @param align : the aligning property for the output text.
	 * @param b : the base representation used for the output.
	 **/
	FormatFloat<Number>& operator() (const Number n, const unsigned int p, const int w, const ecl::Alignment align, const ecl::FloatBase b);

	/******************************************
	** Insert the formatter into a stream
	*******************************************/
	/**
	 * Friend function which allows a stream to act on the formatter to produce
	 * the formatted result. This works on normal c++ streams
	 * as well as TextStreams.
	 * @param ostream : the stream.
	 * @param formatter : the formatter with prespecified format parameters and input value.
	 * @return OutputStream& : return the stream that was utilised.
	 **/
template <typename OutputStream, typename N>
friend OutputStream& operator << (OutputStream& ostream, FormatFloat<N>& formatter) ecl_assert_throw_decl(StandardException);

protected:
	/******************************************
	** Parameters
	*******************************************/
	int prm_width,tmp_width;
	int prm_precision,tmp_precision;
	ecl::Alignment prm_alignment,tmp_alignment;
	ecl::FloatBase prm_base,tmp_base;
	int *width_, *precision_;
	ecl::Alignment *alignment_;
	ecl::FloatBase *base_;
	bool ready_to_format;
	Number value_;

	/******************************************
	** Padding
	*******************************************/
	template <typename OutputStream> void pad(int n, OutputStream &ostream) const;
	template <typename OutputStream> void prePad(int n, OutputStream &ostream) const;
	template <typename OutputStream> void postPad(int n, OutputStream &ostream) const;

	/******************************************
	** Formatter Functions
	*******************************************/
	template <typename OutputStream> void formatFixed(OutputStream &ostream) const;
	template <typename OutputStream> void formatSci(OutputStream &ostream) const;
};

/*****************************************************************************
* FormatFloat Implementation  [single parameter configuration]
*****************************************************************************/
/**
 * Sets the base format. For floating point values, this can be fixed, exponential or scientific.
 * @param b : the base representation used for the output.
 * @sa FloatBase
 **/
template <typename Number>
FormatFloat<Number>& FormatFloat<Number>::base(const ecl::FloatBase b)
{
    *base_ = b;
    return *this;
}
/**
 * Sets the width format parameter. Setting this to -1 will turn off width formatting constraints.
 * @param w : the total width of the text output.
 **/
template <typename Number>
FormatFloat<Number>& FormatFloat<Number>::width(const int w)
{
    *width_ = w;
    if ( ( *width_ > 0 ) && ( *alignment_ == NoAlign ) )
    {
        *alignment_ = RightAlign;
    }
    return *this;
}

/**
 * Sets the alignment format parameter. This aligns the text output in the text window which has a width specified by the width parameter.
 * @param a : the aligning property for the output text.
 * @sa Alignment
 **/
template <typename Number>
FormatFloat<Number>& FormatFloat<Number>::align(const ecl::Alignment a)
{
    *alignment_ = a;
    if ( *alignment_ == NoAlign )
    {
        *width_ = 0;
    }
    return *this;
}

/*****************************************************************************
* FormatFloat Implementation [format combination configuration]
*****************************************************************************/

template <typename Number>
FormatFloat<Number>& FormatFloat<Number>::operator () (const unsigned int p, const int w)
{
    width(w);
    precision(p);
    return *this;
}

template <typename Number>
FormatFloat<Number>& FormatFloat<Number>::operator () (const unsigned int p, const int w, const ecl::Alignment a, const ecl::FloatBase b)
{
    width(w);
    precision(p);
    align(a);
    base(b);
  return *this;
}

/*****************************************************************************
* FormatFloat Implementation [value]
*****************************************************************************/
template <typename Number>
FormatFloat<Number>& FormatFloat<Number>::operator()(const Number n)
{
    value_ = n;
    ready_to_format = true;
  return *this;
}
/*****************************************************************************
* FormatFloat Implementation [temporary formatting]
*****************************************************************************/
template <typename Number>
FormatFloat<Number>& FormatFloat<Number>::operator() (const Number n, unsigned int p, const int w)
{
    width_ = &tmp_width;
    width(w);

    precision_ = &tmp_precision;
    precision(p);

    alignment_ = &tmp_alignment;
    align(prm_alignment);

    base_ = &tmp_base;
    base(prm_base);

    value_ = n;
    ready_to_format = true;
    return *this;
}

template <typename Number>
FormatFloat<Number>& FormatFloat<Number>::operator() (const Number n, const unsigned int p, const int w, const ecl::Alignment a, const ecl::FloatBase b)
{
    width_ = &tmp_width;
    width(w);

    precision_ = &tmp_precision;
    precision(p);

    alignment_ = &tmp_alignment;
    align(a);

    base_ = &tmp_base;
    base(b);

    value_ = n;
    ready_to_format = true;
    return *this;
}


/*****************************************************************************
* FormatFloat Implementation [internal formatting]
*****************************************************************************/
template <typename Number>
  template <typename OutputStream>
void FormatFloat<Number>::formatFixed(OutputStream &ostream) const
{
    static char format_specifier[6] = "%.xff";
    if ( *precision_ < 10 ) {
        format_specifier[2] = '0'+*precision_;
        format_specifier[3] = 'f';
        format_specifier[4] = '\0';
    } else if ( *precision_ < 20 )  {
        format_specifier[2] = '1';
        format_specifier[3] = '0'+(*precision_ - 10);
        format_specifier[4] = 'f';
    } else {
        format_specifier[2] = '2';
        format_specifier[3] = '0';
        format_specifier[4] = 'f';
    }
    static const int buffer_size = 30;
    static char buffer[buffer_size];

    snprintf(buffer, buffer_size, format_specifier, value_);

    /******************************************
    ** Streaming out
    *******************************************/
    int length = strlen(buffer);
    prePad(*width_ - length,ostream);
    ostream << buffer;
    postPad(*width_ - length,ostream);
}

template <typename Number>
  template <typename OutputStream>
void FormatFloat<Number>::formatSci(OutputStream &ostream) const
{
    ostream << "Scientific format is not yet supported." << "\n";
    ostream.flush();
}

template <typename Number>
  template <typename OutputStream>
void FormatFloat<Number>::prePad(int n, OutputStream &ostream) const
{
    if ( n <= 0 ) { return; }
    switch ( *alignment_ )
    {
        case ( NoAlign ) : { break; }
        case ( LeftAlign ) : { break; }
        case ( RightAlign ) : { pad(n,ostream); break; }
        case ( CentreAlign ) : { pad(n/2+ n%2,ostream); break; } // Add the remainder
        default : break;
    }
}

template <typename Number>
  template <typename OutputStream>
void FormatFloat<Number>::postPad(int n, OutputStream &ostream) const
{
    if ( n <= 0 ) { return; }
    switch ( *alignment_ )
    {
        case ( NoAlign ) : { break; }
        case ( LeftAlign ) : { pad(n,ostream); break; }
        case ( RightAlign ) : { break; }
        case ( CentreAlign ) : { pad(n/2,ostream); break; }  // Do not add the remainder
        default : break;
    }
}

template <typename Number>
  template <typename OutputStream>
void FormatFloat<Number>::pad(int n, OutputStream &ostream) const
{
    for (int i = n; i > 0; --i )
    {
        ostream << ' ';
    }
}

/*****************************************************************************
* FormatFloat Implementation [streaming]
*****************************************************************************/
template <typename OutputStream, typename N>
OutputStream& operator << (OutputStream &ostream, FormatFloat<N>& formatter ) ecl_assert_throw_decl(StandardException)
{
    bool ready = formatter.ready_to_format;

    ecl_assert_throw(ready, StandardException(LOC,UsageError,"The formatter cannot print any data - "
            "either there is no data available, or you have tried to use the "
            "formatter more than once in a single streaming operation. "
            "C++ produces unspecified results when functors are used multiply "
            "in the same stream sequence, so this is not permitted here.") );

    if ( ready )
    {
        switch(*(formatter.base_) )
        {
            case(Fixed) : {
                formatter.formatFixed(ostream);
                break;
            }
            case(Sci) : {
                formatter.formatSci(ostream);
                break;
            }
        }

        // Switch pointers back to prms if not already there.
        if ( formatter.width_ != &(formatter.prm_width) ) {
            formatter.width_ = &(formatter.prm_width);
            formatter.precision_ = &(formatter.prm_precision);
            formatter.alignment_ = &(formatter.prm_alignment);
            formatter.base_ = &(formatter.prm_base);
        }
        formatter.ready_to_format = false;
    }
    return ostream;
}

} // namespace interfaces

/*****************************************************************************
* Format Classes
*****************************************************************************/
/**
 * @brief Formatter for float types.
 *
 * It is a specialisation of the Format<T> template family. Its entire functionality
 * is inherited from FormatFloat (this class simply provides a convenient handle in the
 * Format<T> family).
 **/
template <>
class Format<float> : public interfaces::FormatFloat<float>
{
    public:
        /**
         * Default constructor. Initialises the format tags for width, precision, alignment and base.
         * @param w : width (default - no width constraints)
         * @param p : the number of decimal places of precision (default - 4)
         * @param a : the textual alignment (default - no alignment constraints)
         * @param b : the base format for the floating point representation (default - fixed)
         **/
        Format(int w = -1, int p = 4, Alignment a = NoAlign, FloatBase b = Fixed) :
            interfaces::FormatFloat<float>(w, p, a, b) {}

        virtual ~Format() {}
};
/**
 * @brief Formatter for double types.
 *
 * It is a specialisation of the Format<T> template family. Its entire functionality
 * is inherited from FormatFloat (this class simply provides a convenient handle in the
 * Format<T> family).
 **/
template <>
class Format<double> : public interfaces::FormatFloat<double>
{
    public:
        /**
         * Default constructor. Initialises the format tags for width, precision, alignment and base.
         * @param w : width (default - no width constraints)
         * @param p : the number of decimal places of precision (default - 4)
         * @param a : the textual alignment (default - no alignment constraints)
         * @param b : the base format for the floating point representation (default - fixed)
         **/
        Format(int w = -1, int p = 4, Alignment a = NoAlign, FloatBase b = Fixed) :
            interfaces::FormatFloat<double>(w, p, a, b) {
            }

        virtual ~Format() {}
};

}; // namespace ecl

#endif /*ECL_FORMATTERS_FLOATS_HPP_*/
