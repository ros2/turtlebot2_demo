/**
 * @file /include/ecl/formatters/number.hpp
 *
 * @brief Integral type formatter.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_FORMATTERS_NUMBER_HPP_
#define ECL_FORMATTERS_NUMBER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "common.hpp"
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/converters/char_strings.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Format Tags
*****************************************************************************/
/**
 * @brief Base format tags for the integral formatters.
 *
 * Format tags for text formatting of integral numbers.
 **/
enum IntegralBase
{
    Bin, /**< @brief Binary representation for integral types. **/
    Hex, /**< @brief Hex representation for integral types. **/
    Dec, /**< @brief Decimal (i.e. normal) representation for integral types. **/
};

namespace interfaces {

/*****************************************************************************
** FormatNumber Interface [Integral types]
*****************************************************************************/

/**
 * @brief  Formatter for integral types.
 *
 * Default formatter for integral types. For convenience, use the individual
 * Format classes that inherit this one instead.
 **/
template < typename Number >
class FormatNumber {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * Default constructor. Initialises the format tags for width, alignment and base.
         * @param w : width (default - no width constraints)
         * @param a : the textual alignment (default - no alignment constraints)
         * @param b : the base format for the integral representation (default - decimal)
         **/
        FormatNumber(int w = -1, ecl::Alignment a = NoAlign, ecl::IntegralBase b = Dec) : prm_width(w),prm_alignment(a),prm_base(b),width_(&prm_width),alignment_(&prm_alignment),base_(&prm_base),ready_to_format(false) {}

        virtual ~FormatNumber() {}

        /******************************************
        ** Set
        *******************************************/
        /**
         * Sets the base format. For integral types, this can be binary, hexadecimal or decimal.
         * @param b : the base representation used for the output.
         * @sa IntegralBase
         **/
        FormatNumber<Number>& base(ecl::IntegralBase b);
        /**
         * Sets the width format parameter. Setting this to -1 will turn off width formatting constraints.
         * @param w : the total width of the text output.
         **/
        FormatNumber<Number>& width(int w);
        /**
         * Sets the alignment format parameter. This aligns the text output in the text window which has a width specified by the width parameter.
         * @param a : the aligning property for the output text.
         * @sa ecl::formatters::Alignment
         **/
        FormatNumber<Number>& align(ecl::Alignment a);

        /******************************************
         * Set Format Combinations
         ******************************************/
        /**
         * Convenient stream format parameter setter. This one permanently configures the width alignment and base as specified.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @param b : the base representation used for the output.
         * @return FormatNumber& : this formatter readied for use by a stream.
         **/
        FormatNumber<Number>& operator ()(int w, ecl::Alignment a, ecl::IntegralBase b);

        /******************************************
        ** Format a value
        *******************************************/
        /**
         * Convenient stream formatter. This function directly formats the specified
         * input value with the stored settings.
         * @param n : the input value to be formatted.
         * @return FormatNumber& : this formatter readied for use by a stream.
         **/
        FormatNumber<Number>& operator() (Number n);

        /******************************************
        ** Common format usages
        *******************************************/
        /**
         * Convenient stream formatter. This member directly formats the specified
         * input value with the supplied/temporary parameters.
         * @param n : the input value to be formatted.
         * @param w : the total width of the text output.
         * @param b : the base representation used for the output.
         * @return FormatNumber& : this formatter readied for use by a stream.
         **/
        FormatNumber<Number>& operator() (Number n, int w, ecl::IntegralBase b);
        /**
        * Convenient stream formatter. This member directly formats the specified
        * input value with the supplied/temporary parameters.
        * @param n : the input value to be formatted.
        * @param w : the total width of the text output.
        * @param a : the aligning property for the output text.
        * @param b : the base representation used for the output.
        * @return FormatNumber& : this formatter readied for use by a stream.
        **/
        FormatNumber<Number>& operator() (Number n, int w, ecl::Alignment a, ecl::IntegralBase b);

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
         *
         * @exception StandardException : throws if the formatter is used multiply in one stream operation [debug mode only].
         **/
        template <typename OutputStream, typename N> friend OutputStream& operator << (OutputStream& ostream, FormatNumber<N>& formatter) ecl_assert_throw_decl(StandardException);

    protected:
        /******************************************
        ** Paramaters
        *******************************************/
        int prm_width, tmp_width;
        ecl::Alignment prm_alignment, tmp_alignment;
        ecl::IntegralBase prm_base, tmp_base;
        int *width_;
        ecl::Alignment *alignment_; IntegralBase *base_;
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
        template <typename OutputStream> void formatBin(OutputStream &ostream) const;
        template <typename OutputStream> void formatHex(OutputStream &ostream) const;
        template <typename OutputStream> void formatDec(OutputStream &ostream) const;

}; // FormatNumber

/*****************************************************************************
* FormatNumber Implementation  [configuration]
*****************************************************************************/
template <typename Number>
FormatNumber<Number>& FormatNumber<Number>::base(ecl::IntegralBase b)
{
    *base_ = b;
    return *this;
}

template <typename Number>
FormatNumber<Number>& FormatNumber<Number>::width(int w)
{
    *width_ = w;
    if ( ( *width_ > 0 ) && ( *alignment_ == NoAlign ) ) {
        *alignment_ = RightAlign;
    }
    return *this;
}

template <typename Number>
FormatNumber<Number>& FormatNumber<Number>::align(ecl::Alignment a)
{
    *alignment_ = a;
    if ( *alignment_ == NoAlign )
    {
        *width_ = 0;
    }
    return *this;
}
template <typename Number>
FormatNumber<Number>& FormatNumber<Number>::operator () (int w, ecl::Alignment a, ecl::IntegralBase b)
{
    base(b);
    width(w);
    align(a);
    return *this;
}

/*****************************************************************************
* FormatNumber Implementation [internal formatting]
*****************************************************************************/
template <typename Number>
  template <typename OutputStream>
void FormatNumber<Number>::formatBin(OutputStream &ostream) const
{
    int size = 8*sizeof(Number);
    prePad(*width_ - (size + 2),ostream);
    ostream << "0b";
    for (int i = size - 1; i>=0; i--)
    {
        ostream << "01"[((value_ >> i) & 1)];
    }
    postPad(*width_ - (size + 2),ostream);
}

template <typename Number>
  template <typename OutputStream>
void FormatNumber<Number>::formatHex(OutputStream &ostream) const
{
    static const char hex_string[16] = { '0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f' };

    int size = 2*sizeof(Number);
    prePad(*width_ - (size + 2),ostream);
    ostream << "0x";
    for (int i = size - 1; i>=0; i--)
    {
//        ostream << "0123456789abcdef"[((value_ >> i*4) & 0xF)];
        ostream << hex_string[((value_ >> i*4) & 0xF)]; // Dont have to recreate this every time
    }
    postPad(*width_ - (size + 2),ostream);
}

template <typename Number>
  template <typename OutputStream>
void FormatNumber<Number>::formatDec(OutputStream &ostream) const
{
    static ecl::Converter<char*> convert;
    char *s = convert(value_);
    int size = strlen(s);
    prePad(*width_ - size,ostream);
    ostream << s;
    postPad(*width_ - size,ostream);
}

template <typename Number>
  template <typename OutputStream>
void FormatNumber<Number>::prePad(int n, OutputStream &ostream) const
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
void FormatNumber<Number>::postPad(int n, OutputStream &ostream) const
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
void FormatNumber<Number>::pad(int n, OutputStream &ostream) const
{
    for (int i = n; i > 0; --i )
    {
        ostream << ' ';
    }
}
/*****************************************************************************
* FormatNumber Implementation [value]
*****************************************************************************/
/**
 * Insert an input value to be formatted when passed to a stream.
 **/
template <typename Number>
FormatNumber<Number>& FormatNumber<Number>::operator()(Number n)
{
    value_ = n;
    ready_to_format = true;
    return *this;
}

/*****************************************************************************
* FormatNumber Implementation [temporary formatting]
*****************************************************************************/

/**
 * Temporarily configure the formatter with a width, base combination along with a value to be formatted.
 **/
template <typename Number>
FormatNumber<Number>& FormatNumber<Number>::operator() (Number n, int w, ecl::IntegralBase b)
{
    width_ = &tmp_width;
    alignment_ = &tmp_alignment;
    base_ = &tmp_base;
    base(b);
    width(w);
    value_ = n;
    ready_to_format = true;
    return *this;
}
/**
 * Temporarily configure the formatter with a complete parameter set along with a value to be formatted.
 **/
template <typename Number>
FormatNumber<Number>& FormatNumber<Number>::operator() (Number n, int w, ecl::Alignment a, ecl::IntegralBase b)
{
    width_ = &tmp_width;
    alignment_ = &tmp_alignment;
    base_ = &tmp_base;
    base(b);
    width(w);
    align(a);
    value_ = n;
    ready_to_format = true;
    return *this;
}


/*****************************************************************************
* FormatNumber Implementation [streaming]
*****************************************************************************/
template <typename OutputStream, typename N>
OutputStream& operator << (OutputStream &ostream, FormatNumber<N>& formatter ) ecl_assert_throw_decl(StandardException)
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
            case(Bin) : {
                formatter.formatBin(ostream);
                break;
            }
            case(Hex) : {
                formatter.formatHex(ostream);
                break;
            }
            case(Dec) : {
                formatter.formatDec(ostream);
                break;
            }
        }
        if ( formatter.width_ != &(formatter.prm_width) ) {
            formatter.width_ = &(formatter.prm_width);
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
 * @brief  Formatter for short types. It is a specialisation of the Format<IntegralNumber>
 * class.
 **/
template <>
class Format<short> : public interfaces::FormatNumber<short>
{
    public:
        /**
         * Default constructor.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @param b : the base representation used for the output.
         */
        Format(int w = -1, Alignment a = NoAlign, IntegralBase b = Dec) : interfaces::FormatNumber<short>(w, a, b) {}
		virtual ~Format() {}
};
/**
 * @brief  Formatter for int types. It is a specialisation of the Format<IntegralNumber>
 * class.
 **/
template <>
class Format<int> : public interfaces::FormatNumber<int>
{
    public:
        /**
         * Default constructor.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @param b : the base representation used for the output.
         */
        Format(int w = -1, Alignment a = NoAlign, IntegralBase b = Dec) : interfaces::FormatNumber<int>(w, a, b) {}
		virtual ~Format() {}
};
/**
 * @brief  Formatter for long types. It is a specialisation of the Format<IntegralNumber>
 * class.
 **/
template <>
class Format<long> : public interfaces::FormatNumber<long>
{
    public:
        /**
         * Default constructor.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @param b : the base representation used for the output.
         */
        Format(int w = -1, Alignment a = NoAlign, IntegralBase b = Dec) : interfaces::FormatNumber<long>(w, a, b) {}
		virtual ~Format() {}
};

/**
 * @brief  Formatter for char types. It is a specialisation of the Format<IntegralNumber>
 * class.
 **/
template <>
class Format<char> : public interfaces::FormatNumber<char>
{
    public:
        /**
         * Default constructor.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @param b : the base representation used for the output.
         */
        Format(int w = -1, Alignment a = NoAlign, IntegralBase b = Dec) : interfaces::FormatNumber<char>(w, a, b) {}
		virtual ~Format() {}
};

/**
 * @brief  Formatter for signed char types. It is a specialisation of the Format<IntegralNumber>
 * class.
 **/
template <>
class Format<signed char> : public interfaces::FormatNumber<signed char>
{
    public:
        /**
         * Default constructor.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @param b : the base representation used for the output.
         */
        Format(int w = -1, Alignment a = NoAlign, IntegralBase b = Dec) : interfaces::FormatNumber<signed char>(w, a, b) {}
		virtual ~Format() {}
};

/**
 * @brief  Formatter for unsigned short types. It is a specialisation of the Format<IntegralNumber>
 * class.
 **/
template <>
class Format<unsigned short> : public interfaces::FormatNumber<unsigned short>
{
    public:
        /**
         * Default constructor.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @param b : the base representation used for the output.
         */
        Format(int w = -1, Alignment a = NoAlign, IntegralBase b = Dec) : interfaces::FormatNumber<unsigned short>(w, a, b) {}
		virtual ~Format() {}
};

/**
 * @brief  Formatter for unsigned int types. It is a specialisation of the Format<IntegralNumber>
 * class.
 **/
template <>
class Format<unsigned int> : public interfaces::FormatNumber<unsigned int>
{
    public:
        /**
         * Default constructor.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @param b : the base representation used for the output.
         */
        Format(int w = -1, Alignment a = NoAlign, IntegralBase b = Dec) : interfaces::FormatNumber<unsigned int>(w, a, b) {}
		virtual ~Format() {}
};
/**
 * @brief  Formatter for unsigned long types. It is a specialisation of the Format<IntegralNumber>
 * class.
 **/
template <>
class Format<unsigned long> : public interfaces::FormatNumber<unsigned long>
{
    public:
        /**
         * Default constructor.
         * @param w : the total width of the text output.
         * @param a : the aligning property for the output text.
         * @param b : the base representation used for the output.
         */
        Format(int w = -1, Alignment a = NoAlign, IntegralBase b = Dec) : interfaces::FormatNumber<unsigned long>(w, a, b) {}
		virtual ~Format() {}
};
/**
 * @brief  Formatter for unsigned char types. It is a specialisation of the Format<IntegralNumber>
 * class.
 **/
template <>
class Format<unsigned char> : public interfaces::FormatNumber<unsigned char>
{
    public:
        Format(int w = -1, Alignment a = NoAlign, IntegralBase b = Dec) : interfaces::FormatNumber<unsigned char>(w, a, b) {} /**< Default constructor. */
		virtual ~Format() {}
};

}; // namespace ecl

#endif /*ECL_FORMATTERS_NUMBER_HPP_*/
