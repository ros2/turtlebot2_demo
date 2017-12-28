/**
 * @file /include/ecl/streams/text_streams/input_text_stream.hpp
 *
 * @brief Input text streaming interface and implementation.
 *
 * Provides an input text streaming interface that can be connected
 * to input devices.
 *
 * @date October 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_INPUT_TEXT_STREAM_HPP_
#define ECL_STREAMS_INPUT_TEXT_STREAM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <limits>
#include <string>
#include <ecl/config/macros.hpp>
#include <ecl/config/portable_types.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/concepts/devices.hpp>
#include <ecl/type_traits/numeric_limits.hpp>
#include "base_text_stream.hpp"
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace interfaces {

/*****************************************************************************
** Interface [InputTextStream]
*****************************************************************************/

/**
 * @brief Parent template for input text streams.
 *
 * This template is defined so that is can provide an empty
 * interface if the calling device is not in fact an input device.
 * The actual input interface is defined in the boolean true specialisation.
 *
 * @tparam Device : underlying device connected to the stream.
 * @tparam InputDevice : boolean indicating if the device is an input device or not.
 *
 * @sa InputTextStream<Device,true>
 */
template <typename Device, bool InputDevice = true >
class ECL_PUBLIC InputTextStream {
};

/**
 * @brief Input text stream interface.
 *
 * Defines the appropriate functionality required for input text streams.
 *
 * @tparam Device : this must be a class that satisfies the
 * input device concept (refer to the documentation in ecl_concepts for details).
 *
 * <b>Usage:</b>
 *
 * - Instantiate
 * - Open the underlying device
 * - Stream
 *
 * @code
 * TextStream<IFile> ifstream;
 * ifstream.device().open("dudes.txt");
 * char c;
 * ifstream >> c;
 * @endcode
 *
 * Some notes - text streams support reading by one of three methods:
 * - readln()
 * - element by element (separated by whitespace or carriage returns
 * - raw characters (by default this is disabled, enable with enableRawCharReads())
 **/
template <typename Device>
class ECL_PUBLIC InputTextStream<Device,true> : public virtual BaseTextStream<Device> {
public:
	/*********************
	** Constructors
	**********************/
	InputTextStream();
	virtual ~InputTextStream() {}

	/*********************
	** Streaming Operators
	**********************/
    InputTextStream<Device>& operator >> ( char &c ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( std::string &s ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( short &i ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( int &i ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( long &i ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( unsigned char &c ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( unsigned short &i ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( unsigned int &i ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( unsigned long &i ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( float &f ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( double &d ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( long long &i ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( unsigned long long &i ) ecl_assert_throw_decl(ecl::StandardException);
    InputTextStream<Device>& operator >> ( bool &b ) ecl_assert_throw_decl(ecl::StandardException);

    /*********************
	** Non-Streaming API
	**********************/
    void enableRawCharReads();
    void disableRawCharReads();

private:
    bool raw_char_reads;

    /*********************
	** Private Parsers
	**********************/
    bool skipLeadingWhiteSpace(char &c);
    template <typename Number>
    bool getIntegerFromStream(Number &i);
    template <typename Number>
    bool parseHexInteger(Number &i);
    template <typename Number>
    bool getFloatFromStream(Number &f);
};

/*****************************************************************************
** Implementation [InputTextStream]
*****************************************************************************/
/**
 * @brief Connects the stream to an input device.
 *
 * Connects the text stream to the associated input device. Use
 * the device() handle to properly open the device, e.g.
 *
 * @code
 * InputTextStream<IFile> ifstream;
 * @endcode
 **/
template <typename Device>
InputTextStream<Device,true>::InputTextStream() : raw_char_reads(false) {
	ecl_compile_time_concept_check(ecl::InputCharDeviceConcept<Device>);
}

/*****************************************************************************
** Implementation [InputTextStream][Streaming Operators]
*****************************************************************************/
/**
 * @brief Reads an unformatted char to the stream.
 *
 * Reads an unformatted char from the stream. Note, this will absolutely
 * read every character in the device.
 *
 * @param c : input value being read from the stream.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device> & InputTextStream<Device,true>::operator >> ( char &c ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));

    if ( !raw_char_reads ) {
    	if( skipLeadingWhiteSpace(c) ) {
    		this->error = NoError;
    	} else {
    		this->error = ReadError;
    	}
    } else {
		if ( this->io_device.read(c) == 1 ) {
			this->error = NoError;
		} else {
			this->error = ReadError;
		}
    }
    return *this;
}

/**
 * @brief Reads an unformatted string from the stream.
 *
 * Reads an unformatted string from the stream. Essentially, this reads characters into a string until
 * either a space or a newline char is reached, that is, it reads whole words at a time and removes any
 * leading or trailing space/newline entities.
 *
 * @param s : input being read from the stream.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( std::string &s ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));

    char c;
	if ( skipLeadingWhiteSpace(c) ) {
	    s.clear();
	    do {
	        s.push_back(c);
	        if( this->io_device.read(c) < 1 ) {
	        	this->error = ReadError;
	        	break;
			}
	    } while ( ( c != ' ' ) && ( c != '\n' ) );
	    this->error = NoError;
	} else {
		this->error = ReadError;
	}

    // Original method - reads a whole line (must be terminated by a \n)
//    static char buffer[80];
//    this->io_device.read(&buffer[0],80);
//    s.assign(buffer);
    return *this;
}

/**
 * @brief Converts a value from the stream to a short.
 * If there is an error at any part of the conversion (non-digit, too large) it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param i : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( short &i ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));

    if ( getIntegerFromStream(i) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getIntegerFromStreams

    return *this;
}

/**
 * @brief Converts a value from the stream to an int.
 * If there is an error at any part of the conversion (non-digit, too large) it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param i : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( int &i ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getIntegerFromStream(i) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getIntegerFromStreams
    return *this;
}

/**
 * @brief Converts a value from the stream to a long.
 * If there is an error at any part of the conversion (non-digit, too large) it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param i : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( long &i ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getIntegerFromStream(i) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getIntegerFromStreams
    return *this;
}
/**
 * @brief Converts a value from the stream to a long long.
 * If there is an error at any part of the conversion (non-digit, too large) it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param i : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( long long &i ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getIntegerFromStream(i) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getIntegerFromStreams
    return *this;
}
/**
 * @brief Converts a value from the stream to an unsigned char.
 *
 * If there is an error at any part of the conversion (non-digit, too large) it will leave the input
 * argument unchanged and set the fail() response to true.
 *
 * @param uc : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( unsigned char &uc ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getIntegerFromStream(uc) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getIntegerFromStreams
    return *this;
}

/**
 * @brief Converts a value from the stream to an unsigned short.
 * If there is an error at any part of the conversion (non-digit, too large) it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param i : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( unsigned short &i ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getIntegerFromStream(i) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getIntegerFromStreams
    return *this;
}

/**
 * @brief Converts a value from the stream to an unsigned int.
 * If there is an error at any part of the conversion (non-digit, too large) it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param i : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( unsigned int &i ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getIntegerFromStream(i) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getIntegerFromStreams
    return *this;
}

/**
 * @brief Converts a value from the stream to an unsigned long.
 * If there is an error at any part of the conversion (non-digit, too large) it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param i : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( unsigned long &i ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getIntegerFromStream(i) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getIntegerFromStreams
    return *this;
}

/**
 * @brief Converts a value from the stream to an unsigned long long.
 * If there is an error at any part of the conversion (non-digit, too large) it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param i : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( unsigned long long &i ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getIntegerFromStream(i) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getIntegerFromStreams
    return *this;
}


/**
 * @brief Converts a value from the stream to a float.
 * If there is an error at any part of the conversion it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param f : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( float &f ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getFloatFromStream(f) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getFloatFromStreams
    return *this;
}

/**
 * @brief Converts a value from the stream to a float.
 * If there is an error at any part of the conversion it will leave the input
 * argument unchanged and set the fail() response to true.
 * @param d : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( double &d ) ecl_assert_throw_decl(ecl::StandardException) {

	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( getFloatFromStream(d) ) {
    	this->error = NoError;
    } // Real errors are handled internally in getFloatFromStreams
    return *this;
}

/**
 * @brief Converts a value from the stream to a bool.
 * If there is an error at any part of the conversion it will leave the input
 * argument unchanged and set the fail() response to true. Valid text input for a bool value
 * includes 'true', 'TRUE', 'false', 'FALSE'.
 *
 * @param b : input value to be filled.
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return InputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
InputTextStream<Device>& InputTextStream<Device,true>::operator >> ( bool &b ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    std::string s;
    *this >> s;

    if ( this->fail() ) { // string could not be read
    	return *this; // error already set
	}

    if ( ( s == "true" ) || ( s == "TRUE" ) || ( s == "True" ) ) {
        b = true;
        this->error = NoError;
    } else if ( ( s == "false" ) || ( s == "FALSE" ) || ( s == "False" ) ) {
        b = false;
        this->error = NoError;
    } else {
    	this->error = ConversionError;
    }
    return *this;
}

/*****************************************************************************
** Implementation [InputTextStream][Non-Streaming Interface]
*****************************************************************************/

/**
 * @brief This permits true char by char reads (including space and newline chars).
 *
 * By default, text streams read chars (via the operator>> (char &c)) method in the same
 * way as every other operator is used, that is, element by element. As a result, any
 * leading whitespace or newline characters are ignored. By calling this function, you
 * can disable this functionality (for the operator>> (char&c) method only) so true
 * byte by byte reads can be performed. This is usually only useful for things like
 * serial devices - not so much for files or standard input style devices.
 *
 * @sa disableRawCharReads().
 */
template <typename Device>
void InputTextStream<Device,true>::enableRawCharReads() { raw_char_reads = true; }

/**
 * @brief This ensures char reads are read element by element (not byte by byte).
 *
 * This has the opposite functionality to enableRawCharReads(). See that method for
 * more information.
 *
 * @sa enableRawCharReads().
 */
template <typename Device>
void InputTextStream<Device,true>::disableRawCharReads() { raw_char_reads = false; }


/*****************************************************************************
** Implementation [InputTextStream][Private Parsers]
*****************************************************************************/

/**
 * @brief Private implementation that churns through any leading white space.
 *
 * This checks for leading space and newline characters. If successful,
 * it returns the first read character in its argument.
 *
 * @param c : if successful, the first legitimate character is returned here.
 * @return bool : true if successful, false otherwise.
 */
template <typename Device>
bool InputTextStream<Device,true>::skipLeadingWhiteSpace(char &c)
{
	do {
        if( this->io_device.read(c) < 1 ) { // Fail if either there is a read error OR there is nothing in the device.
        	this->error = ReadError;
        	return false;
        }
	} while ( ( c == ' ' ) || ( c == '\n' ) );
	return true;
}

/**
 * Parses a hex number from a stream into an integral type.
 * @param i : the integer to stream the value to.
 */
template <typename Device>
template <typename Number>
bool InputTextStream<Device,true>::parseHexInteger(Number &i)
{
    char c;
    static int digits[25];
    static const short ceiling = numeric_limits<short>::maximum/10;

    i = 0;
    int j = 0;
    while ( j < 25 ) {
    	long n = this->io_device.read(c);
    	if ( n < 0 ) {
        	this->error = ReadError;
        	return false;
		} else if ( n == 0 ) {
			break; // nothing more to read.
		}
        if ( ( c >= '0' ) && ( c <= '9') ) {
            digits[j] = c - '0';
        } else if ( ( c >= 'a' ) && ( c <= 'f') ) {
            digits[j] = 10 + c - 'a';
        } else if ( ( c >= 'A' ) && ( c <= 'F') ) {
            digits[j] = 10 + c - 'A';
        } else {
            break; // No more valid characters to read.
        }
        ++j;
    }
    if ( j == 0 ) {
    	this->error = ReadError;
    	return false;
	}

    short number = 0;
    for ( int k = 0; k < j; ++k ) {
        if ( number < ceiling ) {
            number = 16*number + (digits[k]);
        } else {
        	this->error = OutOfRangeError;
            return false;
        }
    }
    i = number;
    return true;
}

/**
 * Parses a value from a stream into an integral type.
 *
 * @param i : the integral type variable to stream the value into.
 */
template <typename Device>
template <typename Number>
bool InputTextStream<Device,true>::getIntegerFromStream(Number &i) {

    static char digits[25]; // 20 is the maximum needed on a 64 bit system for a long type
    static const Number ceiling = ecl::numeric_limits<Number>::maximum/10;
    char c = ' ';
    Number sign_multiplier = 1;
    Number number;

	if ( !skipLeadingWhiteSpace(c) ) { return false; }

    if ( c == '-' ) { // Negative
		if ( std::numeric_limits<Number>::min() != 0 ) {
            sign_multiplier = -1;
            if ( this->io_device.read(c) < 1 ) {
            	this->error = ReadError;
            	return false;
			}
        } // else do nothing and continue, an error will be found at the next step.
    } else if ( c == '+' ) { // Positive
        if ( this->io_device.read(c) < 1 ) {
        	this->error = ReadError;
        	return false;
		}
    } else if ( c == '0' ) {
    	long n = this->io_device.read(c);
    	if ( ( n == 0 ) || ( c == ' ' ) || ( c == '\n' ) ) { // It's just a single zero.
        	i = 0;
        	return true;
        } else if ( n < 0 ) {
        	this->error = ReadError;
        	return false;
		} else if ( c == 'x' ) {
            return parseHexInteger(i);
        } else {
        	// its a regular integer, just continue
        }
    } else if ( c == 'x' ) {
        return parseHexInteger(i);
    }

    /*********************
    ** Parse reg. integer
    **********************/
    int j = 0;
    while ( ( c >= '0' ) && ( c <= '9') && ( j < 25 ) ) {
        digits[j] = c;
        j++;
        long n = this->io_device.read(c);
		if ( n < 0 ) {
			this->error = ReadError;
			return false;
		} else if ( n == 0 ) {
			break;
		} else { // n > 1
        }
    }
    if ( j == 0 ) {
    	this->error = ConversionError;
    	return false;
	}
    number = 0;
    for ( int k = 0; k < j; ++k )
    {
        if ( number < ceiling )
        {
            number = 10*number + (digits[k] - '0');
        } else {
        	this->error = OutOfRangeError;
            return false;
        }
    }
    i = sign_multiplier*number;

    return true;
}

template <typename Device>
template <typename Number>
bool InputTextStream<Device,true>::getFloatFromStream(Number &f) {
    static const int bufferSize = 25;
    static char digits[bufferSize]; // 20 is the maximum needed on a 64 bit system for a long type
    char c = ' ';
    int integral_length = 0;
    int decimal_length = 0;
    Number sign_multiplier = 1;
    Number number;

	if ( !skipLeadingWhiteSpace(c) ) { return false; }

    if ( c == '-' ) {
        sign_multiplier = -1;
        if ( this->io_device.read(c) < 1 ) { this->error = ReadError; return false; }
    } else if ( c == '+' ) {
        if ( this->io_device.read(c) < 1 ) { this->error = ReadError; return false; }
    }
    number = 0;
    /******************************************
    ** Integral part
    *******************************************/
    while ( ( c >= '0' ) && ( c <= '9') && ( integral_length < bufferSize ) ) {
        digits[integral_length] = c;
        long n = this->io_device.read(c);
		if ( n < 0 ) {
			this->error = ReadError; return false;
		} else if ( n == 0 ) {
			break;
		} else { // n > 1
			integral_length++;
        }
    }
    for ( int k = 0; k < integral_length; ++k ) {
        number = 10*number + (digits[k] - '0');
    }

    if ( c == '.' ) {
        float frac_multiplier = 1;
        if ( this->io_device.read(c) < 1 ) { this->error = ReadError; return false; }
        /******************************************
        ** Decimal part
        *******************************************/
        while ( ( c >= '0' ) && ( c <= '9') && ( decimal_length < bufferSize ) ) {
            digits[decimal_length] = c;
            long n = this->io_device.read(c);
    		if ( n < 0 ) {
    			this->error = ReadError; return false;
    		} else if ( n == 0 ) {
    			break;
    		} else { // n > 1
                decimal_length++;
            }
        }
        for ( int k = 0; k < decimal_length; ++k ) {
            frac_multiplier *= 0.1f;
            number = number + frac_multiplier*(digits[k] - '0');
        }
    }

    if ( (integral_length+decimal_length) == 0 ) {
    	this->error = ConversionError;
    	return false;
	}

    f = sign_multiplier*number;

    return true;
}

} // namespace interfaces
} // namespace ecl

#endif /* ECL_STREAMS_INPUT_TEXT_STREAM_HPP_ */
