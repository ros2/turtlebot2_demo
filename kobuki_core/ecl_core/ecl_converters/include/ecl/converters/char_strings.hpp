/**
 * @file /include/ecl/converters/char_strings.hpp
 *
 * @brief Type conversions to character strings.
 *
 * A variety of functors that do type conversion to character
 * strings. These are designed to be very fast, unlike the string converters.
 *
 * @date May 2009
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONVERTERS_CONVERT_CHAR_STRINGS_HPP_
#define ECL_CONVERTERS_CONVERT_CHAR_STRINGS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdio> // vsnprintf
#include <cstring> //strcat
#include <new>
#include "converter.hpp"
#include <ecl/errors/handlers.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/mpl/converters.hpp>

#if _MSC_VER
#define snprintf _snprintf
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @cond DO_NOT_DOXYGEN
 */
namespace converters {

/*****************************************************************************
** Inheritable Buffer Interface
*****************************************************************************/
/**
 * @brief An inheritable buffer class for the character string converters.
 *
 * Provides a buffer interface for the character string classes. Buffers can
 * either be created internally, or provide from without. Any inheriting class
 * should provide constructors that will initialise the inherited buffer
 * appropriately.
 **/
class CharStringBuffer : public ConverterBase {
    protected:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * @brief Initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        CharStringBuffer(char* begin, char* end) : buffer_begin(begin),buffer_end(end),delete_buffer(false) {}
        /**
         * @brief Initialises with an internal buffer.
         *
         * If allocation of the internal buffer fails, it will throw an exception (debug mode only) or set
         * the error handler with a MemoryError.
         *
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
         * @exception StandardException : throws if the allocation for the internal buffer failed (debug mode only).
         **/
        CharStringBuffer(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : delete_buffer(true) {
           	buffer_begin = new (std::nothrow) char[buffer_size];
        	buffer_end = buffer_begin + (buffer_size-1);
           	if ( buffer_begin == NULL ) {
           		ecl_debug_throw(ecl::StandardException(LOC,ecl::MemoryError,"Failed to allocate memory for the char string buffer."));
           		error_handler = ecl::MemoryError;
           	}
        }
        /**
         * Default destructor. If a buffer was initialised by this object, it is deleted.
         **/
        virtual ~CharStringBuffer() {
        	if(delete_buffer) {
        		delete[] buffer_begin;
        		delete_buffer = false;
        	}
        }

        char *buffer_begin;
        char *buffer_end;
        bool delete_buffer;
};

/*****************************************************************************
** Character String Converter Utilities
*****************************************************************************/
/**
 * @brief Fast internal utility function that converts an unsigned integral and puts it on the buffer.
 *
 * This function uses an assert (debug mode only) at the end of processing to check the buffer's
 * range has not been exceeded.
 *
 * @param number : The unsigned number to be converted.
 * @param buffer_begin : start of the buffer to be used.
 * @param buffer_end : end of the buffer to be used.
 * @return char* : a pointer to the buffer (NULL if the buffer was exceeded).
 **/
template <typename Number>
char* convertUnsignedIntegral(Number number, char* buffer_begin, char* buffer_end)
{
    *buffer_end = 0; // Set to the null terminator
    Number lsd;
    char* str_ptr = buffer_end;

    do {
        lsd = static_cast<Number>(number % 10);     // Determine the least significant digit.
        number = static_cast<Number>(number / 10);  // Deal with next most significant.
        --str_ptr;
        if ( str_ptr < buffer_begin ) { return NULL; }
        *str_ptr = '0'+lsd;
    } while (number != 0);
    return str_ptr;
};

/**
 * Very fast internal utility function that converts a signed integral and puts it on the buffer.
 * @param number : the unsigned number to be converted.
 * @param buffer_begin : start of the buffer to be used.
 * @param buffer_end : end of the buffer to be used.
 * @return char* : a pointer to the buffer.
 *
 **/
template <typename Number>
char* convertSignedIntegral(const Number &number, char* buffer_begin, char* buffer_end)
{
    typedef typename Unsigned<Number>::type UnsignedNumber;

    char *s;
    if ( number >= 0 ) {
        s = convertUnsignedIntegral(static_cast<UnsignedNumber>(number),buffer_begin,buffer_end);
    } else {
        s = convertUnsignedIntegral(static_cast<UnsignedNumber>(-1*number),buffer_begin+1,buffer_end);
        --s;
        *s = '-';
    }
    return s;
}


}; // namespace converters

/**
 * @endcond
 */

/*****************************************************************************
** Char String Converters [char]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,char> : public virtual converters::CharStringBuffer {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {}
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
         * @exception StandardException : throws if the allocation for the internal buffer failed (debug mode only).
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) :
        		converters::CharStringBuffer(buffer_size) {}

        virtual ~Converter() {}

        /******************************************
        ** Converters
        *******************************************/
        /**
         * Converts a char to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const char &input) {
            return converters::convertUnsignedIntegral(input,this->buffer_begin,this->buffer_end);
        }
};
/*****************************************************************************
** Char String Converters [unsigned char]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,unsigned char> : public virtual converters::CharStringBuffer {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {}
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
         * @exception StandardException : throws if the allocation for the internal buffer failed (debug mode only).
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {}

        virtual ~Converter() {}

        /******************************************
        ** Converters
        *******************************************/
        /**
         * Converts an unsigned char to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const unsigned char &input){ return converters::convertUnsignedIntegral(input,this->buffer_begin,this->buffer_end); }
};
/*****************************************************************************
** Char String Converters [short]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,short> : public virtual converters::CharStringBuffer {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {}
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {}

        virtual ~Converter() {}

        /******************************************
        ** Converters
        *******************************************/
        /**
         * Converts a short to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const short &input){ return converters::convertSignedIntegral(input,this->buffer_begin,this->buffer_end); }
};
/*****************************************************************************
** Char String Converters [unsigned short]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,unsigned short> : public virtual converters::CharStringBuffer {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {}
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {}

        virtual ~Converter() {}

        /******************************************
        ** Converters
        *******************************************/
        /**
         * Converts an unsigned short to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const unsigned short &input){ return converters::convertUnsignedIntegral(input,this->buffer_begin,this->buffer_end); }
};

/*****************************************************************************
** Char String Converters [int]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,int> : public virtual converters::CharStringBuffer {
    public:
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {}
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {}

        virtual ~Converter() {}

        /**
         * Converts a int to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const int &input){ return converters::convertSignedIntegral(input,this->buffer_begin,this->buffer_end); }
};
/*****************************************************************************
** Char String Converters [unsigned int]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,unsigned int> : public virtual converters::CharStringBuffer {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {}
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {}

        virtual ~Converter() {}

        /******************************************
        ** Converters
        *******************************************/
        /**
         * Converts an unsigned int to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const unsigned int &input){ return converters::convertUnsignedIntegral(input,this->buffer_begin,this->buffer_end); }
};

/*****************************************************************************
** Char String Converters [long]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,long> : public virtual converters::CharStringBuffer {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {}
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {}

        virtual ~Converter() {}

        /******************************************
        ** Converters
        *******************************************/
        /**
         * Converts a long to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const long &input){ return converters::convertSignedIntegral(input,this->buffer_begin,this->buffer_end); }
};
/*****************************************************************************
** Char String Converters [unsigned long]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,unsigned long> : public virtual converters::CharStringBuffer {
    public:
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {}
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {}

        virtual ~Converter() {}

        /**
         * Converts an unsigned long to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const unsigned long &input){ return converters::convertUnsignedIntegral(input,this->buffer_begin,this->buffer_end); }
};
/*****************************************************************************
** Char String Converters [long long]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,long long> : public virtual converters::CharStringBuffer {
    public:
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {}
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {}

        virtual ~Converter() {}

        /**
         * Converts a long long to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const long long &input){ return converters::convertSignedIntegral(input,this->buffer_begin,this->buffer_end); }
};
/*****************************************************************************
** Char String Converters [unsigned long long]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * This class uses a character buffer for the character strings that it
 * generates. Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,unsigned long long> : public virtual converters::CharStringBuffer {
    public:
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {
        }
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {
        }

        virtual ~Converter() {}

        /**
         * Converts an unsigned long long to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const unsigned long long &input){ return converters::convertUnsignedIntegral(input,this->buffer_begin,this->buffer_end); }
};
/*****************************************************************************
** Char String Converters [float]
*****************************************************************************/
/**
 * @brief Conversion to text format with character strings.
 *
 * This class uses a character buffer for the character strings that it
 * generates. If your buffer is not long enough, this will just
 * truncate the output to fit the buffer.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,float> : public virtual converters::CharStringBuffer {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * @brief Initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {
    		format_specifier[0] = '%';
    		format_specifier[1] = '.';
    		format_specifier[2] = 'x';
    		format_specifier[3] = 'f';
    		format_specifier[4] = 'f';
    		format_specifier[5] = '\0';
        }
        /**
         * @brief Initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {
    		format_specifier[0] = '%';
    		format_specifier[1] = '.';
    		format_specifier[2] = 'x';
    		format_specifier[3] = 'f';
    		format_specifier[4] = 'f';
    		format_specifier[5] = '\0';
        }

        virtual ~Converter() {}

        /******************************************
        ** Converters
        *******************************************/
        /**
         * @brief Convert the specified double to int.
         *
         * Converts an double to a char string held in the converter's buffer.
         * I'd like to one day have my own (perhaps faster?) or refined
         * implementation of snprintf but it will do for now.
         *
         * @param input : input value to be converted.
         * @param precision : number of decimal places to show (-1 to automagically allocate).
         * @returns char* : output text string.
         **/
        char* operator ()(const float &input, const int& precision = -1){
        	if ( precision < 0 ) {
                format_specifier[1] = 'f';
                format_specifier[2] = '\0';
        	} else if ( precision < 10 ) {
                format_specifier[1] = '.';
                format_specifier[2] = '0'+precision;
                format_specifier[3] = 'f';
                format_specifier[4] = '\0';
            } else if ( precision < 20 )  {
                format_specifier[1] = '.';
                format_specifier[2] = '1';
                format_specifier[3] = '0'+(precision - 10);
                format_specifier[4] = 'f';
            } else {
                format_specifier[1] = '.';
                format_specifier[2] = '2';
                format_specifier[3] = '0';
                format_specifier[4] = 'f';
            }

            snprintf(this->buffer_begin, this->buffer_end-this->buffer_begin, format_specifier, input);
            return this->buffer_begin;
//            return converters::convertDecimal<32>(input,this->buffer_begin,this->buffer_end);
        }
    private:
        char format_specifier[6];
};
/*****************************************************************************
** Char String Converters [double]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * This class uses a character buffer for the character strings that it
 * generates. If your buffer is not long enough, this will just
 * truncate the output to fit the buffer.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,double> : public virtual converters::CharStringBuffer {
    public:
        /******************************************
        ** C&D's
        *******************************************/
        /**
         * @brief Initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {
    		format_specifier[0] = '%';
    		format_specifier[1] = '.';
    		format_specifier[2] = 'x';
    		format_specifier[3] = 'f';
    		format_specifier[4] = 'f';
    		format_specifier[5] = '\0';
        }
        /**
         * @brief Initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {
    		format_specifier[0] = '%';
    		format_specifier[1] = '.';
    		format_specifier[2] = 'x';
    		format_specifier[3] = 'f';
    		format_specifier[4] = 'f';
    		format_specifier[5] = '\0';
        }

        virtual ~Converter() {}

        /******************************************
        ** Converters
        *******************************************/
        /**
         * @brief Convert the specified double to int.
         *
         * Converts an double to a char string held in the converter's buffer.
         * I'd like to one day have my own (perhaps faster?) or refined
         * implementation of snprintf but it will do for now.
         *
         * @param input : input value to be converted.
         * @param precision : number of decimal places to show (-1 to automagically allocate).
         * @returns char* : output text string.
         **/
        char* operator ()(const double &input, const int& precision = -1){
        	if ( precision < 0 ) {
                format_specifier[1] = 'f';
                format_specifier[2] = '\0';
        	} else if ( precision < 10 ) {
                format_specifier[1] = '.';
                format_specifier[2] = '0'+precision;
                format_specifier[3] = 'f';
                format_specifier[4] = '\0';
            } else if ( precision < 20 )  {
                format_specifier[1] = '.';
                format_specifier[2] = '1';
                format_specifier[3] = '0'+(precision - 10);
                format_specifier[4] = 'f';
            } else {
                format_specifier[1] = '.';
                format_specifier[2] = '2';
                format_specifier[3] = '0';
                format_specifier[4] = 'f';
            }
            snprintf(this->buffer_begin, this->buffer_end-this->buffer_begin, format_specifier, input);
            return this->buffer_begin;
//            return converters::convertDecimal<64>(input,this->buffer_begin,this->buffer_end);
        }
    private:
        char format_specifier[6];
};
/*****************************************************************************
** Char String Converters [bool]
*****************************************************************************/
/**
 * @brief Fast conversion to text format with character strings.
 *
 * Provides a fast conversion to text using character strings.
 *
 * Unlike some of the numeric converters, this uses an internal character
 * buffer as the output is of known lengths (outputs are either 'true' or 'false').
 * Ensure your converter does not go out of scope while using
 * the character strings otherwise your character strings will end up
 * pointing at rubbish.
 *
 * @sa Converter
 **/
template<>
class Converter<char*,bool> : public virtual converters::CharStringBuffer {
    public:
        /**
         * Constructor that initialises with an internal buffer.
         **/
        Converter() : converters::CharStringBuffer(6) {}

        virtual ~Converter() {}

        /**
         * Converts a bool to a char string held in the converter's buffer.
         * @param input : input value to be converted.
         * @returns char* : output text string.
         **/
        char* operator ()(const bool &input){
            *buffer_begin = '\0';
            input ? strcat(buffer_begin,"true") : strcat (buffer_begin,"false");
            return buffer_begin;
        }
};

/*****************************************************************************
 * Char String Converter Family
 ****************************************************************************/
/**
 * @brief Family of converters to character strings.
 *
 * This groups all the character string converters of fundamental types into a single
 * cohesive superclass. This is often more convenient, especially as each converter
 * will utilise a single buffer. This is important if doing many different character
 * string conversions in the same scope.
 *
 * @sa Converter
 **/
template <>
class Converter<char*,void>  :
    public Converter<char*,char>,
    public Converter<char*,short>,
    public Converter<char*,int>,
    public Converter<char*,long>,
    public Converter<char*,unsigned char>,
    public Converter<char*,unsigned short>,
    public Converter<char*,unsigned int>,
    public Converter<char*,unsigned long>,
    public Converter<char*,float>,
    public Converter<char*,double>,
    public Converter<char*,long long>,
    public Converter<char*,unsigned long long>,
    public Converter<char*,bool>
{
    public:
        /**
         * Constructor that initialises with an external buffer.
         * @param begin : character pointer that points to the start of the external buffer.
         * @param end :  character pointer that points to the end of the external buffer.
         **/
        Converter(char* begin, char* end) : converters::CharStringBuffer(begin,end) {
        }
        /**
         * Constructor that initialises with an internal buffer.
         * @param buffer_size : size of the buffer to create - if not supplied it initialises a buffer of size 250.
         *
		 * @exception StandardException : throws if it failed to allocate memory for the internal buffer [debug mode only].
         **/
        Converter(int buffer_size = 250) ecl_debug_throw_decl(StandardException) : converters::CharStringBuffer(buffer_size) {
        }

        virtual ~Converter() {}

        using Converter<char*,char>::operator();
        using Converter<char*,short>::operator();
        using Converter<char*,int>::operator();
        using Converter<char*,long>::operator();
        using Converter<char*,unsigned char>::operator();
        using Converter<char*,unsigned short>::operator();
        using Converter<char*,unsigned int>::operator();
        using Converter<char*,unsigned long>::operator();
        using Converter<char*,float>::operator();
        using Converter<char*,double>::operator();
        using Converter<char*,long long>::operator();
        using Converter<char*,unsigned long long>::operator();
        using Converter<char*,bool>::operator();
};

}; // Namespace ecl

#endif /*ECL_CONVERTERS_CONVERT_CHAR_STRINGS_HPP_*/
