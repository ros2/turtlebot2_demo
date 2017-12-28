/**
 * @file /include/ecl/streams/text_streams/output_text_stream.hpp
 *
 * @brief Output text streaming interface and implementation.
 *
 * Provides an output text streaming interface that can be connected
 * to output devices.
 *
 * @date October 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_OUTPUT_TEXT_STREAM_HPP_
#define ECL_STREAMS_OUTPUT_TEXT_STREAM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstring>
#include <string>
#include <ecl/config/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/converters.hpp>
#include <ecl/concepts/devices.hpp>
#include "../manipulators/manipulator.hpp"
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace interfaces {

/*****************************************************************************
** Interface [OutputTextStream]
*****************************************************************************/

/**
 * @brief Parent template for output text streams.
 *
 * This template is defined so that is can provide an empty
 * interface if the calling device is not in fact an output device.
 * The actual output interface is defined in the boolean true specialisation.
 *
 * @tparam Device : underlying device connected to the stream.
 * @tparam OutputDevice : boolean indicating if the device is an output device or not.
 *
 * @sa OutputTextStream<Device,true>
 */
template <typename Device, bool OutputDevice = true >
class ECL_PUBLIC OutputTextStream {};

/**
 * @brief Output text stream interface.
 *
 * Defines the appropriate functionality required for output text streams.
 *
 * @tparam Device : this must be a class that satisfies the
 * output device concept (refer to the documentation in ecl_concepts for
 * details).
 *
 * <b>Usage:</b>
 *
 * - Instantiate
 * - Open the underlying device
 * - Stream
 *
 * @code
 * TextStream<OFile> ofstream;
 * ofstream.device().open("dudes.txt",New);
 * ofstream << "dudes " << 32.36;
 * ofstream.flush();
 * @endcode
 **/
template <typename Device>
class ECL_PUBLIC OutputTextStream<Device,true> : public virtual BaseTextStream<Device> {
    public:
        /**
         * @brief Connects the stream to an output device.
         *
         * Connects the text stream to the associated output device. Use
         * the device() handle to properly open the device, e.g.
         *
         * @code
         * OutputTextStream<OFile> ofstream;
         * ofstream.device().open("dudes.txt",New);
         * @endcode
         **/
        OutputTextStream() :
        	toCharString(30) // character buffer size
		{
            ecl_compile_time_concept_check(OutputCharDeviceConcept<Device>);
		};

        virtual ~OutputTextStream() {}

        OutputTextStream<Device>& operator << ( const char &c ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const char *s ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const std::string &s ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const short &i ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const int &i ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const long &i ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const long long &i ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const unsigned short &i ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const unsigned int &i ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const unsigned long &i ) ecl_assert_throw_decl(ecl::StandardException);
		OutputTextStream<Device>& operator << ( const unsigned long long &i ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const bool b ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const float &f ) ecl_assert_throw_decl(ecl::StandardException);
        OutputTextStream<Device>& operator << ( const double &d ) ecl_assert_throw_decl(ecl::StandardException);

        /*********************
         * Friend Manipulators
         *********************/
        template <typename Action>
        OutputTextStream<Device>& operator << ( ecl::Manipulator<Action> &manipulator ) ecl_assert_throw_decl(ecl::StandardException)
        {
        	manipulator.insert(*this);
        	return *this;
        }

        /**
         * Provides a streaming operator for handling output textstream manipulators. Manipulators are simple
         * functions that accept the stream as input and manipulate it in a minor way. They can hold no state.
         * The most common example of such a manipulator is the endln manipulator. This manipulator works in
         * the same fashion as the c++ endl manipulator.
         **/
//        friend interfaces::OutputTextStream& operator << (interfaces::OutputTextStream& ostream, void (*manipulator)(interfaces::OutputTextStream&) )
//        {
//            (*manipulator)(ostream);
//            return ostream;
//        }

        /*********************
          ** Buffer functionality
        **********************/
         void flush() ecl_assert_throw_decl(ecl::StandardException);

    private:
        ecl::Converter<char*> toCharString;
};

/*****************************************************************************
** Implementation [OutputTextStream]
*****************************************************************************/

/**
 * @brief Sends an unformatted char to the stream.
 * Sends an unformatted char to the stream.
 * @param c : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator<< ( const char& c ) ecl_assert_throw_decl(ecl::StandardException) {
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
	this->io_device.write(c);
    return *this;
}
/**
 * @brief Sends an unformatted char string to the stream.
 * Sends an unformatted char string to the stream.
 *
 * @param s : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const char *s ) ecl_assert_throw_decl(ecl::StandardException) {
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    this->io_device.write(s,strlen(s));
    return *this;
}

/**
 * @brief Sends an unformatted string to the stream.
 * Sends an unformatted string to the stream.
 *
 * @param s : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const std::string &s ) ecl_assert_throw_decl(ecl::StandardException) {
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    this->io_device.write(&s[0],s.size());
    return *this;
}
/**
 * @brief Sends an unformatted short to the stream.
 * Sends an unformatted short to the stream.
 * @param i : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const short &i ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(i);
    return *this;
}
/**
 * @brief Sends an unformatted int to the stream.
 * Sends an unformatted int to the stream.
 * @param i : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const int &i ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(i);
    return *this;
}

/**
 * @brief Sends an unformatted long to the stream.
 * Sends an unformatted long to the stream.
 * @param i : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const long &i ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(i);
    return *this;
}

/**
 * @brief Sends an unformatted long long to the stream.
 * Sends an unformatted long long to the stream.
 * @param i : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const long long &i ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(i);
    return *this;
}

/**
 * @brief Sends an unformatted unsigned short to the stream.
 * Sends an unformatted unsigned short to the stream.
 * @param i : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const unsigned short &i ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(i);
    return *this;
}

/**
 * @brief Sends an unformatted unsigned int to the stream.
 * Sends an unformatted unsigned int to the stream.
 * @param i : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const unsigned int &i ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(i);
    return *this;
}

/**
 * @brief Sends an unformatted unsigned long to the stream.
 * Sends an unformatted unsigned long to the stream.
 * @param i : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const unsigned long &i ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(i);
    return *this;
}
/**
 * @brief Sends an unformatted unsigned long long to the stream.
 * Sends an unformatted unsigned long long to the stream.
 * @param i : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const unsigned long long &i ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(i);
    return *this;
}
/**
 * @brief Sends a boolean to the stream.
 * Sends a boolean to the stream.
 * @param b : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const bool b ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    if ( b ) { *this << "true"; } else { *this << "false"; };
    return *this;
}

/**
 * Sends an unformatted float to the stream.
 * @param f : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation
 **/
template <typename Device>
OutputTextStream<Device>& OutputTextStream<Device,true>::operator << ( const float &f ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(f);
    return *this;
}

/**
 * Sends an unformatted double to the stream.
 * @param d : input value being sent to the stream
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 * @return OutputTextStream : returns the stream for further manipulation.
 **/
template <typename Device>
OutputTextStream<Device>&  OutputTextStream<Device,true>::operator << ( const double &d ) ecl_assert_throw_decl(ecl::StandardException)
{
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
    *this << toCharString(d);
    return *this;
}

/**
 * @brief Flushes the underlying device's buffer.
 *
 * Flushes the underlying device's buffer.
 *
 * @exception ecl::StandardException : throws if the underlying device has not been opened [debug mode only].
 **/
template <typename Device>
void OutputTextStream<Device,true>::flush() ecl_assert_throw_decl(ecl::StandardException) {
	ecl_assert_throw(this->io_device.open(),ecl::StandardException(LOC,OpenError,"The underlying stream device is not open."));
	this->io_device.flush();
}

} // namespace interfaces
} // namespace ecl

#endif /* ECL_STREAMS_OUTPUT_TEXT_STREAM_HPP_ */
