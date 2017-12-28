/**
 * @file /include/ecl/concepts/devices.hpp
 *
 * @brief Defines validating functionality for the <i>device</i> concepts.
 *
 * Defines validating functionality for the <i>device</i> concepts.
 *
 * @date October 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONCEPTS_DEVICES_HPP_
#define ECL_CONCEPTS_DEVICES_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Concept [InputCharDevice]
*****************************************************************************/

/**
 * @brief Validates functionality for the @ref devicesConcept "input char device concept".
 *
 * This checks if a device can handle input of char types.
 **/
template <typename Implementation>
class InputCharDeviceConcept {
public:
	/**
	 * @brief Implements a concept test for input devices.
	 *
	 * The following conditions are required by input devices:
	 *
	 * - long read(char &c)   : read a single character from the device buffer.
	 * - long read(char *s, unsigned long n)   : read a char buffer from the device buffer.
	 */
	ecl_compile_time_concept_test(InputCharDeviceConcept) {
		char c;
		char buffer[10];
		long no_read;
		bool result;
		result = input_char_device.open();
		no_read = input_char_device.read(c);
		no_read = input_char_device.read(buffer,10);
                (void) no_read; // remove set but unused warnings
                (void) result; // remove set but unused warnings
	}

private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation input_char_device;
};

/*****************************************************************************
** Concept [OutputCharDevice]
*****************************************************************************/
/**
 * @brief Validates functionality for the @ref devicesConcept "output char device concept".
 *
 * This checks if a device can handle output of char types.
 **/
template <typename Implementation>
class OutputCharDeviceConcept {
public:
	/**
	 * @brief Implements a concept test for output char devices.
	 *
	 * The following conditions are required by output char devices:
	 *
	 * - long write(char &c)                  : write a single character to the device buffer.
	 * - long write(char *s, unsigned long n) : write a char buffer to the device buffer.
	 * - void flush()  						  : a method for flushing the output device buffer to the device.
	 */
	ecl_compile_time_concept_test(OutputCharDeviceConcept)
	{
		long no_written;
		char write_buffer[] = { 'd', 'u', 'd', 'e', '\n' };
		bool result;
		result = output_char_device.open();
		no_written = output_char_device.write('\n');
		no_written = output_char_device.write(write_buffer,5);
		output_char_device.flush();
                (void) no_written; // remove set but unused warnings
                (void) result; // remove set but unused warnings
	}

private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation output_char_device;
};

/*****************************************************************************
** Concept [InputOutputByteDevice]
*****************************************************************************/

/**
 * @brief Validates functionality for the @ref devicesConcept "input-output char device concept".
 *
 * This checks if a device can handle input-output of char types.
 **/
template <typename Implementation>
class InputOutputCharDeviceConcept {
public:
	/**
	 * @brief Implements a concept test for input-output byte devices.
	 *
	 * The following conditions are required by input-output byte devices:
	 *
	 * - long read(char &c)                   : read a single character from the device buffer.
	 * - long read(char *s, unsigned long n)  : read a char buffer from the device buffer.
	 * - long write(char &c)                  : write a single character to the device buffer.
	 * - long write(char *s, unsigned long n) : write a char buffer to the device buffer.
	 * - void flush()  						  : a method for flushing the output device buffer to the device.
	 */
	ecl_compile_time_concept_test(InputOutputCharDeviceConcept)
	{
		char c;
		char read_buffer[10];
		long no_read;
		no_read = input_output_char_device.read(c);
		no_read = input_output_char_device.read(read_buffer,10);
		long no_written;
		char write_buffer[] = { 'd', 'u', 'd', 'e', '\n' };
		no_written = input_output_char_device.write('\n');
		no_written = input_output_char_device.write(write_buffer,5);
		input_output_char_device.flush();
	}
private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation input_output_char_device;
};

/*****************************************************************************
** Concept [InputByteDevice]
*****************************************************************************/

/**
 * @brief Validates functionality for the @ref devicesConcept "input byte device concept".
 *
 * This checks if a device can handle input of all char types
 * (char, signed char, unsigned char).
 **/
template <typename Implementation>
class InputByteDeviceConcept {
public:
	/**
	 * @brief Implements a concept test for input devices.
	 *
	 * The following conditions are required by input devices:
	 *
	 * - long read(char &c)   : read a single character from the device buffer.
	 * - long read(char *s, unsigned long n)   : read a char buffer from the device buffer.
	 *
	 * Along with the same function implemented for signed and unsigned char types.
	 */
	ecl_compile_time_concept_test(InputByteDeviceConcept) {
		signed char sc;
		signed char sbuffer[10];
		char c;
		char buffer[10];
		unsigned char usc;
		unsigned char usbuffer[10];
		long no_read;
		bool result;
		result = input_byte_device.isOpen();
		no_read = input_byte_device.read(sc);
		no_read = input_byte_device.read(sbuffer,10);
		no_read = input_byte_device.read(c);
		no_read = input_byte_device.read(buffer,10);
		no_read = input_byte_device.read(usc);
		no_read = input_byte_device.read(usbuffer,10);
	}

private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation input_byte_device;
};

/*****************************************************************************
** Concept [OutputByteDevice]
*****************************************************************************/
/**
 * @brief Validates functionality for the @ref devicesConcept "output byte device concept".
 *
 * This checks if a device can handle output of all char types
 * (char, signed char, unsigned char).
 **/
template <typename Implementation>
class OutputByteDeviceConcept {
public:
	/**
	 * @brief Implements a concept test for output byte devices.
	 *
	 * The following conditions are required by output byte devices:
	 *
	 * - long write(char &c)                  : write a single character to the device buffer.
	 * - long write(char *s, unsigned long n) : write a char buffer to the device buffer.
	 * - void flush()  						  : a method for flushing the output device buffer to the device.
	 *
	 * Along with the write function implemented for signed and unsigned char types.
	 */
	ecl_compile_time_concept_test(OutputByteDeviceConcept)
	{
		long no_written;
		signed char sc = '\n';
		char c = '\n';
		unsigned char usc = 0x01;
		signed char s_write_buffer[] = { 'd', 'u', 'd', 'e', '\n' };
		char write_buffer[] = { 'd', 'u', 'd', 'e', '\n' };
		unsigned char us_write_buffer[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
		bool result;
		result = output_byte_device.isOpen();
		no_written = output_byte_device.write(sc);
		no_written = output_byte_device.write(c);
		no_written = output_byte_device.write(usc);
		no_written = output_byte_device.write(s_write_buffer,5);
		no_written = output_byte_device.write(write_buffer,5);
		no_written = output_byte_device.write(us_write_buffer,5);
		output_byte_device.flush();
	}

private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation output_byte_device;
};

/*****************************************************************************
** Concept [InputOutputByteDevice]
*****************************************************************************/

/**
 * @brief Validates functionality for the @ref devicesConcept "input-output byte device concept".
 *
 * This checks if a device can handle input-output of all char types
 * (char, signed char, unsigned char).
 **/
template <typename Implementation>
class InputOutputByteDeviceConcept {
public:
	/**
	 * @brief Implements a concept test for input-output byte devices.
	 *
	 * The following conditions are required by input-output byte devices:
	 *
	 * - long read(char &c)                   : read a single character from the device buffer.
	 * - long read(char *s, unsigned long n)  : read a char buffer from the device buffer.
	 * - long write(char &c)                  : write a single character to the device buffer.
	 * - long write(char *s, unsigned long n) : write a char buffer to the device buffer.
	 * - void flush()  						  : a method for flushing the output device buffer to the device.
	 *
	 * Along with the read/write functions implemented for signed and unsigned char types.
	 */
	ecl_compile_time_concept_test(InputOutputByteDeviceConcept)
	{
		signed char sc;
		signed char sbuffer[10];
		char c;
		char buffer[10];
		unsigned char usc;
		unsigned char usbuffer[10];
		long no_read;
		bool result;
		result = input_output_byte_device.isOpen();
		no_read = input_output_byte_device.read(sc);
		no_read = input_output_byte_device.read(sbuffer,10);
		no_read = input_output_byte_device.read(c);
		no_read = input_output_byte_device.read(buffer,10);
		no_read = input_output_byte_device.read(usc);
		no_read = input_output_byte_device.read(usbuffer,10);

		long no_written;
		sc = '\n';
		c = '\n';
		usc = 0x01;
		signed char s_write_buffer[] = { 'd', 'u', 'd', 'e', '\n' };
		char write_buffer[] = { 'd', 'u', 'd', 'e', '\n' };
		unsigned char us_write_buffer[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
		no_written = input_output_byte_device.write(sc);
		no_written = input_output_byte_device.write(c);
		no_written = input_output_byte_device.write(usc);
		no_written = input_output_byte_device.write(s_write_buffer,5);
		no_written = input_output_byte_device.write(write_buffer,5);
		no_written = input_output_byte_device.write(us_write_buffer,5);
		input_output_byte_device.flush();
	}
private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation input_output_byte_device;
};

}; // namespace ecl

#endif /* ECL_CONCEPTS_DEVICES_HPP_ */
