/**
 * @file /include/ecl/converters/converter.hpp
 *
 * @brief Primary templates for the family of converter classes.
 *
 * @date April, 2009
 **/
/*****************************************************************************
** PreProcessor
*****************************************************************************/

#ifndef ECL_CONVERTERS_CONVERTER_HPP_
#define ECL_CONVERTERS_CONVERTER_HPP_

/*****************************************************************************
** Include
*****************************************************************************/

#include <ecl/errors/handlers.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
/**
 * @cond DO_NOT_DOXYGEN
 */
namespace converters {

class ConverterBase {
public:
	ConverterBase() : error_handler(ecl::NoError) {};
	virtual Error error() const { return error_handler; }
protected:
	Error error_handler;
};
} // namespace converters
/**
 * @endcond
 */
/*****************************************************************************
** Converter Interface
*****************************************************************************/
/**
 * @brief Primary template and general fallback for converter classes.
 *
 * This is the primary template for all converter classes. It
 * also provides a default fallback for a single type conversion where both
 * input and output types are defined in as template parameters.
 *
 * @sa Converter<Output,void>
 *
 **/
template <typename Output, typename Input = void>
class Converter : public converters::ConverterBase {
public:
	virtual ~Converter() {}
	Output operator()(const Input &input);
};

/**
 * @brief Primary template for the output-designated only converters.
 *
 * This is the primary template for the sub-family of converters which
 * only designate the output in the template argument. These are
 * designed to incorporate a group of conversions to the specified output type
 * (convenience class).
 **/
template <typename Output>
class Converter<Output,void> : public converters::ConverterBase {
public:
    virtual ~Converter() {}

	template <typename Input>
	Output operator()(const Input &input);
};

/*****************************************************************************
** Converter Implementation
*****************************************************************************/
/**
 * The default fallback for converting a specific input type to a specific
 * output type. This looks inside the output template
 * argument's class for the required converter.
 *
 * @param input : the input value to be converted.
 * @return Output : the converted type.
 **/
template <typename Output, typename Input>
Output Converter<Output,Input>::operator ()(const Input &input)
{
    return typename Output::Converter()(input);
};

/**
 * The generalised fallback for converting various input types to a
 * specific output type. This looks inside the output template
 * argument's class for the required converter.
 *
 * @param input : the input value to be converted.
 * @return Output : the converted type.
 **/
template <typename Output>
template <typename Input>
Output Converter<Output,void>::operator ()(const Input &input)
{
    return typename Output::Converter()(input);
};

}; // Namespace ecl


#endif /* ECL_CONVERTERS_CONVERTER_HPP_ */
