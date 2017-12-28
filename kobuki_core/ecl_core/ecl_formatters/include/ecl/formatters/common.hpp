/**
 * @file /include/ecl/formatters/common.hpp
 *
 * @brief Common formatting definitions.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_FORMATTERS_COMMON_HPP_
#define ECL_FORMATTERS_COMMON_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Format Tags
*****************************************************************************/
/**
 * @brief Alignment tags for text formatting.
 *
 * Common alignment format tag for the formatters.
 **/
enum Alignment
{
    NoAlign,        /**< @brief No alignment used. **/
    LeftAlign,      /**< @brief Align to the left. **/
    RightAlign,     /**< @brief Align to the right. **/
    CentreAlign,    /**< @brief Align in the centre. **/
};

/*****************************************************************************
** Default precision widths
*****************************************************************************/
/*
 * These must match the default format precisions for each fundamental type.
 */
/**
 * @brief Precision width trait for fundamental types.
 *
 * Numerical metafunction used by the default insertion
 * operator to establish the width of a cell when a matrix is sent to
 * an output stream. This value will generally store the space required
 * by integers for decimal precision (i.e. none).
 */
template <typename Type> struct PrecisionWidth {
    public:
        static const int value = 0;
};

/**
 * @brief Precision width trait for floats.
 *
 * Numerical metafunction used by the default insertion operator to
 * establish the width of a cell when a matrix is sent to an output
 * stream. This value determines the number of cells required
 * for the default level of decimal precision (four decimal places).
 */
template <>
struct PrecisionWidth<float> {
    public:
        static const int value = 4;
};

/**
 * @brief Precision width trait for doubles.
 *
 * Numerical meta-function used by the default insertion operator to
 * establish the width of a cell when a matrix is sent to an output
 * stream. This value determines the number of cells required
 * for the default level of decimal precision (four decimal places).
 */
template <>
struct PrecisionWidth<double> {
    public:
        static const int value = 4;
};

/*****************************************************************************
* Format Classes
*****************************************************************************/
/**
 * @brief Primary template for all formatter classes.
 *
 * Most formatters specialise from this primary template, however if no
 * specialisation exists, this will look up the class to be formatted
 * interface for a formatter type to use (T::Formatter). There are many
 * basic type specialisations in the formatters namespace.
 *
 * @sa ecl::formatters.
 *
 **/
template <typename T>
class Format : public T::Formatter {};

}; // namespace ecl

#endif /*ECL_FORMATTERS_COMMON_HPP_*/
