/**
 * @file /ecl_linear_algebra/include/ecl/linear_algebra/formatters.hpp
 *
 * @brief Formatters for eigen types.
 *
 * @date September 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_LINEAR_ALGEBRA_FORMATTERS_HPP_
#define ECL_LINEAR_ALGEBRA_FORMATTERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/formatters.hpp>
#include <ecl/mpl/enable_if.hpp>
#include <ecl/type_traits/fundamental_types.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Eigen {

/*****************************************************************************
** Interface [MatrixFormatter]
*****************************************************************************/

/**
 * @brief Parent for matrix formatters - used to negate unimplemented formatters.
 *
 * This is not to be used directly, rather the specialisations will implement
 * the matrix formatter.
 */
template <typename Derived, typename Scalar, typename Enable = void>
class MatrixFormatter {
private:
	MatrixFormatter() {}
};

/*****************************************************************************
** Interface [FloatMatrixFormatter]
*****************************************************************************/

/**
 * @brief Formatter for Eigen matrix type
 *
 * I want make it formatting any types of eigen
 *
 * @tparam Derived : MatrixBase's type which specify the matrix as vector, matrix with any storage type
 *
 * @todo
 * specialise the formatting; now it just support the float type only.
 */
template<typename Derived>
class FloatMatrixFormatter {
public:
	/******************************************
	** C&D's
	*******************************************/
	/**
	 * @brief Default constructor.
	 *
	 * Initialises the format tags for width, and precision.
	 * @param w : width (default - no width constraints)
	 * @param p : the number of decimal places of precision (default - 4)
	 **/
	FloatMatrixFormatter(const int &w = -1, const unsigned int &p = 2) :
		tmp_formatting(false),
		ready_to_format(false),
		_matrix(NULL)
	{
		format.width(w);
		format.precision(p);
	}
	virtual ~FloatMatrixFormatter() {}

	/******************************************
	** Configuration
	*******************************************/
	/**
	 * @brief Sets the precision format parameter.
	 *
	 * Sets the precision format parameter.
	 *
	 * @param p : the number of decimal places of precision.
	 * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
	 **/
	FloatMatrixFormatter<Derived>& precision( const unsigned int &p ) {
		format.precision(p);
		return *this;
	}

	/**
	 * @brief Sets the width format parameter.
	 *
	 * Sets the width format parameter.
	 *
	 * @param w : the width to use for inserted floats (-1 is no width constraint).
	 * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
	 **/
	FloatMatrixFormatter<Derived>& width( const int &w ) {
		format.width(w);
		return *this;
	}

	/**
	 * @brief Returns the current precision setting.
	 * @return unsigned int : the precision value.
	 */
	unsigned int precision() { return format.precision(); }

	/**
	 * @brief Returns the current width setting.
	 * @return int : the witdh value (-1 for no width constraint).
	 */
	int width() { return format.width(); }

	/******************************************
	** Format a value
	*******************************************/
	/**
	 * @brief Format a matrix with permanently stored precision/width.
	 *
	 * This function directly formats the specified input value
	 * with the stored settings.
	 * @code
	 * Vector2d v; v << 1.0, 2.0;
	 * Format<Vector2d> format;
	 * cout << format(v) << endl;
	 * @endcode
	 * @param matrix : the matrix to be formatted (gets temporarily stored as a pointer).
	 * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
	 **/
	FloatMatrixFormatter< Derived >& operator() (const Derived & matrix ) {
		_matrix = &matrix;
		ready_to_format = true;
		return (*this);
	}
	/**
	 * @brief Format a matrix with temporarily specified precision/width.
	 *
	 * This function directly formats the specified input value
	 * and temporary settings.
	 *
	 * @code
	 * Vector2d v; v << 1.0, 2.0;
	 * Format<Vector2d> format;
	 * cout << format(v,3,-1) << endl; // precision of 3 and no width constraint
	 * @endcode
	 *
	 * @param matrix : the matrix to be formatted (gets temporarily stored as a pointer).
	 * @param w : the width to use for inserted floats (-1 is no width constraint).
         * @param p : the number of decimal places of precision.
	 * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
	 **/
	FloatMatrixFormatter< Derived >& operator() (const Derived & matrix, const int &w, const unsigned int &p) {
		_matrix = &matrix;
		tmp_precision = p;
		tmp_width = w;
		tmp_formatting = true;
		ready_to_format = true;
		return (*this);
	}

	/**
	 * @brief Stream the formatter.
	 *
	 * Insertion operator for sending the formatter  to an output stream.
	 *
	 * @param ostream : the output stream.
	 * @param formatter : the formatter to be inserted.
	 * @tparam OutputStream : the type of the output stream to be inserted into.
	 * @tparam Derived : matrix type.
	 * @return OutputStream : continue streaming with the updated output stream.
	 *
	 * @exception StandardException : throws if the formatter has un-specified _matrix [debug mode only]
	 */
	template <typename OutputStream, typename Derived_ >
	friend OutputStream& operator << ( OutputStream& ostream, FloatMatrixFormatter<Derived_> & formatter ) ecl_assert_throw_decl(ecl::StandardException);

    private:
		ecl::Format<typename Derived::Scalar> format;
		int tmp_width;
		unsigned int tmp_precision;
		bool tmp_formatting;
		bool ready_to_format;
		const Derived *_matrix;
};

template <typename OutputStream, typename Derived_ >
OutputStream& operator << (OutputStream& ostream, FloatMatrixFormatter< Derived_ > & formatter ) ecl_assert_throw_decl(ecl::StandardException)
{
    ecl_assert_throw( formatter._matrix, ecl::StandardException(LOC,ecl::UsageError,"The formatter cannot print any data - "
            "_matrix was not initialised "
            "please pass the your matrix through () operator") );
    ecl_assert_throw(formatter.ready_to_format, ecl::StandardException(LOC,ecl::UsageError,"The formatter cannot print any data - "
            "either there is no data available, or you have tried to use the "
            "formatter more than once in a single streaming operation. "
            "C++ produces unspecified results when functors are used multiply "
            "in the same stream sequence, so this is not permitted here.") );


    if ( formatter.ready_to_format ) {
    	unsigned int prm_precision = formatter.format.precision();;
    	int prm_width = formatter.format.width();
		if ( formatter.tmp_formatting ) {
			formatter.format.precision(formatter.tmp_precision);
			formatter.format.width(formatter.tmp_width);
		}

		/*********************
		** Stream Matrix
		**********************/
		// write matrix data
		// @note norma matrix allows matrix(i,j) however sparse matrix does not.
		// So i put coeff(i,j) function which can be used for accessing each element for both of sparse and dense.
		int rows = formatter._matrix->rows();
		int cols = formatter._matrix->cols();
		for( int i=0; i<rows; i++ )
		{
			for( int j=0; j<cols; j++ )
			{
				ostream <<  formatter.format(formatter._matrix->coeff( i, j )) << "  ";
			}
			if ( rows != 1 ) {  // special condition for row vectors so we can do inline the formatting.
			  ostream << "\n";
			}
		}

		if ( formatter.tmp_formatting ) {
			formatter.format.precision(prm_precision);
			formatter.format.width(prm_width);
			formatter.tmp_formatting = false;
		}
        formatter.ready_to_format = false;
    }
    return ostream;
}

/*****************************************************************************
** Interface [MatrixFormatter][Float Types]
*****************************************************************************/

template <typename Derived, typename Scalar>
class MatrixFormatter<Derived, Scalar, typename ecl::enable_if< ecl::is_float<Scalar> >::type> : public FloatMatrixFormatter<Derived> {
public:
	MatrixFormatter(const int &w = -1, const unsigned int &p = 2) : FloatMatrixFormatter<Derived>(w, p) {};
};

} // namespace Eigen

#endif /* ECL_LINEAR_ALGEBRA_FORMATTERS_HPP_ */
