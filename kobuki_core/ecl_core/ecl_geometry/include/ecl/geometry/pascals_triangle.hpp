/**
 * @file /include/ecl/geometry/pascals_triangle.hpp
 *
 * @brief Templatised specialisations for pascal's triangle.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_PASCALS_HPP_
#define ECL_GEOMETRY_PASCALS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/containers/array.hpp>
#include <ecl/formatters/common.hpp>
#include <ecl/formatters/number.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Class [PascalsTriangle]
*****************************************************************************/

/**
 * @brief Holds the coefficients for pascal's triangle up to row N.
 *
 * Stores the coefficients of all rows (determined in diagonal order)
 * up until row N - i.e. all coefficients up to and including those
 * for (x+y)^0 to (x+y)^N. Coefficients are stored as a sequence of
 * rows going from top to bottom right (of the triangle)
 * and moving left. For example, for N = 5, the representation
 * in the array is as shown below:
 * @code
 * (*this) = 1,1,1,1,1,1,
 *           1,2,3,4,5,
 *           1,3,6,10,
 *           1,4,10,
 *           1,5,
 *           1;
 * @endcode
 *
 * @tparam N : calculate the triangle to the N-th power.
 *
 * @sa @ref polynomialsGeometry "Math::Polynomials".
 */
template <int N>
class ECL_PUBLIC PascalsTriangle {
    public:
        /*********************
        ** C&D
        **********************/
        PascalsTriangle();
        virtual ~PascalsTriangle() {};

        /*********************
        ** Iterators
        **********************/
        typedef typename Array<int,(N+2)*(N+1)/2>::const_iterator const_iterator;  /**< @brief Utilise the array's iterator for parsing the triangle. **/
        const_iterator begin(unsigned int index = 0) const;
        const_iterator end(unsigned int index = 0) const;

        /*********************
        ** Streaming
        **********************/
        template <typename OutputStream, int PowerN>
        friend OutputStream& operator<<(OutputStream &ostream, const PascalsTriangle<PowerN> &triangle);

    private:
        Array<int,(N+2)*(N+1)/2> coefficients;
};
/*****************************************************************************
** Specialisations [Pascals Triangle][3]
*****************************************************************************/
/**
 * @brief Holds the coefficients of pascal's triangle up to n = 3.
 *
 * Stores the coefficients of all diagonals up until row 3 - i.e.
 * all coefficients up to and including those for (x+y)^0 to (x+y)^3.
 * This is used fairly often, so defined here to avoid the slow
 * calculation of the general case.
 *
 * @sa @ref PascalsTriangle, @ref polynomialsGeometry "Math::Polynomials".
 */
template <>
class ECL_PUBLIC PascalsTriangle<3> {
    public:
        /*********************
        ** C&D
        **********************/
        /**
         * @brief Default constructor.
         *
         * This constructor speeds things up
         * by initialising the triangle internally with the coefficients directly.
         */
        PascalsTriangle();
        virtual ~PascalsTriangle() {};

        /*********************
        ** Iterators
        **********************/
        typedef Array<int,10>::const_iterator const_iterator; /**< @brief Utilise the array's iterator for parsing the triangle. **/
        /**
         * @brief Iterator generator for diagonals of pascals triangle [begin].
         *
         * Return a const iterator pointing to the first element of the specified diagonal.
         * @param index : the diagonal to be iterated.
         * @return const_iterator : constant iterator pointing pointing to the first element of the specified diagonal/row.
         **/
        const_iterator begin(unsigned int index = 0) const;
        /**
         * @brief Iterator generator for diagonals of pascals triangle [begin].
         *
         * Return a const iterator just past the last element of the specified diagonal.
         * @param index : the diagonal to be iterated.
         * @return const_iterator : constant iterator pointing just beyond the last of the specified diagonal/row.
         **/
        const_iterator end(unsigned int index = 0) const;

        /*********************
        ** Streaming
        **********************/
        template <typename OutputStream>
        friend OutputStream& operator<<(OutputStream &ostream, const PascalsTriangle<3> &triangle);

    private:
        Array<int,10> coefficients;
};

/*****************************************************************************
** Specialisations [Pascals Triangle][5]
*****************************************************************************/
/**
 * @brief Holds the coefficients of pascal's triangle up to n = 5.
 *
 * Stores the coefficients of all diagonals up until row 5 - i.e.
 * all coefficients up to and including those for (x+y)^0 to (x+y)^5.
 * This is used fairly often, so defined here to avoid the slow
 * calculation of the general case.
 *
 * @sa @ref PascalsTriangle, @ref polynomialsGeometry "Math::Polynomials".
 */
template <>
class ECL_PUBLIC PascalsTriangle<5> {
    public:
        /*********************
        ** C&D
        **********************/
        /**
         * @brief Default constructor.
         *
         * This constructor speeds things up
         * by initialising the triangle internally with the coefficients directly.
         */
        PascalsTriangle();
        virtual ~PascalsTriangle() {};

        /*********************
        ** Iterators
        **********************/
        typedef Array<int,21>::const_iterator const_iterator; /**< @brief Utilise the array's iterator for parsing the triangle. **/
        /**
         * @brief Iterator generator for diagonals of pascals triangle [begin].
         *
         * Return a const iterator pointing to the first element of the specified diagonal.
         * @param index : the diagonal to be iterated.
         * @return const_iterator : constant iterator pointing pointing to the first element of the specified diagonal/row.
         **/
        const_iterator begin(unsigned int index = 0) const;
        /**
         * @brief Iterator generator for diagonals of pascals triangle [begin].
         *
         * Return a const iterator just past the last element of the specified diagonal.
         * @param index : the diagonal to be iterated.
         * @return const_iterator : constant iterator pointing just beyond the last of the specified diagonal/row.
         **/
        const_iterator end(unsigned int index = 0) const;

        /*********************
        ** Streaming
        **********************/
        template <typename OutputStream>
        friend OutputStream& operator<<(OutputStream &ostream, const PascalsTriangle<5> &triangle);

    private:
        Array<int,21> coefficients;
};

/*****************************************************************************
** Implementation [Pascals Triangle]
*****************************************************************************/
/**
 * @brief Default constructor.
 *
 * This computes all coefficients of the triangle up to row N.
 */
template <int N>
PascalsTriangle<N>::PascalsTriangle() {
    int counter = 0;
    for (int i = N+1; i > 0; --i ) {
        for (int j = 0; j < i; ++j ) {
            if ( ( i == N+1 ) || ( j == 0 ) ) {
                coefficients[counter] = 1;
            } else {
                coefficients[counter] =  coefficients[counter-1] + coefficients[counter-(i+1)];
            }
            ++counter;
        }
    }
}

/**
 * @brief Iterator generator for diagonals of pascals triangle [begin].
 *
 * Return a const iterator pointing to the first element of the specified diagonal.
 * @param index : the diagonal to be iterated.
 * @return const_iterator : constant iterator pointing pointing to the first element of the specified diagonal.
 **/
template <int N>
typename PascalsTriangle<N>::const_iterator PascalsTriangle<N>::begin(unsigned int index) const {
    int coeff_index = 0;
    for (unsigned int i = 0; i < index; ++i ) {
        coeff_index += N+1-i;
    }
    return const_iterator( &coefficients[coeff_index] );
}
/**
 * @brief Iterator generator for diagonals of pascals triangle [end].
 *
 * Return a const iterator just past the last element of the specified diagonal.
 * @param index : the diagonal to be iterated.
 * @return const_iterator : constant iterator pointing just beyond the last of the specified diagonal.
 **/
template <int N>
typename PascalsTriangle<N>::const_iterator PascalsTriangle<N>::end(unsigned int index) const {
    int coeff_index = 0;
    for (unsigned int i = 0; i <= index; ++i ) {
        coeff_index += N+1-i;
    }
    coeff_index -= 1; // dont want to call beyond the array limit.
    return const_iterator( (&coefficients[coeff_index])+1);
}

/*****************************************************************************
** Streaming
*****************************************************************************/

/**
 * @brief Streaming output insertion operator for for pascal triangles.
 *
 * Streaming output insertion operator for for pascal triangles.
 *
 * @tparam OutputStream : the type of stream being used.
 * @tparam PowerN : the order of the pascal's triangle being inserted.
 *
 * @param ostream : the stream to send the output to.
 * @param triangle : the pascal triangle object.
 * @return OutputStream : the output stream.
 */
template <typename OutputStream, int PowerN>
OutputStream& operator << ( OutputStream &ostream, const PascalsTriangle<PowerN> &triangle)
{
    Format<int> format(2+PowerN/5,CentreAlign); // Rough hack to get a good auto-sizing working. Might blow up for large N.
    int counter = 0;
    for (int i = PowerN+1; i > 0; --i ) {
        for (int j = 0; j < i; ++j ) {
            ostream << format(triangle.coefficients[counter]) << " ";
            ++counter;
        }
        ostream << "\n";
    }
    ostream.flush();

    return ostream;
}

/**
 * @brief Insertion operator for streaming output from pascal's triangle of degree 3.
 *
 * Insertion operator for streaming output from pascal's triangle.
 *
 * @param ostream : the stream to send the output to.
 * @param triangle : the pascal triangle object.
 * @return OutputStream : the output stream.
 */
template <typename OutputStream>
OutputStream& operator << ( OutputStream &ostream, const PascalsTriangle<3> &triangle)
{
    Format<int> format(2,CentreAlign);
    int counter = 0;
    for (int i = 4; i > 0; --i ) {
        for (int j = 0; j < i; ++j ) {
            ostream << format(triangle.coefficients[counter]) << " ";
            ++counter;
        }
        ostream << "\n";
    }
    ostream.flush();

    return ostream;
}

/**
 * @brief Insertion operator for streaming output from pascal's triangle of degree 5.
 *
 * Insertion operator for streaming output from pascal's triangle.
 *
 * @param ostream : the output stream being used.
 * @param triangle : the pascal triangle object.
 * @return OutputStream : the output stream.
 */
template <typename OutputStream>
OutputStream& operator << ( OutputStream &ostream, const PascalsTriangle<5> &triangle)
{
    Format<int> format(3,CentreAlign);
    int counter = 0;
    for (int i = 6; i > 0; --i ) {
        for (int j = 0; j < i; ++j ) {
            ostream << format(triangle.coefficients[counter]) << " ";
            ++counter;
        }
        ostream << "\n";
    }
    ostream.flush();

    return ostream;
}

} // namespace ecl



#endif /* ECL_GEOMETRY_PASCALS_HPP_ */
