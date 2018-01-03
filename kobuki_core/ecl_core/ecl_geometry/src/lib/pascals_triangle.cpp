/**
 * @file /src/lib/pascals_triangle.cpp
 *
 * @brief Implementation of the pascal triangle specialisations.
 *
 * @date May 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/geometry/pascals_triangle.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [PascalsTriangle<3>]
*****************************************************************************/
PascalsTriangle<3>::PascalsTriangle() {
    coefficients << 1,1,1,1,
                   1,2,3,
                   1,3,
                   1;
}

PascalsTriangle<3>::const_iterator PascalsTriangle<3>::begin(unsigned int row_index) const {
    int coeff_index = 0;
    for (unsigned int i = 0; i < row_index; ++i ) {
        coeff_index += 3+1-i;
    }
    return const_iterator( &coefficients[coeff_index] );
}
PascalsTriangle<3>::const_iterator PascalsTriangle<3>::end(unsigned int row_index) const {
    int coeff_index = 0;
    for (unsigned int i = 0; i <= row_index; ++i ) {
        coeff_index += 3+1-i;
    }
    coeff_index -= 1; // dont want to call beyond the array limit.
    return const_iterator( (&coefficients[coeff_index])+1 );
}


/*****************************************************************************
** Implementation [PascalsTriangle<5>]
*****************************************************************************/
PascalsTriangle<5>::PascalsTriangle() {
    coefficients << 1,1,1,1,1,1,
                    1,2,3,4,5,
                    1,3,6,10,
                    1,4,10,
                    1,5,
                    1;
}

PascalsTriangle<5>::const_iterator PascalsTriangle<5>::begin(unsigned int row_index) const {
    int coeff_index = 0;
    for (unsigned int i = 0; i < row_index; ++i ) {
        coeff_index += 5+1-i;
    }
    return const_iterator( &coefficients[coeff_index] );
}
PascalsTriangle<5>::const_iterator PascalsTriangle<5>::end(unsigned int row_index) const {
    int coeff_index = 0;
    for (unsigned int i = 0; i <= row_index; ++i ) {
        coeff_index += 5+1-i;
    }
    coeff_index -= 1; // dont want to call beyond the array limit.
    return const_iterator( (&coefficients[coeff_index])+1 );
}


} // namespace ecl
