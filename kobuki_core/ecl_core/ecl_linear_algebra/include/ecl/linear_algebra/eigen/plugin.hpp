/**
 * @file /include/ecl/linear_algebra/eigen_plugin.hpp
 *
 * @brief Customises the eigen matrixbase class via the plugin handle.
 *
 * Populates the eigen matrix base class with a few extra functions via
 * the EIGEN_MATRIXBASE_PLUGIN macro.
 *
 * @date October 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_LINEAR_ALGEBRA_EIGEN_PLUGIN_HPP_
#define ECL_LINEAR_ALGEBRA_EIGEN_PLUGIN_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "formatters.hpp"

/*****************************************************************************
** Code
*****************************************************************************/

typedef MatrixFormatter<Derived,Scalar> Formatter;

inline void conservativeResize(int rows, int cols)
{
   PlainObject tmp(rows, cols); // alternatively: PlainObject::Zero(rows,cols)
   const int common_rows = std::min(rows, this->rows());
   const int common_cols = std::min(cols, this->cols());
   tmp.block(0,0,common_rows,common_cols) = this->block(0,0,common_rows,common_cols);
   this->derived().swap(tmp);
}

inline void conservativeResize(int size)
{
   EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)

   if (RowsAtCompileTime == 1)
   {
	  PlainObject tmp(1,size); // alternatively: PlainObject::Zero(1,size)
	  const int common_size = std::min(cols(),size);
	  tmp.block(0,0,1,common_size) = this->block(0,0,1,common_size);
	  this->derived().swap(tmp);
   }
   else
   {
	  PlainObject tmp(size,1); // alternatively: PlainObject::Zero(size,1)
	  const int common_size = std::min(rows(),size);
	  tmp.block(0,0,common_size,1) = this->block(0,0,common_size,1);
	  this->derived().swap(tmp);
   }
}

#endif /* ECL_LINEAR_ALGEBRA_EIGEN_PLUGIN_HPP_ */
