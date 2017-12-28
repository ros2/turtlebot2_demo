/**
 * @file /ecl_mpl/include/ecl/mpl/failed_object.hpp
 *
 * @brief An object to use when you want to throw a compile time failure.
 *
 * @date August 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MPL_FAILED_OBJECT_HPP_
#define ECL_MPL_FAILED_OBJECT_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @brief An object designed to fail with compile time error upon instantiation.
 */
class FailedObject {
private:
	FailedObject() {}
};

} // namespace ecl

#endif /* ECL_MPL_FAILED_OBJECT_HPP_ */
