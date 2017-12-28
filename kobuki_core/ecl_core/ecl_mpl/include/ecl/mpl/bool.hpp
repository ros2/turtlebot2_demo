/**
 * @file /include/ecl/mpl/bool.hpp
 *
 * @brief Defines the metaprogamming equivalent of the boolean type.
 *
 * @date July 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MPL_BOOL_HPP_
#define ECL_MPL_BOOL_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

/**
 * @brief Integral constant wrapper for boolean values.
 *
 * Provides an integral constant wrapper for boolean values (refer to C++
 * template metaprogramming by D.Abrahams p. 61 for details).
 */
template < bool x >
class Bool {
public:
	static bool const value = x;     /**< @brief The value for this integral constant wrapper. **/
	typedef Bool<x> type;            /**< @brief This wrapper's nullary metafunction (simply returns itself). **/
	typedef bool value_type;         /**< @brief The type this wrapper's value. **/
	operator bool() const { return x; } /**< @brief Convenience conversion. **/
	virtual ~Bool(){};
};

typedef Bool<false> False; /**< @brief Convenient typedef for the 'false' integral constant wrapper. **/
typedef Bool<true> True; /**< @brief Convenient typedef for the 'true' integral constant wrapper. **/

} // namespace ecl

#endif /* ECL_MPL_BOOL_HPP_ */
