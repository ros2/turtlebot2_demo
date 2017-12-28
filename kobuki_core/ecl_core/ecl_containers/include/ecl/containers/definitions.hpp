/**
 * @file /include/ecl/containers/definitions.hpp
 *
 * @brief Some common container definitions.
 *
 * @date May, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_DEFINITIONS_HPP_
#define ECL_CONTAINERS_DEFINITIONS_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Enums
*****************************************************************************/
/**
 * @brief A simple flag for denoting the storage type of the container.
 *
 * This simply denotes whether the container is dynamic or fixed memory.
 * Currently it is only used under the hood on the Array container. Users
 * should not need to use this directly.
 */
enum StorageType {
    DynamicStorage = 0,/**< Dynamic **/
    FixedStorage = 1   /**< Fixed **/
};

}; // namespace ecl

#endif /* ECL_CONTAINERS_DEFINITIONS_HPP_ */
