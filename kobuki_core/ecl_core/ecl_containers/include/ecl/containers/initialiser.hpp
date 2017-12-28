/**
 * @file /include/ecl/containers/initialiser.hpp
 *
 * @brief Comma initialiser for container types.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_INITIALISER_HPP_
#define ECL_CONTAINERS_INITIALISER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstddef>  // size_t
#include "definitions.hpp"
#include <ecl/config/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace containers {

/*****************************************************************************
** Interface [BoundedListInitialiser]
*****************************************************************************/
/**
 * @brief  Convenience initialiser with bounds checking for fixed size containers.
 *
 * Initialises containers in a manner similar to that used in Blitz++ and Eigen2
 * with a comma separated list. It also does range checking in debug mode.
 * The containers are assumed to have fixed storage length specified by the
 * third template parameter.
 *
 * <b>Error Handling</b>
 *
 * Out of range operations will always throw an exception in debug mode.
 *
 * @sa Array, DynamicArray.
 **/
template<class Type, class Iterator, std::size_t N = ecl::DynamicStorage>
class ECL_LOCAL BoundedListInitialiser {

    public:
        /**
         * Gets a handle to an iterator of the underlying container and adds the first value.
         *
         * @param value : first element to be added to the initialiser.
         * @param iter  : a pointer to the current element to be added to the container.
         */
        BoundedListInitialiser(const Type &value, Iterator iter);

        virtual ~BoundedListInitialiser() {}

        /**
         * Convenience operator for initialising container types with a comma
         * separated list. It uses a current BoundedListInitialiser instance to add <i>the next</i> value
         * to the underlying class and returns the same instance with incremented iterator.
         *
         * @param value : the value to be inserted at the location of the stored iterator.
         * @return BoundedListInitialiser : the initialiser class.
         *
         * @exception StandardException : throws if the range has been exceeded [debug mode only].
         */
        BoundedListInitialiser<Type, Iterator, N>& operator,(const Type &value) ecl_assert_throw_decl(StandardException);

    protected:
        Iterator iterator;
        std::size_t current_size;
};

/*****************************************************************************
** Implementation [BoundedListInitialiser]
*****************************************************************************/

template <typename Type, typename Iterator, std::size_t N>
BoundedListInitialiser<Type,Iterator,N>::BoundedListInitialiser(const Type &value, Iterator iter) :
    iterator(iter),
    current_size(1)
{
    *iterator = value;
    ++iterator;
}
template <typename Type,typename Iterator,std::size_t N>
BoundedListInitialiser<Type,Iterator,N>& BoundedListInitialiser<Type,Iterator,N>::operator,(const Type &value) ecl_assert_throw_decl(StandardException)
{
    ecl_assert_throw( current_size < N, StandardException(LOC,OutOfRangeError) );
    *iterator = value;
    iterator++;
    current_size++;
    return *this;
}

/*****************************************************************************
** Interface [BoundedListInitialiser][Dynamic]
*****************************************************************************/
/**
 * @brief  Convenience initialiser with bounds checking for dynamic containers.
 *
 * Initialises containers in a manner similar to that used in Blitz++ and Eigen2
 * with a comma separated list. It also does range checking in debug mode.
 * The containers are assumed to have dynamic storage length retrievable from
 * the container class itself.
 *
 * This is a specialisation of the fixed container bounded list initialiser.
 *
 * @sa BoundedClassInitialiser
 **/
template<class Type, class Iterator>
class ECL_LOCAL BoundedListInitialiser<Type,Iterator,DynamicStorage> {

    public:
        /**
         * Gets a handle to an iterator of the underlying container and adds the first value.
         *
         * @param value : first element to be added to the initialiser.
         * @param iter  : a pointer to the current element to be added to the container.
         * @param bound : size of the container it is initialising.
         *
         * @exception StandardException : throws if this container has no storage capacity [debug mode only].
         */
        BoundedListInitialiser(const Type &value, Iterator iter, std::size_t bound) ecl_assert_throw_decl(StandardException);

        virtual ~BoundedListInitialiser() {}
        /**
         * Convenience operator for initialising container types with a comma
         * separated list. It uses a current BoundedListInitialiser instance to add <i>the next</i> value
         * to the underlying class and returns the same instance with incremented iterator.
         *
         * @param value : the value to be inserted at the location of the stored iterator.
         * @return BoundedListInitialiser : the initialiser class.
         *
         * @exception StandardException : throws if the range has been exceeded [debug mode only].
         */
        BoundedListInitialiser<Type, Iterator, DynamicStorage>& operator,(const Type &value) ecl_assert_throw_decl(StandardException);

    protected:
        Iterator iterator;
        std::size_t current_size;
        const std::size_t upper_bound;
};

/*****************************************************************************
** Implementation [BoundedListInitialiser]
*****************************************************************************/

template <typename Type, typename Iterator>
BoundedListInitialiser<Type,Iterator,DynamicStorage>::BoundedListInitialiser(const Type &value, Iterator iter, std::size_t bound) ecl_assert_throw_decl(StandardException) :
    iterator(iter),
    current_size(1),
    upper_bound(bound)
{
    ecl_assert_throw( current_size <= upper_bound, StandardException(LOC,OutOfRangeError) );
    *iterator = value;
    ++iterator;
}
template <typename Type,typename Iterator>
BoundedListInitialiser<Type,Iterator,DynamicStorage>& BoundedListInitialiser<Type,Iterator,DynamicStorage>::operator,(const Type &value) ecl_assert_throw_decl(StandardException)
{
    ecl_assert_throw( current_size < upper_bound, StandardException(LOC,OutOfRangeError) );
    *iterator = value;
    iterator++;
    current_size++;
    return *this;
}

}; // namespace containers
}; // namespace ecl

#endif /* ECL_CONTAINERS_INITIALISER_HPP_ */
