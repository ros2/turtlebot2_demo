/**
 * @file /include/ecl/concepts/containers.hpp
 *
 * @brief Defines and validates the functionality for the <i>container</i> concept.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONCEPTS_CONCEPTS_CONTAINERS_HPP_
#define ECL_CONCEPTS_CONCEPTS_CONTAINERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#include <ecl/type_traits/fundamental_types.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Concept [Container]
*****************************************************************************/

/**
 * @brief Defines validating functionality for the @ref containersConcept "container concept".
 **/
template <typename Implementation>
class ContainerConcept {
public:

	/**
	 * @brief Implements a concept test for containers.
	 *
	 * The following conditions are required by containers:
	 *
	 * - begin() : a method for retrieving a pointer/iterator to the start of the container.
	 * - end() : a method for retrieving a pointer/iterator to the end of the container.
	 * - size() : a method for retrieving the number of elements stored in the container.
	 * - value_type : a typedef for the element type stored in the container.
	 */
	ecl_compile_time_concept_test(ContainerConcept)
	{
		iter = container.begin();
		iter = container.end();
		unsigned int n;
		n = container.size();
                (void) n; // remove set but not used warnings.
	}

private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation container;
	typename Implementation::iterator iter;
};
/**
 * @brief Defines validating functionality for the dynamic @ref containersConcept "container concept".
 **/
template <typename Implementation>
class DynamicContainerConcept {
public:

	/**
	 * @brief Implements a concept test for containers.
	 *
	 * The following conditions are required by containers:
	 *
	 * - begin() : a method for retrieving a pointer/iterator to the start of the container.
	 * - end() : a method for retrieving a pointer/iterator to the end of the container.
	 * - size() : a method for retrieving the number of elements stored in the container.
	 * - resize(int) : a method for resizing the container.
	 * - value_type : a typedef for the element type stored in the container.
	 */
	ecl_compile_time_concept_test(DynamicContainerConcept)
	{
		iter = container.begin();
		iter = container.end();
		unsigned int n;
		n = container.size();
		container.resize(1);
                (void) n; // remove set but not used warnings.
	}

private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation container;
	typename Implementation::iterator iter;
};

/**
 * @cond DO_NOT_DOXYGEN
 */
namespace concepts {

/**
 * @brief Back end to the signed byte test check.
 *
 * Used to invalidate with a private constructor when the condition is right.
 * This is the one that will show up in the printout for a compile time
 * error.
 *
 * Do not use this one directly, use SignedByteTestCheck to make use of it.
 */
template <typename T, bool Result=false>
class SignedByteTest {
private:
	SignedByteTest() {}
};

/**
 * @brief Back end to the signed byte test check.
 *
 * Used to toggle a valid public constructor when the condition is right.
 * This is the one that will show up in the printout for a compile time
 * error.
 *
 * Do not use this one directly, use SignedByteTestCheck to make use of it.
 */
template <typename T>
class SignedByteTest<T,true> {
public:
	SignedByteTest() {}
};

/**
 * @brief Back end to the unsigned byte test check.
 *
 * Used to invalidate with a private constructor when the condition is right.
 * This is the one that will show up in the printout for a compile time
 * error.
 *
 * Do not use this one directly, use UnsignedByteTestCheck to make use of it.
 */
template <typename T, bool Result=false>
class UnsignedByteTest {
private:
	UnsignedByteTest() {}
};

/**
 * @brief Back end to the unsigned byte test check.
 *
 * Used to toggle a valid public constructor when the condition is right.
 * This is the one that will show up in the printout for a compile time
 * error.
 *
 * Do not use this one directly, use UnsignedByteTestCheck to make use of it.
 */
template <typename T>
class UnsignedByteTest<T,true> {
public:
	UnsignedByteTest() {}
};

/**
 * @brief Front end to the signed byte test check.
 *
 * Forces it to throw a compile time error if the check fails because
 * the parent implementation will have a private constructor.
 */
template <typename T>
class SignedByteTestCheck : public SignedByteTest<T, ecl::is_signed_byte<T>::value > {};

/**
 * @brief Front end to the unsigned byte test check.
 *
 * Forces it to throw a compile time error if the check fails because
 * the parent implementation will have a private constructor.
 */
template <typename T>
class UnsignedByteTestCheck : public UnsignedByteTest<T, ecl::is_unsigned_byte<T>::value > {};

/**
 * @brief Used to invalidate non char types storage.
 *
 * This is the default template class for the unsigned character test - if the
 * specialisation from the byte container element type being tested is
 * not a char type, it will use this and result in a compile time error
 * since the constructor is private!
 */
template <typename T>
class ByteTest {
private:
	ByteTest() {}
};

/**
 * @brief Used to validate char types storage.
 *
 * Containers with a unsigned char storage element will use this specialisation
 * when doing the byte container concept check. This results in no
 * compile time error as the constructor is public (as opposed to
 * the generalised instance which has a private constructor).
 */
template <>
class ByteTest<unsigned char> {
public:
	ByteTest() {}
};
/**
 * @brief Used to validate char types storage as byte storage.
 */
template <>
class ByteTest<char> {
public:
	ByteTest() {}
};

/**
 * @brief Used to validate signed char types storage as byte storage.
 */
template <>
class ByteTest<signed char> {
public:
	ByteTest() {}
};

} // namespace concepts
/**
 * @endcond
 */

/**
 * @brief Defines validating functionality for the <i>byte container</i> concept.
 **/
template <typename Implementation>
class UnsignedByteContainerConcept {
public:

	/**
	 * @brief Implements a concept test for byte containers.
	 *
	 * The following conditions are required by byte containers:
	 * - begin() : a method for retrieving a pointer/iterator to the start of the container.
	 * - end() : a method for retrieving a pointer/iterator to the end of the container.
	 * - size() : a method for retrieving the number of elements stored in the container.
	 * - value_type : a typedef for the element type stored in the container, must be unsigned char.
	 */
	ecl_compile_time_concept_test(UnsignedByteContainerConcept)
	{
		typedef typename Implementation::value_type element_type;
		iter = container.begin();
		iter = container.end();
		unsigned int n;
		n = container.size();
		// If not a char, this results in a private call -> compile time error!
		concepts::UnsignedByteTestCheck<element_type> char_test;
                (void) n; // remove set but unused warnings.
	}

private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation container;
	typename Implementation::iterator iter;
};

/**
 * @brief Defines validating functionality for the <i>signed byte container</i> concept.
 **/
template <typename Implementation>
class SignedByteContainerConcept {
public:

	/**
	 * @brief Implements a concept test for byte containers.
	 *
	 * The following conditions are required by byte containers:
	 * - begin() : a method for retrieving a pointer/iterator to the start of the container.
	 * - end() : a method for retrieving a pointer/iterator to the end of the container.
	 * - size() : a method for retrieving the number of elements stored in the container.
	 * - value_type : a typedef for the element type stored in the container, must be signed char.
	 */
	ecl_compile_time_concept_test(SignedByteContainerConcept)
	{
		typedef typename Implementation::value_type element_type;
		iter = container.begin();
		iter = container.end();
		unsigned int n;
		n = container.size();
		// If not a char, this results in a private call -> compile time error!
		concepts::SignedByteTestCheck<element_type> char_test;
	}

private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation container;
	typename Implementation::iterator iter;
};

/**
 * @brief Defines validating functionality for the <i>byte container</i> concept.
 *
 * Elements stored in this container can be either signed, unsigned or char type.
 **/
template <typename Implementation>
class ByteContainerConcept {
public:

	/**
	 * @brief Implements a concept test for byte containers.
	 *
	 * The following conditions are required by byte containers:
	 * - begin() : a method for retrieving a pointer/iterator to the start of the container.
	 * - end() : a method for retrieving a pointer/iterator to the end of the container.
	 * - size() : a method for retrieving the number of elements stored in the container.
	 * - value_type : a typedef for the element type stored in the container, must be char, signed char or unsigned char.
	 */
	ecl_compile_time_concept_test(ByteContainerConcept)
	{
		typedef typename Implementation::value_type element_type;
		iter = container.begin();
		iter = container.end();
		unsigned int n;
		n = container.size();
		// If not a char, this results in a private call -> compile time error!
		concepts::ByteTest<element_type> char_test;
                (void) n; // remove set but unused warnings.
	}

private:
	// Putting instantiations here actually saves instantiation (which can cause a
	// problem if there is no default constructor).
	Implementation container;
	typename Implementation::iterator iter;
};

}; // namespace ecl

#endif /* ECL_CONCEPTS_CONCEPTS_CONTAINERS_HPP_ */
