/**
 * @file /include/ecl/concepts/macros.hpp
 *
 * @brief Mechanisms enabling compile time checking of metaprogramming concepts.
 *
 * @date May, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONCEPTS_MACROS_HPP_
#define ECL_CONCEPTS_MACROS_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace concepts {

/*****************************************************************************
** Classes
*****************************************************************************/
/**
 * @cond DO_NOT_DOXYGEN
 */

  /**
   * @brief Initiates the concept checking function test.
   *
   * The function test resides in the void function pointer argument. When
   * this class gets instantiated, the code in the void function pointer will
   * compile.
   */
  template <void(*)()> struct InstantiateConceptCheck {};

  /**
   * This function runs the concept check, and if it fails, the compile time
   * error will be reported as having come from here by including the name
   * of this function (hence the use of caps and the appropriate naming of the
   * function) in the error output message.
   */
  template <class Model>
  void CONCEPT_CHECK_FAILED()
  {
      ((Model*)0)->~Model();
  }

  /**
   * @brief Instantiates the compile time concept error checking process.
   *
   * Instantiates the compile time error checking process. If it ultimately
   * fails, then there will be an error message emitted by this class (hence
   * the use of caps and the appropriate naming of the class).
   */
  template <class Model>
  struct CONCEPT_CHECK
  {
      InstantiateConceptCheck< CONCEPT_CHECK_FAILED<Model> > x; /**< @brief Instantiates the compile time concept checking code. **/
      enum { instantiate = 1 /**< @brief Simple flag to report success of the concept check. **/ };
  };
  /**
   * @endcond
   */
}; // namespace concepts
}; // namespace ecl

#define ecl_concept_check_name_expand(Name) ConceptCheck ## Name
#define ecl_concept_check_name(Name) ecl_concept_check_name_expand(Name)

/**
 * @addtogroup Macros
 * @{
**/
/**
 * @def ecl_compile_time_concept_check
 *
 * @brief Compile time concept checking assertion.
 *
 * This macro checks a class to verify if it conforms to a compile time metaprogramming
 * concept.
 *
 * <b>Usage:</b>
 *
 * The input argument to this macro function is the concept class type with
 * the testing class as its template parameter.
 * @code
 * ecl_compile_time_concept_check(MyConcept<A>);
 * @endcode
 */
#define ecl_compile_time_concept_check( Model )  \
	enum { ecl_concept_check_name(__LINE__) = ecl::concepts::CONCEPT_CHECK< Model >::instantiate }

/**
 * @def ecl_compile_time_concept_test
 *
 * @brief Convenient notational macro for setting up a concept testing function.
 *
 * The actual mechanics of concept testing has to be implemented in the concept class' destructor.
 * This macro is purely just a notational convenience that makes it clear when programming
 * the precise meaning of what is intended for your concept class.
 *
 * <b>Usage:</b>
 *
 * @code
 * class MyConcept {
 *     public:
 *         ecl_compile_time_concept_test(MyConcept)
 *         {
 *             // compile time concept checks here.
 *         }
 * };
 * @endcode
 */
#define ecl_compile_time_concept_test( Model ) \
	ecl_compile_time_concept_check( Model ); \
    ~Model()

/**
 * @}
 **/
#endif /* ECL_CONCEPTS_MACROS_HPP_ */
