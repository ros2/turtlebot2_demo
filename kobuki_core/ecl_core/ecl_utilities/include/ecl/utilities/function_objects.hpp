/**
 * @file /include/ecl/utilities/function_objects.hpp
 *
 * @brief Functional objects and generators.
 *
 * @date June 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_UTILITIES_FUNCTION_OBJECTS_HPP_
#define ECL_UTILITIES_FUNCTION_OBJECTS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/concepts/nullary_function.hpp>
#include "../utilities/references.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

/*****************************************************************************
** Interface [NullaryFunction]
*****************************************************************************/
/**
 * @brief Virtual interface definition for nullary function objects.
 *
 * Virtual interface definition for nullary function objects (i.e. functions
 * that take no arguments).
 *
 * @tparam R : the return type.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename R = void>
class NullaryFunction {
public:
	typedef R result_type;  /**< @brief The result type. **/
	virtual result_type operator()() = 0; /**< @brief Virtual function call required by nullary function objects. **/
	virtual ~NullaryFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
};

/*****************************************************************************
** Interface [UnaryFunction]
*****************************************************************************/

/**
 * @brief Virtual interface definition for unary function objects.
 *
 * Virtual interface definition for unary function objects (i.e. functions
 * that take a single arguments).
 *
 * @tparam A : the argument type.
 * @tparam R : the return type.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename A, typename R = void>
class UnaryFunction {
public:
	typedef R result_type; /**< @brief The result type. **/
	typedef A argument_type; /**< @brief The first argument type. **/
	/**
	 * @brief Virtual function call required by unary function objects.
	 * @param arg : the argument.
	 **/
	virtual result_type operator()(argument_type arg) = 0; /**<@brief Virtual function call required by unary function objects. **/
	virtual ~UnaryFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
};

/*****************************************************************************
** Interface [BinaryFunction]
*****************************************************************************/
/**
 * @brief Virtual interface definition for binary function objects.
 *
 * Virtual interface definition for binary function objects (i.e. functions
 * that take a pair of arguments).
 *
 * @tparam R : the return type.
 * @tparam A1 : the first argument type.
 * @tparam A2 : the second argument type.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename A1, typename A2, typename R = void>
class BinaryFunction {
public:
	typedef R result_type; /**< @brief The result type. **/
	typedef A1 first_argument_type; /**< @brief The first argument type. **/
	typedef A2 second_argument_type; /**< @brief The second argument type. **/
	/**
	 * @brief Virtual function call required by binary function objects.
	 *
	 * Virtual function call required by binary function objects.
	 * @param arg1 : the first argument.
	 * @param arg2 : the second argument.
	 **/
	virtual result_type operator()(first_argument_type arg1, second_argument_type arg2) = 0; /**<@brief Virtual function call required by binary function objects. **/
	virtual ~BinaryFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
};

/*****************************************************************************
** Interface [FreeFunctions]
*****************************************************************************/
/**
 * @brief Nullary function object for void global/static functions.
 *
 * Creates a function object from a void global/static function.
 *
 * @tparam R : the return type.
 *
 * @sa ecl::utilities::NullaryFreeFunction<void>, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename R = void>
class NullaryFreeFunction : public NullaryFunction<R>
{
public:
	/**
	 * @brief Nullary function object constructor for global/static functions with no args.
	 *
	 * Accepts a global/static function with no args and builds the function object around it.
	 *
	 * @param function : the global/static function.
	 */
	NullaryFreeFunction( R (*function)() ) : free_function(function) {}
	virtual ~NullaryFreeFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief A nullary function object call.
	 *
	 * Redirects the nullary function object call to the composited global/static function.
	 *
	 * @return R : the function's return value.
	 */
	R operator()() { return free_function(); }

private:
	R (*free_function)();
};

/**
 * @brief Specialisation for free nullary functions that return void.
 *
 * Specialisation for free nullary functions that return void.
 *
 * @sa NullaryFreeFunction, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <>
class NullaryFreeFunction<void> : public NullaryFunction<void>
{
public:
	/**
	 * @brief Nullary function object constructor for void global/static functions with no args.
	 *
	 * Accepts a void global/static function with no args and builds the function object around it.
	 *
	 * @param function : the global/static function.
	 */
	NullaryFreeFunction( void (*function)() ) : free_function(function) {}

	virtual ~NullaryFreeFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief A nullary function object call.
	 *
	 * Redirects the nullary function object call to the composited global/static function.
	 *
	 * @return R : the function's return value.
	 */
	void operator()() { free_function(); }

private:
	void (*free_function)();
};

/**
 * @brief Unary function object for global/static functions.
 *
 * Creates a function object from a global/static function with a single argument.
 *
 * @sa generateFunctionObject
 *
 * @tparam A : the argument type.
 * @tparam R : the return type.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename A, typename R = void>
class UnaryFreeFunction : public UnaryFunction<A,R>
{
public:
	/**
	 * @brief Unary function object constructor for global/static functions.
	 *
	 * Accepts a global/static function with a single argument and builds
	 * the function object around it.
	 *
	 * @param function : a global/static function with a single argument.
	 */
	UnaryFreeFunction( R (*function)(A) ) : free_function(function) {}

	virtual ~UnaryFreeFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief A unary function object call.
	 *
	 * Redirects the unary function object call to the composited global/static function.
	 *
	 * @return R : the function's return value.
	 */
	R operator()(A a) { return free_function(a); }

private:
	R (*free_function)(A);
};

/**
 * @brief Specialisations for free unary functions with no return type.
 *
 * Specialisations for free unary functions with no return type.
 *
 * @tparam A : the argument type.
 *
 * @sa ecl::utilities::UnaryFreeFunction, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename A>
class UnaryFreeFunction<A,void> : public UnaryFunction<A,void>
{
public:
	/**
	 * @brief Unary function object constructor for global/static unary functions with no return type.
	 *
	 * Accepts a void global/static function with a single argument and builds
	 * the function object around it.
	 *
	 * @param function : a global/static function with a single argument.
	 */
	UnaryFreeFunction( void (*function)(A) ) : free_function(function) {}

	virtual ~UnaryFreeFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
	/**
	 * @brief A unary function object call.
	 *
	 * Redirects the unary function object call to the composited void global/static function.
	 *
	 * @return R : the function's return value.
	 */
	void operator()(A a) { free_function(a); }

private:
	void (*free_function)(A);
};

/*****************************************************************************
** Interface [BoundFreeFunctions]
*****************************************************************************/
/**
 * @brief Nullary function object for bound unary global/static functions.
 *
 * Binds the argument to a unary global/static function and uses this to
 * construct a nullary function object.
 *
 * <b>Usage: </b>
 * @code
 * void f(int i) {}
 *
 * int main() {
 *     BoundUnaryFreeFunction<int,void> function_object(f,1);
 *     function_object();
 * }
 * @endcode
 *
 * Note, often the use of generateFunctionObject is simpler.
 *
 * @tparam A : the type of the argument to be bound.
 * @tparam R : the return type.
 *
 * @sa ecl::utilities::BoundUnaryFreeFunction<A,void>, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename A, typename R = void>
class BoundUnaryFreeFunction : public NullaryFunction<R>
{
public:
	/**
	 * @brief Binds a unary function and creates a nullary function object.
	 *
	 * Accepts both the function and a value for its single argument, binds them
	 * and creates a nullary function object.
	 * @param function : the unary global/static function.
	 * @param a        : the argument to bind.
	 */
	BoundUnaryFreeFunction( R (*function)(A), A a ) : free_function(function), argument(a) {}
	virtual ~BoundUnaryFreeFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
	/**
	 * @brief A nullary function object call.
	 *
	 * Redirects the function object call to the bound global/static function.
	 *
	 * @return R : the function's return value.
	 */
	R operator()() { return free_function(argument); }

private:
	R (*free_function)(A);
	A argument;
};

/**
 * @brief Specialisation for bound void unary functions.
 *
 * Specialises BoundUnaryFreeFunction for functions with no return type.
 *
 * @tparam A : the type of the argument to be bound.
 *
 * @sa ecl::utilities::BoundUnaryFreeFunction, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename A>
class BoundUnaryFreeFunction<A,void> : public NullaryFunction<void>
{
public:
	/**
	 * @brief Binds a unary function and creates a nullary function object.
	 *
	 * Accepts both the function and a value for its single argument, binds them
	 * and creates a nullary function object.
	 * @param function : the unary global/static function.
	 * @param a        : the argument to bind.
	 */
	BoundUnaryFreeFunction( void (*function)(A), A a ) : free_function(function), argument(a) {}
	virtual ~BoundUnaryFreeFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
	/**
	 * @brief A nullary function object call.
	 *
	 * Redirects the nullary function object call to the bound global/static function.
	 *
	 * @return void : the function's return value.
	 */
	void operator()() { free_function(argument); }

private:
	void (*free_function)(A);
	A argument;
};

/*****************************************************************************
 * Interface [MemberFunctions]
 *****************************************************************************
 * Could make copies+refs here, just like mem_fun and mem_fun_ref but I can't
 * see the need for copy style setups.
 ****************************************************************************/

/**
 * @brief Unary function object for member functions without arguments.
 *
 * Creates a function object from a member function without arguments
 * (note, the single argument to this unary function object is the class
 * instance itself).
 *
 * <b>Usage: </b>
 * @code
 * class A {
 * public:
 *     void f() { //...
 *     }
 * };
 *
 * int main() {
 *     A a;
 *     NullaryMemberFunction<A,void> function_object(&A::f);
 *     function_object(a);
 * }
 * @endcode
 *
 * @tparam C : the member function's class type type.
 * @tparam R : the return type.
 *
 * @sa ecl::utilities::NullaryMemberFunction<C,void>, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename R = void>
class NullaryMemberFunction  : public UnaryFunction<C&,R> {
public:
	/**
	 * @brief Unary function object constructor for member functions without arguments.
	 *
	 * Accepts a void member function, and builds
	 * the function object around it.
	 *
	 * @param function : a void member function.
	 */
	NullaryMemberFunction( R (C::*function)()) : member_function(function) {}
	virtual ~NullaryMemberFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/


	/**
	 * @brief A unary function object call.
	 *
	 * Uses the specified class instance to redirect the function call to the
	 * void member function.
	 *
	 * @param class_object : the member function's class instance.
	 * @return R : the function's return value.
	 */
	R operator()(C &class_object) {
		return (class_object.*member_function)();
	}
private:
    R (C::*member_function)();
};

/**
 * @brief Specialisation of the unary function object for void member functions without arguments.
 *
 * Specialisation for a function object from a void member function without arguments (note, the single
 * argument to this unary function object is the class instance itself).
 *
 * @tparam C : the member function's class type type.
 *
 * @sa ecl::utilities::NullaryMemberFunction, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C>
class NullaryMemberFunction<C,void>  : public UnaryFunction<C&,void> {
public:
	/**
	 * @brief Unary function object constructor for void member functions.
	 *
	 * Accepts a void member function without arguments, and builds
	 * the function object around it.
	 *
	 * @param function : a void member function.
	 */
	NullaryMemberFunction( void (C::*function)()) : member_function(function) {}
	virtual ~NullaryMemberFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief A unary function object call.
	 *
	 * Uses the specified class instance to redirect the function call to the
	 * void member function.
	 *
	 * @param class_object : the member function's class instance.
	 */
	void operator()(C &class_object) {
		(class_object.*member_function)();
	}
private:
    void (C::*member_function)();
};

/**
 * @brief Binary function object for unary member functions.
 *
 * Creates a function object from a unary member function.
 *
 * <b>Usage: </b>
 * @code
 * class A {
 * public:
 *     void f(int i) { //...
 *     }
 * };
 *
 * int main() {
 *     A a;
 *     UnaryMemberFunction<A,int,void> function_object(&A::f);
 *     function_object(a,1);
 * }
 * @endcode
 *
 * @tparam C : the member function's class type.
 * @tparam A : the member function's argument type.
 * @tparam R : the return type.
 *
 * @sa ecl::utilities::UnaryMemberFunction<C,A,void>, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename A, typename R = void>
class UnaryMemberFunction  : public BinaryFunction<C&,A,R> {
public:
	/**
	 * @brief Binary function object constructor for unary member functions.
	 *
	 * Accepts a unary member function, and builds
	 * the function object around it.
	 *
	 * @param function : a unary member function.
	 */
	UnaryMemberFunction( R (C::*function)(A)) : member_function(function) {}
	virtual ~UnaryMemberFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
	/**
	 * @brief A binary function object call.
	 *
	 * Uses the specified class instance and argument to redirect the function call to the
	 * unary member function.
	 *
	 * @param class_object : the member function's class instance.
	 * @param a : the member function's argument value.
	 * @return R : the function's return value.
	 */
	R operator()(C &class_object, A a) {
		return (class_object.*member_function)(a);
	}
private:
    R (C::*member_function)(A);
};

/**
 * @brief Specialisation of the binary function object for void unary member functions.
 *
 * Specialises the binary member function object for void member functions.
 *
 * @tparam C : the member function's class type.
 * @tparam A : the member function's argument type.
 *
 * @sa ecl::utilities::UnaryMemberFunction @sa generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename A>
class UnaryMemberFunction<C,A,void>  : public BinaryFunction<C&,A,void> {
public:
	/**
	 * @brief Binary function object constructor for unary member functions.
	 *
	 * Accepts a unary member function, and builds
	 * the function object around it.
	 *
	 * @param function : a unary member function.
	 */
	UnaryMemberFunction( void (C::*function)(A)) : member_function(function) {}
	virtual ~UnaryMemberFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
	/**
	 * @brief A binary function object call.
	 *
	 * Uses the specified class instance and argument to redirect the function call to the
	 * unary member function.
	 *
	 * @param class_object : the member function's class instance.
	 * @param a : the member function's argument value.
	 * @return R : the function's return value.
	 */
	void operator()(C &class_object, A a) {
		(class_object.*member_function)(a);
	}
private:
    void (C::*member_function)(A);
};

/*****************************************************************************
** Interface [BoundMemberFunctions]
*****************************************************************************/

/**
 * @brief Nullary function object for bound nullary member functions.
 *
 * Binds the class instance for a nullary member function and uses this to
 * construct a nullary function object.
 *
 * <b>Usage: </b>
 * @code
 * class A {
 * public:
 *     void f() { //...
 *     }
 *
 * int main() {
 *     A a;
 *     BoundNullaryMemberFunction<A,void> function_object(&A::f,a);
 *     function_object();
 * }
 * @endcode
 *
 * @tparam C : the member function's class type.
 * @tparam R : the return type.
 *
 * @sa ecl::utilities::BoundNullaryMemberFunction<C,void>, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename R = void>
class BoundNullaryMemberFunction : public NullaryFunction<R> {
public:
	/**
	 * @brief Binds a unary member function and creates a nullary function object.
	 *
	 * Accepts the function pointer and class instance, binds them
	 * and creates a nullary function object.
	 * @param function : the void member function.
	 * @param class_object : the member function's class instance.
	 */
	BoundNullaryMemberFunction( R (C::*function)(), C &class_object) :
		member_class(class_object),
		member_function(function)
	{}
	virtual ~BoundNullaryMemberFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief A nullary function object call.
	 *
	 * Redirects the nullary function object call to the bound member function.
	 *
	 * @return R : the function's return value.
	 */
	R operator()() { return (member_class.*member_function)(); }

private:
	C &member_class;
    R (C::*member_function)();
};

/**
 * @brief Specialisation of the bound nullary member function for void functions.
 *
 * Binds the class instance for a nullary member function and uses this to
 * construct a nullary function object. This is the specialisation for nullary
 * member functions with void return type.
 *
 * @tparam C : the member function's class type.
 *
 * @sa ecl::utilities::BoundNullaryMemberFunction, generateFunctionObject, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C>
class BoundNullaryMemberFunction<C,void> : public NullaryFunction<void> {
public:
	/**
	 * @brief Binds a unary member function and creates a nullary function object.
	 *
	 * Accepts the function pointer and class instance, binds them
	 * and creates a nullary function object.
	 * @param function : the void member function.
	 * @param class_object : the member function's class instance.
	 */
	BoundNullaryMemberFunction( void (C::*function)(), C &class_object) :
		member_class(class_object),
		member_function(function)
	{}

	virtual ~BoundNullaryMemberFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
	/**
	 * @brief A nullary function object call.
	 *
	 * Redirects the nullary function object call to the bound member function.
	 */
	void operator()() { (member_class.*member_function)(); }

private:
	C &member_class;
    void (C::*member_function)();
};

/**
 * @brief Unary function object for partially bound unary member functions.
 *
 * Binds the class instance but not the argument for a unary member function and uses this to
 * construct a unary function object.
 *
 * <b>Usage: </b>
 * @code
 * class A {
 * public:
 *     void f(int i) { //...
 *     }
 *
 * int main() {
 *     A a;
 *     PartiallyBoundUnaryMemberFunction<A,int,void> function_object(&A::f,a);
 *     function_object(1);
 * }
 * @endcode
 *
 * @sa generateFunctionObject
 *
 * @tparam C : the member function's class type.
 * @tparam A : the member function's argument type.
 * @tparam R : the return type.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename A, typename R = void>
class PartiallyBoundUnaryMemberFunction : public UnaryFunction<A,R> {
public:
	/**
	 * @brief Binds a unary member function and creates a nullary function object.
	 *
	 * Accepts the function, class instance only (not the argument), binds them
	 * and creates a unary function object.
	 * @param function : the unary global/static function.
	 * @param class_object : the member function's class instance.
	 */
	PartiallyBoundUnaryMemberFunction( R (C::*function)(A), C &class_object) :
		member_class(class_object),
		member_function(function)
	{}
	virtual ~PartiallyBoundUnaryMemberFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
	/**
	 * @brief A unary function object call.
	 *
	 * Redirects the unary function object call to the bound member function.
	 *
	 * @param a : the argument passed to the function.
	 * @return R : the function's return value.
	 */
	R operator()(A a) {
		return (member_class.*member_function)(a);
	}
private:
	C &member_class;
    void (C::*member_function)(A);
};
/**
 * @brief Nullary function object for bound unary member functions.
 *
 * Binds the class instance and argument for a unary member function and uses this to
 * construct a nullary function object.
 *
 * <b>Usage: </b>
 * @code
 * class A {
 * public:
 *     void f(int i) { //...
 *     }
 *
 * int main() {
 *     A a;
 *     BoundUnaryMemberFunction<A,int,void> function_object(&A::f,a,1);
 *     function_object();
 * }
 * @endcode
 *
 * @sa generateFunctionObject
 *
 * @tparam C : the member function's class type.
 * @tparam A : the member function's argument type.
 * @tparam R : the return type.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename A, typename R = void>
class BoundUnaryMemberFunction : public NullaryFunction<R> {
public:
	/**
	 * @brief Binds a unary member function and creates a nullary function object.
	 *
	 * Accepts the function, class instance and a value for its single argument, binds them
	 * and creates a nullary function object.
	 * @param function : the unary global/static function.
	 * @param class_object : the member function's class instance.
	 * @param a        : the value of the argument to bind.
	 */
	BoundUnaryMemberFunction( R (C::*function)(A), C &class_object, A a) :
		member_class(class_object),
		member_function(function),
		argument(a)
	{}
	virtual ~BoundUnaryMemberFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
	/**
	 * @brief A nullary function object call.
	 *
	 * Redirects the nullary function object call to the bound member function.
	 *
	 * @return R : the function's return value.
	 */
	R operator()() {
		return (member_class.*member_function)(argument);
	}
private:
	C &member_class;
    void (C::*member_function)(A);
    A argument;
};

/**
 * @brief Binary function object for partially bound binary member functions.
 *
 * Binds the class instance but not the arguments for a binary member function and uses this to
 * construct a binary function object.
 *
 * <b>Usage: </b>
 * @code
 * class A {
 * public:
 *     void f(int i, string n) { //...
 *     }
 *
 * int main() {
 *     A a;
 *     PartiallyBoundBinaryMemberFunction<A,int,string,void> function_object(&A::f,a);
 *     function_object(1,"dude");
 * }
 * @endcode
 *
 * @sa generateFunctionObject
 *
 * @tparam C : the member function's class type.
 * @tparam A : the member function's first argument type.
 * @tparam B : the member function's second argument type.
 * @tparam R : the return type.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename A, typename B, typename R = void>
class PartiallyBoundBinaryMemberFunction : public BinaryFunction<A,B,R> {
public:
        /**
         * @brief Binds a binary member function and creates a binary function object.
         *
         * Accepts the function, class instance only (not the arguments), binds them
         * and creates a binary function object.
         * @param function : the unary global/static function.
         * @param class_object : the member function's class instance.
         */
        PartiallyBoundBinaryMemberFunction( R (C::*function)(A, B), C &class_object) :
                member_class(class_object),
                member_function(function)
        {}
        virtual ~PartiallyBoundBinaryMemberFunction() {}; /**< @brief This ensures any children objects are deleted correctly. **/
        /**
         * @brief A binary function object call.
         *
         * Redirects the binary function object call to the bound member function.
         *
         * @param a : the first argument passed to the function (type A).
         * @param b : the second argument passed to the function (type B).
         * @return R : the function's return value.
         */
        R operator()(A a, B b) {
                return (member_class.*member_function)(a, b);
        }
private:
        C &member_class;
        void (C::*member_function)(A, B);
};
/*****************************************************************************
** Function Object Wrappers
*****************************************************************************/
/**
 * @brief Create a NullaryFunction object composited from an existing function object.
 *
 * Takes a nullary function object (strictly by definition) and creates a
 * NullaryFunction child object. This
 * is useful in utilising the inheritance from NullaryFunction (needed for slots
 * and similar classes).
 *
 * @tparam FunctionObject : type of the function object to be wrapped.
 * @tparam R : the return type.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename FunctionObject, typename Result = void>
class NullaryFunctionCopy : public NullaryFunction<Result>
{
public:
	/**
	 * @brief NullaryFunction child constructor for nullary function objects.
	 *
	 * Creates a child of the NullaryFunction class by copying a nullary function
	 * object (one that is purely by definition).
	 *
	 * @param f_o : the function object to be assigned to the NullaryFunction child.
	 */
	NullaryFunctionCopy(const FunctionObject &f_o ) : function_object(f_o) {
		ecl_compile_time_concept_check(NullaryFunction<FunctionObject>);
	}
	virtual ~NullaryFunctionCopy() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief The nullary function object call.
	 *
	 * Redirects the call to the composited nullary function object call.
	 *
	 * @return R : the function's return value.
	 */
	Result operator()() { return function_object(); }

private:
	FunctionObject function_object;
};

/**
 * @brief Specialisation of NullaryFunctionCopy for void return types.
 *
 * Specialises the NullaryFunctionCopy class for void return types.
 *
 * @tparam FunctionObject : type of the function object to be wrapped.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename FunctionObject>
class NullaryFunctionCopy<FunctionObject,void> : public NullaryFunction<void>
{
public:
	/**
	 * @brief NullaryFunction child constructor for nullary function objects.
	 *
	 * Creates a child of the NullaryFunction class by copying a nullary function
	 * object (one that is purely by definition).
	 *
	 * @param f_o : the function object to be assigned to the NullaryFunction child.
	 */
	explicit NullaryFunctionCopy( const FunctionObject &f_o ) : function_object(f_o) {
		ecl_compile_time_concept_check(NullaryFunction<FunctionObject>);
	}
	virtual ~NullaryFunctionCopy() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief The nullary function object call.
	 *
	 * Redirects the call to the composited nullary function object call.
	 */
	void operator()() { function_object(); }

private:
	FunctionObject function_object;
};

/**
 * @brief Creates a nullary function from a reference wrapper.
 *
 * Takes a reference wrapper containing a nullary function (strictly by definition) object
 * reference and creates a NullaryFunction descendant. This
 * is useful in utilising the inheritance from NullaryFunction (needed for slots
 * and similar classes).
 *
 * @tparam FunctionObject : type of the function object to be referenced.
 * @tparam Result : the return type of the nullary function.
 *
 * @sa NullaryFunctionReference<FunctionObject,void>, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename FunctionObject, typename Result = void>
class NullaryFunctionReference : public NullaryFunction<Result>
{
public:
	/**
	 * @brief Creates a NullaryFunction descendant from a reference wrapper.
	 *
	 * Creates a NullaryFunction descendant by reference (not copying).
	 *
	 * @param wrapper : the reference wrapper holding the nullary function object to be referenced.
	 */
	explicit NullaryFunctionReference( const ReferenceWrapper<FunctionObject> &wrapper ) : function_object(wrapper.reference()) {
		ecl_compile_time_concept_check(NullaryFunction<FunctionObject>);
	}
	virtual ~NullaryFunctionReference() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief The nullary function object call.
	 *
	 * Redirects the call to the referenced nullary function object call.
	 *
	 * @return R : the function's return value.
	 */
	Result operator()() { return function_object(); }

private:
	FunctionObject &function_object;
};

/**
 * @brief Creates a void nullary function from a reference wrapper.
 *
 * Takes a reference wrapper containing a nullary function (strictly by definition) object
 * reference and creates a NullaryFunction descendant. This
 * is a specialisation which caters to nullary function objects with void return type.
 *
 * @tparam FunctionObject : type of the function object to be referenced.
 *
 * @sa NullaryFunctionReference, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename FunctionObject>
class NullaryFunctionReference< FunctionObject, void > : public NullaryFunction<void>
{
public:
	/**
	 * @brief Creates a NullaryFunction descendant from a reference wrapper.
	 *
	 * Creates a NullaryFunction descendant by reference (not copying).
	 *
	 * @param wrapper : the reference wrapper holding the nullary function object to be referenced.
	 */
	explicit NullaryFunctionReference( const ReferenceWrapper<FunctionObject> &wrapper ) : function_object(wrapper.reference()) {
		ecl_compile_time_concept_check(NullaryFunction<FunctionObject>);
	}
	virtual ~NullaryFunctionReference() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief The nullary function object call.
	 *
	 * Redirects the call to the referenced nullary function object call.
	 */
	void operator()() { function_object(); }

private:
	FunctionObject &function_object;
};

/**
 * @brief Create a UnaryFunction object composited from an existing function object.
 *
 * Takes a unary function object (strictly by definition) and creates a
 * UnaryFunction child object. This
 * is useful in utilising the inheritance from UnaryFunction (needed for slots
 * and similar classes).
 *
 * @tparam FunctionObject : type of the function object to be wrapped.
 * @tparam T : the unary data type.
 * @tparam R : the return type.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename FunctionObject, typename T, typename Result = void>
class UnaryFunctionCopy : public UnaryFunction<T, Result>
{
public:
	/**
	 * @brief UnaryFunction child constructor for unary function objects.
	 *
	 * Creates a child of the UnaryFunction class by copying a unary function
	 * object (one that is purely by definition).
	 *
	 * @param f_o : the function object to be assigned to the UnaryFunction child.
	 */
	UnaryFunctionCopy(const FunctionObject &f_o ) : function_object(f_o) {
//		ecl_compile_time_concept_check(UnaryFunction<FunctionObject>);
	}
	virtual ~UnaryFunctionCopy() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief The unary function object call.
	 *
	 * Redirects the call to the composited nullary function object call.
	 *
	 * @return R : the function's return value.
	 */
	Result operator()(T t) { return function_object(t); }

private:
	FunctionObject function_object;
};

/**
 * @brief Specialisation of UnaryFunctionCopy for void return types.
 *
 * Specialises the UnaryFunctionCopy class for void return types.
 *
 * @tparam FunctionObject : type of the function object to be wrapped.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename FunctionObject, typename T>
class UnaryFunctionCopy<FunctionObject,T,void> : public UnaryFunction<T, void>
{
public:
	/**
	 * @brief UnaryFunction child constructor for unary function objects.
	 *
	 * Creates a child of the UnaryFunction class by copying a unary function
	 * object (one that is purely by definition).
	 *
	 * @param f_o : the function object to be assigned to the UnaryFunction child.
	 */
	explicit UnaryFunctionCopy( const FunctionObject &f_o ) : function_object(f_o) {
//		ecl_compile_time_concept_check(UnaryFunction<FunctionObject>);
	}
	virtual ~UnaryFunctionCopy() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief The unary function object call.
	 *
	 * Redirects the call to the composited unary function object call.
	 */
	void operator()(T t) { function_object(t); }

private:
	FunctionObject function_object;
};

/**
 * @brief Creates a unary function from a reference wrapper.
 *
 * Takes a reference wrapper containing a unary function (strictly by definition) object
 * reference and creates a UnaryFunction descendant. This
 * is useful in utilising the inheritance from UnaryFunction (needed for slots
 * and similar classes).
 *
 * @tparam FunctionObject : type of the function object to be referenced.
 * @tparam Result : the return type of the unary function.
 *
 * @sa UnaryFunctionReference<FunctionObject,void>, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename FunctionObject, typename T, typename Result = void>
class UnaryFunctionReference : public UnaryFunction<T, Result>
{
public:
	/**
	 * @brief Creates a UnaryFunction descendant from a reference wrapper.
	 *
	 * Creates a UnaryFunction descendant by reference (not copying).
	 *
	 * @param wrapper : the reference wrapper holding the unary function object to be referenced.
	 */
	explicit UnaryFunctionReference( const ReferenceWrapper<FunctionObject> &wrapper ) : function_object(wrapper.reference()) {
//		ecl_compile_time_concept_check(UnaryFunction<FunctionObject>);
	}

	virtual ~UnaryFunctionReference() {}; /**< @brief This ensures any children objects are deleted correctly. **/
	/**
	 * @brief The unary function object call.
	 *
	 * Redirects the call to the referenced unary function object call.
	 *
	 * @return R : the function's return value.
	 */
	Result operator()(T t) { return function_object(t); }

private:
	FunctionObject &function_object;
};

/**
 * @brief Creates a void unary function from a reference wrapper.
 *
 * Takes a reference wrapper containing a unary function (strictly by definition) object
 * reference and creates a UnaryFunction descendant. This
 * is a specialisation which caters to unary function objects with void return type.
 *
 * @tparam FunctionObject : type of the function object to be referenced.
 *
 * @sa UnaryFunctionReference, @ref functionobjectsGuide "FunctionObjects".
 */
template <typename ReferenceWrapper, typename T>
class UnaryFunctionReference< ReferenceWrapper, T, void > : public UnaryFunction<T,void>
{
public:
	typedef typename ReferenceWrapper::type FunctionObject; /**< The wrapper's function object reference type. **/
	/**
	 * @brief Creates a UnaryFunction descendant from a reference wrapper.
	 *
	 * Creates a UnaryFunction descendant by reference (not copying).
	 *
	 * @param wrapper : the reference wrapper holding the unary function object to be referenced.
	 */
	explicit UnaryFunctionReference( const ReferenceWrapper &wrapper ) : function_object(wrapper.reference()) {
//		ecl_compile_time_concept_check(UnaryFunction<FunctionObject>);
	}
	virtual ~UnaryFunctionReference() {}; /**< @brief This ensures any children objects are deleted correctly. **/

	/**
	 * @brief The unary function object call.
	 *
	 * Redirects the call to the referenced unary function object call.
	 */
	void operator()(T t) { function_object(t); }

private:
	FunctionObject &function_object;
};

/*****************************************************************************
** Nullary Function Generators
*****************************************************************************/

/**
 * @brief Generate a nullary function object from a void global/static function.
 *
 * Overloaded function type, this particular overload generates a nullary
 * function object from a void global/static function.
 *
 * @tparam R : the return type.
 * @param function : the void global/static function.
 * @return NullaryFreeFunction<R> : nullary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename R>
NullaryFreeFunction<R> generateFunctionObject( R (*function)() ) {
	return NullaryFreeFunction<R>( function );
}

/**
 * @brief Generate a unary function object from a unary global/static function.
 *
 * Overloaded function type, this particular overload generates a unary
 * function object from a unary global/static function.
 *
 * @tparam A : the function's argument type.
 * @tparam R : the function's return type.
 * @param function : the unary global/static function.
 * @return UnaryFreeFunction<R> : unary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename A, typename R>
UnaryFreeFunction<A,R> generateFunctionObject( R (*function)(A) ) {
	return UnaryFreeFunction<A,R>( function );
}

/**
 * @brief Generate a nullary function object from a bound unary global/static function.
 *
 * Overloaded function type, this particular overload generates a nullary
 * function object from a bound unary global/static function.
 *
 * @tparam A : the function's argument type.
 * @tparam R : the return type.
 * @tparam I : a mask for the function's argument type.
 * @param function : the void global/static function.
 * @param a : the argument value to bind.
 * @return BoundUnaryFreeFunction<A,R> : nullary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename A, typename R, typename I>
BoundUnaryFreeFunction<A,R> generateFunctionObject( R (*function)(A), I& a ) {
	return BoundUnaryFreeFunction<A,R>( function, a );
}

/**
 * @brief Generate a nullary function object from a bound unary global/static function.
 *
 * Overloaded function type, this particular overload generates a nullary
 * function object from a bound unary global/static function (with supplied const argument).
 * The const argument bind allows for binding from temporaries.
 *
 * @tparam A : the function's argument type.
 * @tparam R : the return type.
 * @tparam I : a mask for the function's argument type.
 * @param function : the void global/static function.
 * @param a : the argument value to bind  (const).
 * @return BoundUnaryFreeFunction<A,R> : nullary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename A, typename R, typename I>
BoundUnaryFreeFunction<A,R> generateFunctionObject( R (*function)(A), const I& a ) {
	return BoundUnaryFreeFunction<A,R>( function, a );
}

/**
 * @brief Generate a unary function object from a nullary member function.
 *
 * Overloaded function type, this particular overload generates a unary
 * function object from a void member function.
 *
 * @tparam C : the member function's class type.
 * @tparam R : the member function's return type.
 * @param function : the void member function.
 * @return NullaryMemberFunction<C,R> : unary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename R>
NullaryMemberFunction<C,R> generateFunctionObject( R (C::*function)() ) {
	return NullaryMemberFunction<C,R>( function );
}

/**
 * @brief Generate a nullary function object by binding a nullary member function with its instance.
 *
 * Overloaded function type, this particular overload generates a nullary
 * function object by binding the class instance to a void member function.
 *
 * @tparam C : the member function's class type.
 * @tparam R : the member function's return type.
 * @param function : the void member function.
 * @param c : the member function's class instance.
 * @return NullaryMemberFunction<C,R> : unary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename R>
BoundNullaryMemberFunction<C,R> generateFunctionObject( R (C::*function)(), C &c ) {
	return BoundNullaryMemberFunction<C,R>( function, c );
}

/**
 * @brief Generate a binary function object from a unary member function.
 *
 * Overloaded function type, this particular overload generates a binary
 * function object from a unary member function.
 *
 * @tparam C : the member function's class type.
 * @tparam A : the member function's argument type.
 * @tparam R : the member function's return type.
 * @param function : the unary member function.
 * @return UnaryMemberFunction<C,A,R> : binary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename A, typename R>
UnaryMemberFunction<C,A,R> generateFunctionObject( R (C::*function)(A) ) {
	return UnaryMemberFunction<C,A,R>( function );
}

/**
 * @brief Generate a unary function object by partially binding a unary member function.
 *
 * Overloaded function type, this particular overload generates a unary
 * function object by binding the class instance but not the argument to
 * a unary member function.
 *
 * @tparam C : the member function's class type.
 * @tparam A : the member function's argument type.
 * @tparam R : the member function's return type.
 *
 * @param function : the void member function.
 * @param c : the member function's class instance.
 * @return PartiallyBoundUnaryMemberFunction<C,R> : nullary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename A, typename R>
PartiallyBoundUnaryMemberFunction<C,A,R> generateFunctionObject( R (C::*function)(A), C& c) {
	return PartiallyBoundUnaryMemberFunction<C,A,R>( function, c);
}

/**
 * @brief Generate a nullary function object by binding a unary member function.
 *
 * Overloaded function type, this particular overload generates a nullary
 * function object by binding the class instance and argument to a unary member function.
 *
 * @tparam C : the member function's class type.
 * @tparam A : the member function's argument type.
 * @tparam R : the member function's return type.
 * @tparam I : a mask for the function's argument type.
 * @param function : the void member function.
 * @param c : the member function's class instance.
 * @param a : the argument value to bind.
 * @return BoundUnaryMemberFunction<C,R> : nullary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename A, typename R, typename I>
BoundUnaryMemberFunction<C,A,R> generateFunctionObject( R (C::*function)(A), C& c, I& a ) {
	// The I here is a bit of a trick...if you use A in the constructor above instead of I
	// then it gets confused trying to deduce A as often the function arg and the input value
	// are different types (e.g. const int& as opposed to int). So fix A in the template
	// direction first, then pass it a type I which can be converted on the fly where needed.
	return BoundUnaryMemberFunction<C,A,R>( function, c, a );
}

/**
 * @brief Generate a nullary function object by binding a unary member function.
 *
 * Overloaded function type, this particular overload generates a nullary
 * function object by binding the class instance and argument to a unary member function.
 * The const argument bind allows for binding from temporaries.
 *
 * @tparam C : the member function's class type.
 * @tparam A : the member function's argument type.
 * @tparam R : the member function's return type.
 * @tparam I : a mask for the function's argument type.
 * @param function : the void member function.
 * @param c : the member function's class instance.
 * @param a : the argument value to bind (const).
 * @return BoundUnaryMemberFunction<C,R> : nullary function object.
 *
 * @sa @ref functionobjectsGuide "FunctionObjects".
 */
template <typename C, typename A, typename R, typename I>
BoundUnaryMemberFunction<C,A,R> generateFunctionObject( R (C::*function)(A), C& c, const I& a ) {
	// This one differs from the previous due to the const reference - this allows things like
	// temporaries to pass through unscathed to a function with an argument like const int&.
	return BoundUnaryMemberFunction<C,A,R>( function, c, a );
}

}; // namespace ecl

#endif /* ECL_UTILITIES_FUNCTION_OBJECTS_HPP_ */
