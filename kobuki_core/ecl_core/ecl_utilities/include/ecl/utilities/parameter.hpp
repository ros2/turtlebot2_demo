/**
 * @file /include/ecl/utilities/parameter.hpp
 *
 * @brief Defines a formalised interface for class parameters.
 *
 * A templatised construct that defines access and configuration
 * interfaces for a simple parameter type embedded in a class.
 *
 * @date April 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_UTILITIES_PARAMETER_HPP_
#define ECL_UTILITIES_PARAMETER_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Parameter
*****************************************************************************/
/**
 * @brief  General parameter type for member variables of a pre-specified class.
 *
 * A Parameter is a templatised construct that formally defines an interface to
 * a simple type embedded in a class.
 *
 * <b>Usage:</b>
 *
 * Definition:
 * @code
 * class A {
 *     public:
 *         Parameter<int> counter;
 * };
 * @endcode
 * and usage (like get/set, but much less verbose):
 * @code
 *
 * int main() {
 *     A a;
 *     a.counter(1); // <-- Sets the variable.
 *     a.counter = 1; // <-- This is also ok.
 *     cout << a.counter(); << endl; // <-- Gets the variable.
 * }
 * @endcode
 *
 * @sa src/test/core/parameters.cpp, @ref parametersGuide "Parameters"
 *
 **/
template <typename T> class Parameter {
    public:
        explicit Parameter() : parameter() {}; /**< Configures the parameter with the type's default value. **/
        /**
         * Configures the parameter with a specified value.
         * @param value : the value to instantiate the parameter with.
         */
        Parameter(const T& value) : parameter(value){};

        virtual ~Parameter() {}

        /******************************************
        ** Operator =
        *******************************************/
        /**
         * @brief Allow assignment from the underlying type.
         *
         * Note that this does not help with conversions for construction (or
         * in argument passing to functions, because that is construction also).
         *
         * @param value : the value to configure the internal variable with.
         */
        void operator =(const T& value) { parameter = value; }
        /**
         * Convenience operator so that it can be treated as a normal variable if you wish
         * (i.e. no () call needed).
         * Note that we do not permit the returned reference to be modified as doing so
         * would mean this class loses control of the variable.
         *
         * @return T& : a const reference to the parameter's variable.
         */
        operator const T&() const { return parameter; }

        /******************************************
        ** Operator ()
        *******************************************/
        /**
         * Formal access to the parameter's variable.
         * Note that we do not permit the returned reference to be modified as doing so
         * would mean this class loses control of the variable.
         *
         * @return T& : a reference to the parameter's variable.
         */
        const T& operator ()() const { return parameter; }
        /**
         * Formal setting operator for the parameter's variable.
         * Note that we dont need the reference here since we're copying into the private
         * variable - you get the same number of constructions either way.
         *
         * @param value : the value to configure the internal variable with.
         */
        void operator ()( const T& value ) { parameter = value; }

    private:
        T parameter;
};

}; // Namespace ecl

#endif /*ECL_UTILITIES_PARAMETER_HPP_*/
