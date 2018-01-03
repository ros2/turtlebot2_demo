/**
 * @file /include/ecl/utilities/singleton.hpp
 *
 * @brief Singleton construction via the curiously recurring template pattern.
 *
 * Enables singletons created via the curiously recurring template pattern.
 * See the wikipedia entry on singletons for more detail.
 *
 * @sa @ref singletonsGuide "Singletons"
 *
 * @date April, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_UTILITIES_SINGLETON_HPP_
#define ECL_UTILITIES_SINGLETON_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Class [Singleton]
*****************************************************************************/
/**
 * @brief Singleton construction via the curiously recurring template pattern.
 *
 * This class doesn't provide singletons, but enables the construction of
 * referenced based singletons via the curiously recurring template pattern.
 *
 * If you wish to create a singleton, simply:
 * - inherit this class (supplying your class as the template parameter).
 * - this class is declared a friend.
 * - your class has a protected constructor.
 * - use a macro as shown below to call it.
 *
 * @code
 * #define Test TestObject::instance()
 *
 * ...
 *
 * cout << Test.value() << endl;
 * @endcode
 *
 * @sa src/test/singleton.cpp, @ref singletonsGuide "Singletons"
 **/
template<typename T> class Singleton
{
  public:
    /**
     * Stores a reference to a single instantiation of the template object. If
     * the template class has a protected or private default constructor
     * (no arguments), then it ensures the class will be a singleton itself.
     **/
    static T& instance()
    {
        static T the_single_instance;  // assumes T has a protected default constructor
        return the_single_instance;
    }

    virtual ~Singleton () {}
};

}; // namespace ecl

#endif /*ECL_UTILITIES_SINGLETON_HPP_*/
