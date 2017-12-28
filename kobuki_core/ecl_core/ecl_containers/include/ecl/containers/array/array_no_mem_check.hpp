/**
 * @file /include/ecl/containers/array/array_no_mem_check.hpp
 *
 * @brief Fixed size containers with a few bells and whistles.
 *
 * @date September 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_ARRAY_NO_MEM_CHECK_HPP_
#define ECL_CONTAINERS_ARRAY_NO_MEM_CHECK_HPP_
#ifndef ECL_MEM_CHECK_ARRAYS

/*****************************************************************************
** Includes
*****************************************************************************/

#include <algorithm> // std::copy
#include <cstddef>  // size_t
#include <iterator>
#include "../definitions.hpp"
#include "../initialiser.hpp"
#include "../stencil.hpp"
#include <ecl/config/macros.hpp>
#include <ecl/concepts/macros.hpp>
#include <ecl/concepts/blueprints.hpp>
#include <ecl/concepts/streams.hpp>
#include <ecl/errors/compile_time_assert.hpp>
#include <ecl/exceptions/standard_exception.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forward Declarations
*****************************************************************************/

namespace blueprints {

template <typename Type, size_t N> class ArrayFactory;
template <typename Derived> class ArrayBluePrint;
template <typename Type, size_t Size> class ConstantArray;

} // namespace blueprints

namespace formatters {

template <typename Type, size_t N> class ArrayFormatter;

} // namespace formatters

/*****************************************************************************
** Interface [Array][Fixed]
*****************************************************************************/
/**
 * @brief  Fixed size container with a few bells and whistles.
 *
 * All of the stl arrays are dynamically sizable, which means they are stored
 * on the heap which is generally much slower for access/write operations than
 * one which is stored on the stack. However, the stl provides no option for
 * a fixed size container which take advantage of this, so this class provides
 * that option. Essentially, its simply a fixed size array with a few bells
 * and whistles.
 *
 * <b>Initialisation</b>
 *
 * We use the comma initialiser (similar to that used in Blitz++ and Eigen2) here
 * for convenient initialisation. However there are other mechanisms, the
 * most interesting of which is Boost's Array class which uses a normal
 * aggregate initialiser (like that of a struct with curly braces). I did
 * some tests to compare the performance of both, and both perform similarly.
 * Since there was no real difference in performance I chose the comma
 * initialiser here as it can also be used after construction and will provide
 * a consistent means of initialisation seeing as we also use Eigen in the Math
 * module. If you're in for strict c++ layouts, then the comma initialiser also
 * lets you hide the base c array as a private member (the aggregate initialiser
 * only works when your base c array is a public member).
 *
 * @code
 * Array<int,4> array; // At this point it is uninitialised.
 * array = 1,2,3,4;    // If NDEBUG is not defined, this will throw if you exceed the range.
 * @endcode
 *
 * <b>Error Handling</b>
 *
 * Arrays will throw exceptions whenever an operation performs an out of range operation.
 * For most operations, this will only occur in debug mode, so release mode will run
 * at full speed. The only exception to this is the accessor function, which provides
 * a handle that will always throw if safety is always required.
 *
 * @code
 * int i = array[2];   // If NDEBUG is not defined, this will throw if you exceed the range.
 * int i = array.at(2); // This will always throw if you exceed the range.
 * @endcode
 *
 * @tparam Type : the type of the element stored in the array.
 *
 * @sa @ref ecl::containers::BoundedListInitialiser "BoundedListInitialiser", @ref blueprints::ArrayFactory "ArrayFactory".
 **/
template<typename Type, std::size_t Size = DynamicStorage>
class ECL_PUBLIC Array : public blueprints::ArrayFactory<Type,Size> {
    public:
        /*********************
        ** Typedefs
        **********************/
        typedef Type           value_type; /**< Array's element type. **/
        typedef Type*          iterator;   /**< Array's iterator type. **/
        typedef const Type*    const_iterator;  /**< Array's constant iterator type. **/
        typedef Type&          reference;  /**< Array's element reference type. **/
        typedef const Type&    const_reference; /**< Array's element const reference type. **/
        typedef std::size_t    size_type;  /**< Array's type used to denote the length of the array. **/
        typedef std::ptrdiff_t difference_type;
        typedef std::reverse_iterator<iterator> reverse_iterator; /**< Array's reverse iterator type. **/
        typedef std::reverse_iterator<const_iterator> const_reverse_iterator;  /**< Array's constant reverse iterator type. **/
        typedef blueprints::ArrayFactory<value_type,Size> Factory; /**< @brief Generates blueprints for this class. **/
        typedef formatters::ArrayFormatter<Type,Size> Formatter; /**< @brief Formatter for this class. **/

        /*********************
        ** Constructors
        **********************/
        /**
         * Default constructor that simply allocates memory, but leaves all the
         * element uninitialised.
         */
        Array() {};
        /**
         * @brief Blueprint constructor.
         *
         * Constructor that allows automatic generation of the array from an
         * existing blueprint. This can be used simply in the following manner
         * for any static element belonging to the @ref ecl::blueprints::ArrayFactory "BluePrintFactory".
         * @code
         * Array<int,4> array = Array<int,4>::Constant(3);
         * @endcode
         * Since this is not explicit, it will also allow assignment.
         * @code
         * Array<int,4> array;
         * array = Array<int,4>::Constant(3);
         * @endcode
         *
         * This will emit a compile time failure if the template argument does
         * not conform to the blueprint concept (refer to ecl_concepts documentation).
         *
         * @param blueprint : the blue print to use to generate this instance.
         *
         * @sa @ref ecl::blueprints::ArrayFactory "ArrayFactory".
         */
        template<typename T>
        Array(const blueprints::ArrayBluePrint< T > &blueprint) {
            ecl_compile_time_concept_check(BluePrintConcept<T>);
            blueprint.implementApply(*this);
        }

        virtual ~Array() {};

        /*********************
        ** Assignment
        **********************/
        /**
         * Provides a comma initialisation facility. This initiates the comma initialiser
         * with an iterator to the underlying array and then leaves the initialiser to
         * do the rest. The initialiser will do range checking if NDEBUG is not defined.
         *
         * @code
         * Array<int,4> array; // At this point it is uninitialised.
         * array << 1,2,3,4;    // If NDEBUG is not defined, this will throw if you exceed the range.
         * @endcode
         *
         * @param value : the first value to enter , ElementType, ArraySizeinto the array.
         * @return BoundedListInitialiser : the comma initialiser mechanism.
         */
        containers::BoundedListInitialiser<Type,Type*,Size> operator<<(const Type &value) {
            return containers::BoundedListInitialiser<value_type,iterator,Size>(value,elements);
        }

        /*********************
        ** Iterators
        **********************/
        /**
         * Generates a pointer (iterator) pointing to the start of the array.
         * @return iterator : points to the beginning of the array.
         */
        iterator begin() { return elements; }
        /**
         * Generates a const pointer (iterator) pointing to the start of the array.
         * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
         */
        const_iterator begin() const { return elements; }
        /**
         * Generates an pointer (iterator) pointing to the end of the array.
         * @return iterator : points to the end of the array.
         */
        iterator end() { return elements+Size; }
        /**
         * Generates a const pointer (iterator) pointing to the end of the array.
         * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
         */
        const_iterator end() const { return elements+Size; }
        /**
         * Generates a reverse iterator pointing to the end of the array.
         * @return reverse_iterator : points to the end of the array.
         */
        reverse_iterator rbegin() { return reverse_iterator(end()); }
        /**
         * Generates a constant reverse iterator pointing to the end of the array.
         * @return const_reverse_iterator : constant reverse iterator pointing to the end of the array.
         */
        const_reverse_iterator rbegin() const { return const_reverse_iterator(end()); }
        /**
         * Generates a reverse iterator pointing to the beginning of the array.
         * @return reverse_iterator : points to the beginning of the array.
         */
        reverse_iterator rend() { return reverse_iterator(begin()); }
        /**
         * Generates a constant reverse iterator pointing to the beginning of the array.
         * @return const_reverse_iterator : constant reverse iterator pointing to the beginning of the array.
         */
        const_reverse_iterator rend() const { return const_reverse_iterator(begin()); }

        /*********************
        ** Front/Back
        **********************/
        /**
         * Generates an reference to the first element in the array.
         * @return reference : reference to the first element in the array.
         */
        reference front() { return elements[0]; }
        /**
         * Generates a constant reference to the first element in the array (cannot change the value).
         * @return const_reference : const_reference to the first element in the array.
         */
        const_reference front() const { return elements[0]; }
        /**
         * Generates an reference to the last element in the array.
         * @return reference : reference to the last element in the array.
         */
        reference back() { return elements[Size-1]; }
        /**
         * Generates a constant reference to the last element in the array (cannot change the value).
         * @return const_reference : const_reference to the last element in the array.
         */
        const_reference back() const { return elements[Size-1]; }

        /*********************
        ** Accessors
        **********************/
        /**
         * @brief Open a window (stencil) onto the array.
         *
         * Opens a window onto the array, providing a similar container-like class to manipulate.
         *
         * @param start_index : start of the stencil window.
         * @param n : number of elements to include in the window.
         * @return Stencil<Array> : the generated stencil.
         *
         * @exception : StandardException : throws if the indices provided are out of range [debug mode only].
         */
        Stencil< Array<Type,Size> > stencil(const unsigned int& start_index, const unsigned int& n) ecl_assert_throw_decl(StandardException) {
        	ecl_assert_throw(start_index < Size, StandardException(LOC, OutOfRangeError, "Start index provided is larger than the underlying array size."));
        	ecl_assert_throw(start_index+n <= Size, StandardException(LOC, OutOfRangeError, "Finish index provided is larger than the underlying array size."));
        	return Stencil< Array<Type,Size> >(*this, begin()+start_index, begin()+start_index+n);
        }

        /**
         * Accesses elements in the array, returning references to the requested element. This
         * accessor only does range checks in debug mode (NDEBUG is not defined). Compare this with
         * the at() accessor which always checks if the range is exceeded.
         *
         * @return reference : a reference to the requested element.
         *
         * @exception : StandardException : throws if range is requested element is out of range [debug mode only].
         **/
        reference operator[](size_type i) ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( i<Size, StandardException(LOC,OutOfRangeError));
            return elements[i];
        }
        /**
         * Accesses elements in the array, returning references to the requested element. This
         * accessor only does range checks in debug mode (NDEBUG is not defined). Compare this with
         * the at() accessor which always checks if the range is exceeded. This also ensures the
         * references are constant, which in turn ensures the contents cannot of the array cannot be
         * modified.
         *
         * @return const_reference : a constant reference to the requested element.
         *
         * @exception : StandardException : throws if range is requested element is out of range [debug mode only].
         **/
        const_reference operator[](size_type i) const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( i<Size, StandardException(LOC,OutOfRangeError));
            return elements[i];
        }
        /**
         * Accesses elements in the array, returning references to the requested element. This
         * accessor always does range checks. Compare this with
         * the [] accessor which only checks if NDEBUG is not defined.
         *
         * @param i : the index of the requested element.
         * @return reference : a reference to the requested element.
         *
         * @exception : StandardException : throws if range is requested element is out of range.
         */
        reference at(size_type i) throw(StandardException) {
            if ( i>=Size ) {
                throw StandardException(LOC,OutOfRangeError);
            }
            return elements[i];
        }
        /**
         * Accesses elements in the array, returning references to the requested element. This
         * accessor always does range checks. Compare this with
         * the [] accessor which only checks if NDEBUG is not defined. This also ensures the
         * references are constant, which in turn ensures the contents cannot of the array cannot be
         * modified.
         *
         * @param i : the index of the requested element.
         * @return const_reference : a const_reference to the requested element.
         *
         * @exception : StandardException : throws if range is requested element is out of range.
         */
        const_reference at(size_type i) const throw(StandardException) {
            if ( i>=Size ) {
                throw StandardException(LOC,OutOfRangeError);
            }
            return elements[i];
        }

        /*********************
        ** Utilities
        **********************/
        /**
         * The size of the array. Since the array storage is fixed, this need only be a static
         * method for the whole class.
         * @return size_type : the size of the array.
         */
        static size_type size() { return Size; }

        /*********************
        ** Streaming
        **********************/
        /**
         * Insertion operator for sending the array to an output stream. This
         * is raw, and has no formatting.
         * @param ostream : the output stream.
         * @param array : the array to be inserted.
         * @return OutputStream : continue streaming with the updated output stream.
         */
        template <typename OutputStream, typename ElementType, size_t ArraySize>
        friend OutputStream& operator<<(OutputStream &ostream , const Array<ElementType,ArraySize> &array);

    private:
        value_type elements[Size];
};

/*****************************************************************************
** Implementation [Array][Insertion Operators]
*****************************************************************************/

template <typename OutputStream, typename ElementType, size_t ArraySize>
OutputStream& operator<<(OutputStream &ostream , const Array<ElementType,ArraySize> &array) {

	ecl_compile_time_concept_check(StreamConcept<OutputStream>);

    ostream << "[ ";
    for(size_t i = 0; i < ArraySize; ++i )
    {
        ostream << array[i] << " ";
    }
    ostream << "]";
    ostream.flush();

    return ostream;
}


/*****************************************************************************
** BluePrints
*****************************************************************************/

namespace blueprints {

/**
 * @brief Parent class for array blueprints.
 *
 * This parent class for array blueprints is necessary to provide a convenient
 * assignment interface to the array class. Without this, no casting to simple
 * fundamental types gets done when using the comma initialiser because they
 * get locked into a templatised blueprint assignment operator. e.g.
 *
 * @code
 * Array<double,4> array;
 * array = 1,2,3,4;
 * @endcode
 * This would cause a concept checking compile time error because the first value
 * is an integer, not a double. Hence this would be passed to the templatised
 * constructor and the concept check would fail. What was intended was that this
 * be automatically casted before being sent to the assignment operator for the
 * double.
 *
 * So here we generate a parent class and design the constructor in the Array
 * to use that. We don't want to lose speed through virtual calls though, so we
 * use the curiously recurring template pattern trick to get us to the goal!
 *
 * @sa @ref ecl::Array "Array".
 */
template <typename Derived>
class ArrayBluePrint {
    public:
        /**
         * A crtp virtual call to its children's instantiate method. The template
         * argument is used, because the crtp base cannot catch typedef's in its
         * children at this point of the compilation. With the template parameter
         * this check is delayed until enough of this base class is known.
         *
         * Note, this cannot have the same name as the blueprint method because
         * then the child will inherit it, even if it doesn't have the class and
         * the concept check will fail.
         *
         * @return BaseType : the type to be generated by the blueprint.
         */
        template <typename BaseType>
        BaseType implementInstantiate() {
            return static_cast<Derived*>(this)->instantiate();
        }
        /**
         * A crtp virtual call to its children's apply method. This acts on the
         * blueprint's target class to configure it according to the blueprint.
         * The template parameter is a trick to delay the determination of the
         * target type, because at this point of the compilation it cannot
         * retrieve the base_type from the derived type.
         *
         * Note, this cannot have the same name as the blueprint method because
         * then the child will inherit it, even if it doesn't have the class and
         * the concept check will fail.
         *
         * @param array : the underlying array to be configured.
         */
        template <typename BaseType>
        void implementApply(BaseType& array) const {
            static_cast<const Derived*>(this)->apply(array);
        }

        virtual ~ArrayBluePrint() {}
};
/**
 * @brief Blueprint for instantiating/configuring an array filled with a constant.
 *
 * Implements the BluePrint concept for instantiating/configuring an array
 * filled with a constant.
 *
 * @sa @ref ecl::Array "Array".
 */
template <typename Type, size_t Size>
class ConstantArray : public ArrayBluePrint< ConstantArray<Type, Size> > {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::Array<Type,Size> base_type;
        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * This constructor saves the input parameters to use when generating the
         * instance later.
         *
         * @param value : the constant value to fill the array with (defaults to zero).
         */
        ConstantArray(const Type& value = 0) : val(value) {}

        virtual ~ConstantArray() {};

        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new array configured with a constant value. Take care
         * not to use this in assignment, as it will do a costly copy of the entire
         * array. It is, however, fine to use when constructing as the copy
         * constructor will simply use this copy for the constructing object.
         *
         * @return Array<Type,Size> : a copy of the array.
         */
        base_type instantiate() {
            ecl::Array<Type,Size> array;
            std::fill_n(array.begin(),Size,val);
            return array;
        }

        /**
         * Fill all elements of an existing array with a constant value.
         * @param array : the array to fill.
         */
        void apply(base_type& array) const {
            std::fill_n(array.begin(),Size,val);
        }

    private:
        const Type& val;
};

/**
 * @brief BluePrint factory for the @ref ecl::Array "Array" class.
 *
 * Generates various blueprints that instantiate or configure arrays with
 * commonly used configurations.
 *
 * @sa @ref ecl::Array "Array".
 */
template<typename Type, size_t Size>
class ArrayFactory {
    public:
        /**
         * Generates a blueprint for creating constant arrays. The array constructor
         * and assignment operator handle the usage of the blueprint from there.
         * @param value : the value to fill the array with.
         * @return ConstantArray : a blueprint for generating or configuring a constant array.
         */
        static ConstantArray<Type,Size> Constant(const Type& value) {
            return ConstantArray<Type,Size>(value);
        }

        virtual ~ArrayFactory() {};
};

} // namespace blueprints
} // namespace ecl

#endif /* !ECL_MEM_CHECK_ARRAYS */
#endif /* ECL_CONTAINERS_ARRAY_NO_MEM_CHECK_HPP_ */
