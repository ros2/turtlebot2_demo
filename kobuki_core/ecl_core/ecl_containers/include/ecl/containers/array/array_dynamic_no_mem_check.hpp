/**
 * @file /include/ecl/containers/array/array_dynamic_no_mem_check.hpp
 *
 * @brief Dynamically sized containers with a few bells and whistles.
 *
 * @date May, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_ARRAY_DYNAMIC_NO_MEM_CHECK_HPP_
#define ECL_CONTAINERS_ARRAY_DYNAMIC_NO_MEM_CHECK_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <algorithm>
#include <ecl/config/macros.hpp>
#include <ecl/utilities/blueprints.hpp>
#include "../definitions.hpp"
#include "../initialiser.hpp"
#include "../stencil.hpp"
#include "array_no_mem_check.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forward Declarations
*****************************************************************************/

template <typename Type> class Array<Type,DynamicStorage>;

namespace blueprints {

template <typename Type> class ConstantDynamicArray;

} // namespace blueprints

namespace formatters {

template <typename Type, size_t N> class ArrayFormatter;

} // namespace formatters

/*****************************************************************************
** BluePrintFactory
*****************************************************************************/
/**
 * @brief Blueprint factory for dynamic arrays.
 *
 * Generates various blueprints that instantiate or configure dynamic arrays from commonly
 * used algorithms.
 *
 * @sa @ref ecl::Array "Array".
 */
template<typename Type>
class ECL_PUBLIC BluePrintFactory< Array<Type,DynamicStorage> > {
    public:
        /**
         * @brief Generates a constant array of the specified size.
         *
         * Generates a blueprint for creating a dynamically sizeable array with
         * a constant value of the specified size.
         *
         * @return ConstantDynamicArray : the resulting blueprint.
         */
        static blueprints::ConstantDynamicArray<Type> Constant(size_t size, const Type &value) {
            return blueprints::ConstantDynamicArray<Type>(size,value);
        }
        virtual ~BluePrintFactory() {};
};

/*****************************************************************************
** Interface [Array][Dynamic]
*****************************************************************************/
/**
 * @brief  Dynamic size container with a few bells and whistles.
 *
 * This is a specialisation of the fixed array class (for size = 0), but
 * instead reserves memory dynamically at run-time (on the heap, not the stack).
 * Consequently, it is more convenient, but slower than the fixed size array class.
 *
 * <b>Comparisons:</b>
 *
 * This class doesn't do resizing on the fly like the stl vector does. It could
 * be argued that this makes it less convenient, but the control programmer
 * should be manually managing the resizing of the vector anyway (without doing this
 * behaviour will be slower and more unpredictable). So this is not really
 * a disadvantage.
 *
 * On the plus side, like the fixed size array, it utilises the comma initialiser.
 *
 * <b>Usage:</b>
 *
 * This initialises like the fixed size array, but without the size template parameter.
 * Instead, size is managed either through a constructor argument or via one of the
 * storage commands.
 *
 * @code
 * Array<int,4> array; // At this point it is uninitialised.
 * array << 1,2,3,4;    // If NDEBUG is not defined, this will throw if you exceed the range.
 * @endcode
 *
 * Every other facility that is available with fixed size arrays is also available with this class.
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
 * @sa @ref ecl::Array "Array".
 **/
template<typename Type>
class ECL_PUBLIC Array<Type,DynamicStorage> : public BluePrintFactory< Array<Type,DynamicStorage> >  {
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
        typedef formatters::ArrayFormatter<Type,DynamicStorage> Formatter; /**< @brief Formatter for this class. **/

        /** @brief Generates blueprints for this class. **/
        typedef BluePrintFactory< Array<Type,DynamicStorage> > Factory;

        /*********************
        ** Constructors
        **********************/
        /**
         * @brief Default constructor.
         *
         * Does not reserve any storage for the array. Just creates the container object.
         */
        explicit Array() : buffer_size(0), buffer(NULL) {}
        /**
         * @brief Reserves storage for the array.
         *
         * This simply creates a buffer of the requested size on the heap.
         * The values are left uninitialised.
         *
         * @param reserve_size : the number of elements to be allocated to the container.
         */
        explicit Array(const unsigned int reserve_size) : buffer_size(reserve_size), buffer(NULL) {
			buffer = new Type[reserve_size];
        };
        /**
         * @brief Copy constructor.
         *
         * This accepts another dynamic array and uses the stl to copy over the contents.
         *
         * @param array : the array to copy from.
         */
        Array(const Array<Type,DynamicStorage>& array) :
        	Factory(),
        	buffer_size(0),
        	buffer(NULL)
		{
        	if ( array.size() != 0 ) {
				resize(array.size());
				std::copy(array.begin(),array.end(),begin());
        	}
        }
        /**
         * @brief Blueprint constructor.
         *
         * Constructor that allows automatic generation from an
         * existing blueprint. This can be used simply in the following manner
         * for any static element belonging to the BluePrintFactory.
         * @code
         * Array<int> array = Array<int>::Constant(3,4);
         * @endcode
         * Since this is not explicit, it will also allow assignment.
         * @code
         * Array<int> array;
         * array = Array<int>::Constant(3,4);
         * @endcode
         *
         * This will emit a compile time failure if the template argument does
         * not conform to the blueprint concept (refer to ecl_concepts documentation).
         *
         * @param blueprint : the blue print to use to generate this instance.
         */
        template<typename T>
        Array(const blueprints::ArrayBluePrint< T > &blueprint) : buffer_size(0), buffer(NULL) {
            // Note we're using a partially specialised parent interface here otherwise the
            // constructor that reserves sizes as well as the comma initialiser won't
            // conveniently convert types correctly
            // (e.g. Array<double> array(4); array = 1,2,3,4; will fail)
            ecl_compile_time_concept_check(BluePrintConcept<T>);
            blueprint.implementApply(*this);
        }
        /**
         * @brief Default destructor.
         *
         * It cleans up the memory that was used on the heap.
         **/
        ~Array() {
            if ( buffer != NULL ) {
                delete[] buffer;
            }
        }
        /*********************
        ** Assignment
        **********************/
        /**
         * Provides a comma initialisation facility. This initiates the comma initialiser
         * with an iterator to the underlying array and then leaves the initialiser to
         * do the rest. The initialiser will do range checking if NDEBUG is not defined.
         *
         * @code
         * Array<int> array(4); // At this point it is uninitialised.
         * array << 1,2,3,4;    // If NDEBUG is not defined, this will throw if you exceed the range.
         * @endcode
         *
         * @param value : the first value to enter , ElementType, ArraySizeinto the array.
         * @return BoundedListInitialiser : the comma initialiser mechanism.
         */
        containers::BoundedListInitialiser<value_type,iterator,DynamicStorage> operator<<(const Type &value) {
            return containers::BoundedListInitialiser<value_type,iterator,DynamicStorage>(value,buffer,buffer_size);
        }

        void operator=(const Array<Type,DynamicStorage>& array) {
        	if ( array.size() == 0 ) {
        		clear();
        	} else {
				resize(array.size());
				std::copy(array.begin(),array.end(),begin());
        	}
        }

        /*********************
        ** Iterators
        **********************/
        /**
         * Generates a pointer (iterator) pointing to the start of the array.
         * @return iterator : points to the beginning of the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        iterator begin() ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return buffer;
            }
        /**
         * Generates a const pointer (iterator) pointing to the start of the array.
         * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        const_iterator begin() const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return buffer;
        }
        /**
         * Generates an pointer (iterator) pointing to the end of the array.
         * @return iterator : points to the end of the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        iterator end() ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return buffer+buffer_size;
        }
        /**
         * Generates a const pointer (iterator) pointing to the end of the array.
         * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        const_iterator end() const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return buffer+buffer_size;
        }
        /**
         * Generates a reverse iterator pointing to the end of the array.
         * @return reverse_iterator : points to the end of the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        reverse_iterator rbegin() ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return reverse_iterator(end());
        }
        /**
         * Generates a constant reverse iterator pointing to the end of the array.
         * @return const_reverse_iterator : constant reverse iterator pointing to the end of the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        const_reverse_iterator rbegin() const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return const_reverse_iterator(end());
        }
        /**
         * Generates a reverse iterator pointing to the beginning of the array.
         * @return reverse_iterator : points to the beginning of the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        reverse_iterator rend() ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return reverse_iterator(begin());
        }
        /**
         * Generates a constant reverse iterator pointing to the beginning of the array.
         * @return const_reverse_iterator : constant reverse iterator pointing to the beginning of the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        const_reverse_iterator rend() const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return const_reverse_iterator(begin());
        }

        /*********************
        ** Front/Back
        **********************/
        /**
         * Generates an reference to the first element in the array.
         * @return reference : reference to the first element in the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        reference front() ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return buffer[0];
        }
        /**
         * Generates a constant reference to the first element in the array (cannot change the value).
         * @return const_reference : const_reference to the first element in the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        const_reference front() const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return buffer[0];
        }
        /**
         * Generates an reference to the last element in the array.
         * @return reference : reference to the last element in the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        reference back() ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return buffer[buffer_size-1];
        }
        /**
         * Generates a constant reference to the last element in the array (cannot change the value).
         * @return const_reference : const_reference to the last element in the array.
         *
         * @exception StandardException : throws if no storage has been allocated [debug mode only].
         */
        const_reference back() const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( buffer != NULL, StandardException(LOC,OutOfRangeError) );
            return buffer[buffer_size-1];
        }

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
        Stencil< Array<Type,DynamicStorage> > stencil(const unsigned int& start_index, const unsigned int& n) ecl_assert_throw_decl(StandardException) {
        	ecl_assert_throw(start_index < size(), StandardException(LOC, OutOfRangeError, "Start index provided is larger than the underlying array size."));
        	ecl_assert_throw(start_index+n <= size(), StandardException(LOC, OutOfRangeError, "Finish index provided is larger than the underlying array size."));
        	return Stencil< Array<Type,DynamicStorage> >(*this, begin()+start_index, begin()+start_index+n);
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
            ecl_assert_throw( i<buffer_size, StandardException(LOC,OutOfRangeError));
            return buffer[i];
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
            ecl_assert_throw( i<buffer_size, StandardException(LOC,OutOfRangeError));
            return buffer[i];
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
            if ( i>=buffer_size ) {
                throw StandardException(LOC,OutOfRangeError);
            }
            return buffer[i];
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
            if ( i>=buffer_size ) {
                throw StandardException(LOC,OutOfRangeError);
            }
            return buffer[i];
        }

        /*********************
        ** Utilities
        **********************/
        /**
         * @brief The size of the array.
         *
         * Returns the current (dynamic) size of the array.
         *
         * @return size_type : the size of the array.
         */
        size_type size() const { return buffer_size; }

        /**
         * @brief Resize the array, clearing whatever was in there before.
         *
         * This resizes the array. Take care with this as it will clear whatever was
         * previously stored in the buffer. All values are uninitialised after clearing.
         *
         * @param n : the new size to be allocated for the array.
         */
        void resize( size_t n ) {
            if ( buffer != NULL ) {
                delete[] buffer;
            }
            buffer = new Type[n];
            buffer_size = n;
        }
        /**
         * @brief Clear the array, deleting all storage space previously allocated.
         *
         * Clear the array, deleting all storage space previously allocated.
         */
        void clear() {
            if ( buffer != NULL ) {
                delete[] buffer;
                buffer = NULL;
            }
            buffer_size = 0;
        }

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
        template <typename OutputStream, typename ElementType>
        friend OutputStream& operator<<(OutputStream &ostream , const Array<ElementType,DynamicStorage> &array);

    private:
        unsigned int buffer_size;
        Type *buffer;

};

/*****************************************************************************
** Implementation [Array]
*****************************************************************************/

template <typename OutputStream, typename ElementType>
OutputStream& operator<<(OutputStream &ostream , const Array<ElementType,DynamicStorage> &array) {

    ostream << "[ ";
    for(size_t i = 0; i < array.buffer_size; ++i )
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

/*****************************************************************************
** Interface [ConstantDynamicArray]
*****************************************************************************/

/**
 * @brief  Blueprint for generating a cubic spline satisfying C2 constraints.
 *
 * Generates a blueprint for creating cubic splines on a set of data
 * satisfying a heuristic that automatically generates y' values at the
 * via points.
 **/
template <typename Type>
class ConstantDynamicArray: public ArrayBluePrint< ConstantDynamicArray<Type> >  {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::Array<Type,ecl::DynamicStorage> base_type;
        /**
         * @brief Default constructor.
         *
         * Default constructor (only utilised by the blueprint compile time assert).
         */
        ConstantDynamicArray() {};

        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * This constructor saves the input parameters to use when generating the
         * instance later.
         *
         * @param size : the size of the array to create.
         * @param value : the constant value to fill the array with (defaults to zero).
         */
        ConstantDynamicArray(size_t size, const Type &value ) :
            reserve_size(size),
            val(value)
            {}

        virtual ~ConstantDynamicArray() {};

        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new array configured with a constant value. Take care
         * not to use this in assignment, as it will do a costly copy of the entire
         * array. It is, however, fine to use when constructing as the copy
         * constructor will simply use this copy for the constructing object.
         *
         * @return Array<Type> : a copy of the array.
         */
        base_type instantiate() {
            ecl::Array<Type> array(reserve_size);
            std::fill_n(array.begin(),reserve_size,val);
            return array;
        }

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Fill all elements of an existing array with a constant value.
         * Note that this clears whatever was initially in the array.
         *
         * @param array : the array to fill.
         */
        void apply(base_type& array) const {
            array.resize(reserve_size);
            std::fill_n(array.begin(),reserve_size,val);
        }

    private:
        size_t reserve_size;
        Type val;
};


} // namespace blueprints
} // namespace ecl


#endif /* ECL_CONTAINERS_ARRAY_DYNAMIC_NO_MEM_CHECK_HPP_ */
