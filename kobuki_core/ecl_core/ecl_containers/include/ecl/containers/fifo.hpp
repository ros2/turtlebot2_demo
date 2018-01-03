/**
 * @file /include/ecl/containers/fifo.hpp
 *
 * @brief A simple fifo implementation.
 *
 * Internally it uses ecl::Array for various error handling and other
 * routines.
 *
 * @date August 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_FIFO_HPP_
#define ECL_CONTAINERS_FIFO_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <algorithm> // std::copy
#include <ecl/config/macros.hpp>
#include <ecl/errors/compile_time_assert.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "array.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface
*****************************************************************************/
/**
 * @brief Really simple fifo implementation.
 *
 * For control programs, fifo is typically used for data storage,
 * moving average and some applications. It uses ecl::Array internally for
 * storage (all exception handling is thus handled by ecl::Array).
 *
 * <b>Usage</b>:
 *
 * During the construction of fifo, typically length of buffer may be important.
 *
 * @code
 * FiFo<double> fifo(4);
 * fifo.push_back( data_incoming );
 *
 * But initial value for whole buffer can be important to you.
 * Especially if you want to use fifo for moving average,
 * you should wait for N (length of buffer) incoming of data.
 * But if you use below code, you can use fifo immediately
 *
 * @code
 * FiFo<double> fifo(4, initial_value); // construct with initial value for whole buffer
 * fifo.push_back( next_data );		    // set next data
 * ... 								    // calculate the average here
 * @endcode
 *
 * Data access
 *
 * [0] always return oldest data set from the fifo and [N-1]
 * returns always new data from the fifo once your buffer is full.
 *
 * @code
 * FiFo<double> fifo(4);
 * fifo.push_back(1.0);
 * fifo.push_back(2.0);
 * fifo.push_back(3.0);
 * fifo.push_back(4.0);
 *
 * std::cout << fifo[0] << std::endl;		// 1.0
 * std::cout << fifo[3] << std::endl;		// 4.0
 * @endcode
 *
 * @sa ecl::Array.
 */
template<typename T>
class ECL_PUBLIC FiFo
{
public:
	/**
	 * @brief Initialises the fifo, but does not fill it.
	 * @param length : size of the fifo.
	 */
	FiFo( const unsigned int length = 0 ) :
		size_fifo(length),
		running_index(0)
	{
		if ( size_fifo > 0 ) {
			data.resize( size_fifo );
		}
	}

	/**
	 * @brief Initialise and fill the fifo.
	 * @param length : the size of the fifo.
	 * @param value : the constant to fill the fifo with.
	 */
	FiFo( const unsigned int length, const T &value ) :
		size_fifo(length),
		running_index(0)
	{
		if ( size_fifo > 0 ) {
			data = Array<T>::Constant(size_fifo,value);
		}
	}
	virtual ~FiFo() {} /**< @brief Default destructor. **/

	/**
	 * @brief Returns the indexed value offset from the oldest data.
	 *
	 * This ensures that [0] returns always oldest data from the buffer,
	 * others will be offset from this.
	 *
	 * @param idx : index
	 * @return T& : reference (modifiable) to the indexed element.
	 */
	T & operator[] (int idx) {
		return data[ ((running_index+idx)%size_fifo) ];
	}

	/**
	 * @brief Const version of the [] accessor.
	 *
	 * @param idx : index
	 * @return const T& : const reference to the indexed element.
	 */
	const T & operator[] (int idx) const {
		return data[ ((running_index+idx)%size_fifo) ];
	}

	/**
	 * @brief Push back onto the fifo.
	 *
	 * This will overwrite the oldest element in the fifo.
	 *
	 * @param datum : incoming value.
	 */
	void push_back( const T & datum ) {
		data[ running_index++ ] = datum;
		running_index %= size_fifo;
	}

	/**
	 * @brief One-shot fill method.
	 * @param value : the constant value to fill with.
	 */
	void fill( const T & value ) {
		for( unsigned int i=0; i<size_fifo; i++ ) data[i] = value;
	}

	/**
	 * @brief Index of the oldest element in the fifo.
	 * @return int : the index.
	 */
	unsigned int get_idx() { return running_index;}

	/**
	 * @brief Resize the fifo storage.
	 * @param length : new size.
	 * @exception StandardException : throws if a zero sized storage is specified.
	 */
	void resize( unsigned int length ) ecl_assert_throw_decl(StandardException) {
		size_fifo = length;
		ecl_assert_throw( (size_fifo>0), StandardException(LOC, OutOfRangeError, "SimpleFIFO start with zero size buffer"));
		data.resize( size_fifo );
	}

private:
	ecl::Array <T>data;
	unsigned int size_fifo;
	unsigned int running_index;
};

} // namespace ecl

#endif /* ECL_CONTAINERS_FIFO_HPP_ */
