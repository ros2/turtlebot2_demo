/**
 * @file /ecl_time/include/ecl/time/timestamp_base.hpp
 *
 * @brief Base class with common functionality across platforms for timestamps.
 *
 * @date May 24, 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_TIMESTAMP_BASE_HPP_
#define ECL_TIME_TIMESTAMP_BASE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdlib>
#include <ecl/time_lite/types.hpp>
#include <ecl/config/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation
*****************************************************************************/
/**
 * @brief This class provides common functionality across platforms for the timestamps.
 *
 * Most of the operations on a timestamp class is common from one platform
 * to the next. These functions are bundled together in this class.
 */
class ecl_time_PUBLIC TimeStampBase {
public:
    /*********************
    ** Constructors
    **********************/
	explicit TimeStampBase() {}; /**< @brief Default constructor. **/
    /**
     * @brief Initialise the timestamp with a default value.
     *
     * Initialises the timestamp with the specified double value. Note that this is
     * slower than the manual initialisation with a sec/nsec pair.
     *
     * @param decimal_time_value : time units in seconds (integral part) and nanoseconds (decimal part).
     *
     * @exception StandardException : throws if the input arguments is not positive [debug mode only].
     */
    explicit TimeStampBase (const double& decimal_time_value) ecl_assert_throw_decl(StandardException);
    /**
     * @brief Initialises the timestamp with the specified sec/nsec pair.
     *
     * This is the fastest means of initialisation to a set time value.
     *
     * @param seconds : discrete time value measured in whole seconds.
     * @param nanoseconds : fraction of a second measured in nanoseconds.
     * @exception StandardException : throws if input arguments are not positive [debug mode only].
     **/
    TimeStampBase (const time_t &seconds, const long &nanoseconds) ecl_assert_throw_decl(StandardException);

    virtual ~TimeStampBase() {}

    /******************************************
	** Stamps
	*******************************************/
	/**
	 * @brief Manually set the timestamp.
	 *
	 * Manually sets the timestamp with the specified double value. Note that this is
	 * slower than the stamp with a sec/nsec pair.
	 *
	 * @param decimal_time_value : time units in seconds (integral part) and nanoseconds (decimal part).
	 *
	 * @exception StandardException : throws if the input arguments is not positive [debug mode only].
	 */
	const TimeStampBase& stamp (const double& decimal_time_value) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Manually sets the timestamp with the specified values.
	 *
	 * This is the fastest way of setting the timestamp.
	 *
	 * @param seconds : discrete time value measured in whole seconds.
	 * @param nanoseconds : fraction of a second measured in nanoseconds.
	 *
	 * @exception StandardException : throws if input arguments are not positive [debug mode only].
	 */
	const TimeStampBase& stamp (const time_t &seconds, const long &nanoseconds) ecl_assert_throw_decl(StandardException);

    /******************************************
    ** Accessors
    *******************************************/
	/**
	 * @brief Seconds component.
	 *
	 * Get the current number of whole seconds recorded by this timestamp.
	 * @return Seconds : the number of whole seconds recorded by this timestamp.
	 */
	long sec() const { return time.tv_sec; }
	/**
	 * @brief Milliseconds component (after removing seconds).
	 *
	 * Get the fractional number of milliseconds recorded in this timestamp.
	 * @return long : the fractional amount of time recorded by this timestamp (in ms).
	 */
	long msec() const { return time.tv_nsec/1000000L; }
	/**
	 * @brief Microseconds component (after removing seconds).
	 *
	 * Get the fractional number of microseconds recorded in this timestamp.
	 * @return long : the fractional amount of time recorded by this timestamp (in us).
	 */
	long usec() const { return time.tv_nsec/1000; }
	/**
	 * @brief Nanoseconds component (after removing seconds).
	 *
	 * Get the fractional number of nanoseconds recorded in this timestamp.
	 * @return long : the fractional amount of time recorded by this timestamp (in ns).
	 */
	long nsec() const { return time.tv_nsec; }

	/**
	 * @brief Disguises the timestamp as a regular double.
	 *
	 * Converts the posix timespec structure to a double (seconds->integral part,
	 * nanoseconds->fractional part). This is sometimes useful in time calculations.
	 */
	operator double() const { return ( time.tv_sec + time.tv_nsec*0.000000001); }

	/******************************************
	** Comparison Operators
	*******************************************/
	/**
	 * Equality operator.
	 * @param time_stamp : the rhv (timestamp) to be compared to this instance.
	 * @return bool : the result of the comparison operation.
	 */
	bool operator==(const TimeStampBase& time_stamp);
	/**
	 * Inequality operator.
	 * @param time_stamp : the rhv (timestamp) to be compared to this instance.
	 * @return bool : the result of the comparison operation.
	 */
	bool operator!=(const TimeStampBase& time_stamp);
	/**
	 * Less than or equal to operator.
	 * @param time_stamp : the rhv (timestamp) to be compared to this instance.
	 * @return bool : the result of the comparison operation.
	 */
	bool operator<=(const TimeStampBase& time_stamp);
	/**
	 * Greater than or equal to operator.
	 * @param time_stamp : the rhv (timestamp) to be compared to this instance.
	 * @return bool : the result of the comparison operation.
	 */
	bool operator>=(const TimeStampBase& time_stamp);
	/**
	 * Less than operator.
	 * @param time_stamp : the rhv (timestamp) to be compared to this instance.
	 * @return bool : the result of the comparison operation.
	 */
	bool operator<(const TimeStampBase& time_stamp);
	/**
	 * Greater than or equal to operator.
	 * @param time_stamp : the rhv (timestamp) to be compared to this instance.
	 * @return bool : the result of the comparison operation.
	 */
	bool operator>(const TimeStampBase& time_stamp);

	/******************************************
	** Mathematical Operators
	*******************************************/
	/**
	 * Sum operator that returns a new timestamp holding the sum.
	 * @param time_stamp : the rhv (timestamp to be added to this instance).
	 * @return TimeStamp : the sum of the two timestamps.
	 */
	TimeStampBase operator+(const TimeStampBase& time_stamp );
	/**
	 * Sum operator that modifies this instance with the result.
	 * @param time_stamp : the rhv (timestamp to be added to this instance).
	 */
	void operator+=(const TimeStampBase& time_stamp);
	/**
	 * Difference operator that returns a new timestamp holding the difference.
	 * Note that a difference that would result in a negative timestamp is not
	 * permitted (design requirement). Subsequently, this method will throw when
	 * in debug mode if this situation occurs.
	 * @param time_stamp : the rhv (timestamp to be subtracted from this instance).
	 * @return TimeStamp : the difference of the two timestamps.
	 * @exception StandardException : throws if difference would create a negative timestamp [debug mode only].
	 */
	TimeStampBase operator-(const TimeStampBase& time_stamp ) ecl_assert_throw_decl(StandardException);
	/**
	 * Difference operator that modifies this instance with the result.
	 * Note that a difference that would result in a negative timestamp is not
	 * permitted (design requirement). Subsequently, this method will throw when
	 * in debug mode if this situation occurs.
	 * @param time_stamp : the rhv (timestamp to be subtracted from this instance).
	 * @exception StandardException : throws if difference would create a negative timestamp [debug mode only].
	 */
	void operator-=(const TimeStampBase& time_stamp) ecl_assert_throw_decl(StandardException);

    /******************************************
    ** Insertion Operator
    *******************************************/
    template <typename OutputStream>
    friend OutputStream& operator << ( OutputStream &ostream , const TimeStampBase& time_stamp );

protected:
    TimeStructure time;
};


/*****************************************************************************
** Implementation [Insertion Operator]
*****************************************************************************/

template <typename OutputStream>
OutputStream& operator <<( OutputStream &ostream , const TimeStampBase& time_stamp )
{
    if ( ( time_stamp.time.tv_sec == 0 ) && (time_stamp.time.tv_nsec < 0 ) ) {
      ostream << "-";
    }
    ostream << time_stamp.time.tv_sec << ".";
    long nanoseconds = std::abs(time_stamp.time.tv_nsec);
    if ( nanoseconds < 10 ) {
        ostream << "00000000";
    } else if ( nanoseconds < 100 ) {
        ostream << "0000000";
    } else if ( nanoseconds < 1000 ) {
        ostream << "000000";
    } else if ( nanoseconds < 10000 ) {
        ostream << "00000";
    } else if ( nanoseconds < 100000 ) {
        ostream << "0000";
    } else if ( nanoseconds < 1000000 ) {
        ostream << "000";
    } else if ( nanoseconds < 10000000 ) {
        ostream << "00";
    } else if ( nanoseconds < 100000000 ) {
        ostream << "0";
    }
    ostream << nanoseconds;
    return ostream;
}

} // namespace ecl

#endif /* ECL_TIME_TIMESTAMP_BASE_HPP_ */



