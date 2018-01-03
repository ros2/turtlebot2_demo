/**
 * @file /include/ecl/geometry/cartesian_point.hpp
 *
 * @brief Cartesian point representations.
 *
 * Im still not real certain about the usefulness of this class.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_CARTESIAN_POINT_HPP_
#define ECL_GEOMETRY_CARTESIAN_POINT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/math.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [CartesianPoint<T,N>]
*****************************************************************************/
/**
 * @brief Generic container storing a cartesian point of dimension N.
 *
 * This simply stores a cartesian point as a vector of dimension N.
 *
 * @tparam T : the underlying element type.
 * @tparam N : the dimension.
 *
 * @sa CartesianPoint<T,3>, CartesianPoint<T,2>.
 */
template <typename T, unsigned int N>
class ECL_PUBLIC CartesianPoint {
public:
	/**
	 * @brief Default constructor - sets all elements to the same value.
	 */
	CartesianPoint(const T &value = T()) : elements(ecl::linear_algebra::Matrix<T,N,1>::Constant(value)) {}
	/**
	 * @brief Initialises with the specified eigen style vector.
	 *
	 * @param point_vector : input eigen style vector.
	 */
	CartesianPoint(const ecl::linear_algebra::Matrix<T,N,1> &point_vector) : elements(point_vector) {}

    /*********************
    ** Assignment
    **********************/
    /**
     * Provides a comma initialisation facility. This initiates the comma initialiser
     * with an iterator to the underlying elements and then leaves the initialiser to
     * do the rest.
     *
     * @code
     * CartesianPoint<double,3> point; // At this point it is initialised with default values.
     * point << 1.0,2.0,3.0;
     * @endcode
     *
     * @param value : the first of three points to enter.
     * @return CommaInitialiser : eigen's comma initialiser mechanism.
     */
	ecl::linear_algebra::CommaInitializer< ecl::linear_algebra::Matrix<T,N,1> > operator<<(const T &value) {
		return elements.operator<<(value);
    }

	/**
	 * @brief Array like accessor.
	 *
	 * @param index : idnex of the element to access.
	 * @return T& : handle to the underlying element.
     * @exception StandardException : throws if the index is outside of the array range [debug mode only].
	 */
	T& operator [](const unsigned int& index) ecl_assert_throw_decl(StandardException) {
        ecl_assert_throw( index<N, StandardException(LOC,OutOfRangeError));
        return elements[index];
    }

	/**
	 * @brief Const array like accessor.
	 *
	 * @param index : idnex of the element to access.
	 * @return const T& : handle to the underlying element.
     * @exception StandardException : throws if the index is outside of the array range [debug mode only].
	 */
	const T& operator [](const unsigned int& index) const ecl_assert_throw_decl(StandardException) {
        ecl_assert_throw( index<N, StandardException(LOC,OutOfRangeError));
        return elements[index];
    }




	/**
	 * @brief Representation of the position vector in eigen vector format.
	 *
	 * Converts representation to an eigen vector reference - oft
	 * necessary in mathematics calculations.
	 * @return Matrix<T,N,1> : a reference to the underlying storage container.
	 */
	ecl::linear_algebra::Matrix<T,N,1>& positionVector() { return elements; }
	/**
	 * @brief Representation of the position vector in const eigen vector format.
	 *
	 * Converts representation to a const eigen vector reference - oft
	 * necessary in mathematics calculations.
	 * @return Matrix<T,N,1> : a reference to the underlying storage container.
	 */
	const ecl::linear_algebra::Matrix<T,N,1>& positionVector() const { return elements; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	ecl::linear_algebra::Matrix<T,N,1> elements;
};

/*****************************************************************************
** Interface [CartesianPoint<T,3>]
*****************************************************************************/
/**
 * @brief Specialisation for a cartesian point of dimension 3.
 *
 * It introduces handles to x(), y(), z() for convenience, whilst still
 * permitting a vector representation of the underlying storage container
 * via the positionVector() method.
 *
 * @tparam T : the underlying element type.
 *
 * @sa CartesianPoint3d, CartesianPoint3f, CartesianPoint3i.
 */
template<typename T>
class ECL_PUBLIC CartesianPoint<T,3> {
public:
	/**
	 * @brief Default constructor - sets all elements to the same value.
	 */
	CartesianPoint(const T &value = T()) : elements(ecl::linear_algebra::Matrix<T,3,1>::Constant(value)) {}
	/**
	 * @brief Initialises with the specified eigen style vector.
	 *
	 * @param vec : input three dimensional eigen style vector.
	 */
	CartesianPoint(const ecl::linear_algebra::Matrix<T,3,1> &vec) : elements(vec) {}
	/**
	 * @brief Initialises the point with the specified values.
	 *
	 * @param x_i : x co-ordinate.
	 * @param y_i : y co-ordinate.
	 * @param z_i : z co-ordinate.
	 */
	CartesianPoint(const T &x_i, const T &y_i, const T &z_i) {
		elements << x_i, y_i, z_i;
	}

	virtual ~CartesianPoint() {}

    /*********************
    ** Assignment
    **********************/
    /**
     * Provides a comma initialisation facility. This initiates the comma initialiser
     * with an iterator to the underlying elements and then leaves the initialiser to
     * do the rest.
     *
     * @code
     * CartesianPoint<double,3> point; // At this point it is initialised with default values.
     * point << 1.0,2.0,3.0;
     * @endcode
     *
     * @param value : the first of three points to enter.
     * @return CommaInitialiser : eigen's comma initialiser mechanism.
     */
	ecl::linear_algebra::CommaInitializer< ecl::linear_algebra::Matrix<T,3,1> > operator<<(const T &value) {
		return elements.operator<<(value);
    }

	/**
	 * @brief Representation of the position vector in eigen vector format.
	 *
	 * Converts representation to an eigen vector reference - oft
	 * necessary in mathematics calculations.
	 * @return Matrix<T,3,1> : a reference to the underlying storage container.
	 */
	ecl::linear_algebra::Matrix<T,3,1>& positionVector() { return elements; }

	/**
	 * @brief Representation of the position vector in const eigen vector format.
	 *
	 * Converts representation to a const eigen vector reference - oft
	 * necessary in mathematics calculations.
	 * @return Matrix<T,3,1> : a reference to the underlying storage container.
	 */
	const ecl::linear_algebra::Matrix<T,3,1>& positionVector() const { return elements; }

	/**
	 * @brief Size/dimension of the cartesian point.
	 *
	 * Size/dimension of the cartesian point.
	 *
	 * @return unsigned int : size/dimension.
	 */
	unsigned int size() { return 3; }

	/**
	 * @brief Returns a constant reference to the x co-ordinate.
	 *
	 * @return const T& : x value.
	 */
	const T& x() const { return elements[0]; }
	/**
	 * @brief Returns a constant reference to the y co-ordinate.
	 *
	 * @return const T& : y value.
	 */
	const T& y() const { return elements[1]; }
	/**
	 * @brief Returns a constant reference to the z co-ordinate.
	 *
	 * @return const T& : z value.
	 */
	const T& z() const { return elements[2]; }
	/**
	 * @brief Returns a copy of the x co-ordinate.
	 *
	 * @return T : x value.
	 */
	T x() { return elements[0]; }
	/**
	 * @brief Returns a copy of the y co-ordinate.
	 *
	 * @return T : y value.
	 */
	T y() { return elements[1]; }
	/**
	 * @brief Returns a copy of the z co-ordinate.
	 *
	 * @return T : z value.
	 */
	T z() { return elements[2]; }

	void x(const T& value) { elements[0] = value; } /**< @brief Sets the x-coordinate. **/
	void y(const T& value) { elements[1] = value; } /**< @brief Sets the y-coordinate. **/
	void z(const T& value) { elements[2] = value; } /**< @brief Sets the z-coordinate. **/

	template <typename OutputStream, typename Type>
	friend OutputStream& operator<<(OutputStream &ostream , const CartesianPoint<Type,3> &point);

private:
	ecl::linear_algebra::Matrix<T,3,1> elements;
};

typedef CartesianPoint<double,3> CartesianPoint3d;  /**< @brief Eigen style convenience handle for x, y, z triples in double format. **/
typedef CartesianPoint<float,3> CartesianPoint3f;  /**< @brief Eigen style convenience handle for x, y, z triples in float format. **/
typedef CartesianPoint<int,3> CartesianPoint3i;  /**< @brief Eigen style convenience handle for x, y, z triples in integer format. **/

/*********************
** Streaming
**********************/
/**
 * Insertion operator for sending the point to an output stream. This
 * is raw, and has no formatting.
 * @param ostream : the output stream.
 * @param point : the point to be inserted.
 * @return OutputStream : continue streaming with the updated output stream.
 */
template <typename OutputStream, typename Type>
OutputStream& operator<<(OutputStream &ostream , const CartesianPoint<Type,3> &point) {
	ostream << "[ " << point.x() << " " << point.y() << " " << point.z() << " ]";
	return ostream;
}

/*****************************************************************************
** Interface [CartesianPoint<T,2>]
*****************************************************************************/
/**
 * @brief Specialisation for a cartesian point of dimension 2.
 *
 * It introduces handles to x(), y(), z() for convenience, whilst still
 * permitting a vector representation of the underlying storage container
 * via the positionVector() method.
 *
 * @tparam T : the underlying element type.
 *
 * @sa CartesianPoint2d, CartesianPoint2f, CartesianPoint2i.
 */
template<typename T>
class ECL_PUBLIC CartesianPoint<T,2> {
public:
	/**
	 * @brief Default constructor - sets all elements to the same value.
	 */
	CartesianPoint(const T &value = T()) : elements(ecl::linear_algebra::Matrix<T,2,1>::Constant(value)) {}
	/**
	 * @brief Initialises with the specified input vector.
	 *
	 * @param vec : input three dimensional vector.
	 */
	CartesianPoint(const ecl::linear_algebra::Matrix<T,2,1> &vec) : elements(vec) {}
	/**
	 * @brief Default copy constructor.
	 *
	 * @param point : copy this point directly.
	 */
	CartesianPoint(const CartesianPoint<T,2> &point ) : elements( point.positionVector() ) {}

	CartesianPoint(const T &x_i, const T &y_i) {
		elements << x_i, y_i;
	}

	virtual ~CartesianPoint() {}

    /*********************
    ** Assignment
    **********************/
    /**
     * Provides a comma initialisation facility. This initiates the comma initialiser
     * with an iterator to the underlying elements and then leaves the initialiser to
     * do the rest. The initialiser will do range checking if NDEBUG is not defined.
     *
     * @code
     * CartesianPoint<double,2> point; // At this point it is initialised with default values.
     * point << 1.0,2.0,2.0;    // If NDEBUG is not defined, this will throw if you exceed the range.
     * @endcode
     *
     * @param value : the first of three points to enter.
     * @return BoundedListInitialiser : the comma initialiser mechanism.
     */
	ecl::linear_algebra::CommaInitializer< ecl::linear_algebra::Matrix<T,2,1> > operator<<(const T &value) {
		return elements.operator<<(value);
    }

	T& operator [](const unsigned int& index) ecl_assert_throw_decl(StandardException) {
        ecl_assert_throw( index<2, StandardException(LOC,OutOfRangeError));
        return elements[index];
    }

	const T  operator [](const unsigned int& index) const ecl_assert_throw_decl(StandardException) {
        ecl_assert_throw( index<2, StandardException(LOC,OutOfRangeError));
        return elements[index];
    }

	const CartesianPoint<T,2> & operator = (const CartesianPoint<T,2> &v)
	{
		elements = v.positionVector();
		return *this;
	}
	/**
	 * @brief Representation of the position vector in eigen vector format.
	 *
	 * Converts representation to an eigen vector reference - oft
	 * necessary in mathematics calculations.
	 * @return Matrix<T,2,1> : a reference to the underlying storage container.
	 */
	ecl::linear_algebra::Matrix<T,2,1>& positionVector() { return elements; }

	/**
	 * @brief Representation of the position vector in const eigen vector format.
	 *
	 * Converts representation to a const eigen vector reference - oft
	 * necessary in mathematics calculations.
	 * @return Matrix<T,2,1> : a reference to the underlying storage container.
	 */
	const ecl::linear_algebra::Matrix<T,2,1>& positionVector() const { return elements; }

	/**
	 * @brief Size/dimension of the cartesian point.
	 *
	 * Size/dimension of the cartesian point.
	 *
	 * @return unsigned int : size/dimension.
	 */
	unsigned int size() { return 2; }

	/**
	 * @brief Returns a constant reference to the x co-ordinate.
	 *
	 * @return const T& : x value.
	 */
	const T& x() const { return elements[0]; }
	/**
	 * @brief Returns a constant reference to the y co-ordinate.
	 *
	 * @return const T& : y value.
	 */
	const T& y() const { return elements[1]; }
	/**
	 * @brief Returns a copy of the x co-ordinate.
	 *
	 * @return T : x value.
	 */
	T x() { return elements[0]; }
	/**
	 * @brief Returns a copy of the y co-ordinate.
	 *
	 * @return T : y value.
	 */
	T y() { return elements[1]; }

	bool	operator == (const CartesianPoint<T,2> & other ) const
	{
		if( other.x() != x() || other.y() != y() ) return false;
		else return true;
	}

	bool	operator != (const CartesianPoint<T,2> & other ) const
	{
		if( other.x() != x() || other.y() != y() ) return true;
		else return false;
	}

	void rotate( const double & angle )
	{
		double c = cos(angle);
		double s = sin(angle);
		double tx = x()*c - y()*s;
		double ty = x()*s + y()*c;

		elements[0] = tx;
		elements[1] = ty;
	}

	double get_angle(void)
	{
		return atan2( y(), x() );
	}

	void operator += ( const  CartesianPoint<T,2> & other )
	{
		elements += other.positionVector();
	}

	T distance ( const CartesianPoint<T,2> & other )
	{
		ecl::linear_algebra::Matrix<T,2,1> temp_element = elements - other.positionVector();
		return static_cast<T>( sqrt( temp_element[0]*temp_element[0]+temp_element[1]*temp_element[1]) );
	}

	T Normalize(void)
	{
		T mag = ecl::EuclideanNorm()( x(), y() );
//		T mag = static_cast<T>(sqrt( x()*x()+y()*y() ));
		if ( mag == static_cast<T>(0) )	// make a unit vector in z-direction
		{
			elements[0] = 1.0;
			elements[1] = 0.0;
		} else
		{
			elements[0] *= (1.0/mag);
			elements[1] *= (1.0/mag);
		}
		return mag;
	}

	CartesianPoint<T,2> operator * (double d) const
	{
		return CartesianPoint<T,2>( elements*d );
	}

	CartesianPoint<T,2> operator + (const CartesianPoint<T,2> &v) const
	{
		return CartesianPoint<T,2>( x()+v.x(), y()+v.y() );
	}

	CartesianPoint<T,2> operator - (const CartesianPoint<T,2> &v) const
	{
		return CartesianPoint<T,2>( x()-v.x(), y()-v.y() );
	}

	double Norm(void)
	{
		return static_cast<double>( ecl::EuclideanNorm()( x(), y() )  );
//		return (sqrt( x()*x()+y()*y() ));
	}

	void x(const T& value) { elements[0] = value; } /**< @brief Sets the x-cor.dinate. **/
	void y(const T& value) { elements[1] = value; } /**< @brief Sets the y-cordinate. **/

	template <typename OutputStream, typename Type>
	friend OutputStream& operator<<(OutputStream &ostream , const CartesianPoint<Type,2> &point);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	ecl::linear_algebra::Matrix<T,2,1> elements;
};

typedef CartesianPoint<double,2> CartesianPoint2d;  /**< @brief Eigen style convenience handle for x, y, z triples in double format. **/
typedef CartesianPoint<float,2> CartesianPoint2f;  /**< @brief Eigen style convenience handle for x, y, z triples in float format. **/
typedef CartesianPoint<int,2> CartesianPoint2i;  /**< @brief Eigen style convenience handle for x, y, z triples in integer format. **/

/*********************
** Streaming
**********************/
/**
 * Insertion operator for sending the array to an output stream. This
 * is raw, and has no formatting.
 * @param ostream : the output stream.
 * @param point : the point to be inserted.
 * @return OutputStream : continue streaming with the updated output stream.
 */
template <typename OutputStream, typename Type>
OutputStream& operator<<(OutputStream &ostream , const CartesianPoint<Type,2> &point) {
	ostream << "[ " << point.x() << " " << point.y() << " ]";
	return ostream;
}

}  // namespace ecl

#endif /* ECL_GEOMETRY_CARTESIAN_POINT_HPP_ */
