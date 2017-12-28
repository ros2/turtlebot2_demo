/**
 * @file /include/ecl/geometry/homogeneous_point.hpp
 *
 * @brief Real valued x-y-z point with homogeneous representation.
 *
 * Essentially a container that stores 4 floats with the last always being
 * fixed at 1.0.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_HOMOGENEOUS_POINT_HPP_
#define ECL_GEOMETRY_HOMOGENEOUS_POINT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/mpl.hpp>
#include <ecl/type_traits.hpp>

#include <ecl/config/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/math.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [HomogeneousPoint]
*****************************************************************************/
/**
 * @brief Dummy parent class for Homogenous points.
 *
 * SFINAE trick to ensure that only floats are used for the template parameter.
 * The real class is in the specialisation.
 */
template <typename T, typename Enable = void>
class HomogeneousPoint : public ecl::FailedObject {};

/**
 * @brief Container storing a homogenous point.
 *
 * Stores float x-y-z triples and a fourth element set to 1.0.
 *
 * @tparam T : the underlying element type - must be float.
 */
template<typename T>
class HomogeneousPoint<T, typename ecl::enable_if< ecl::is_float<T> >::type> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment
	/**
	 * @brief Default constructor - sets all elements to the same value.
	 */
	HomogeneousPoint(const T &value = T()) {
		elements.template block<3,1>(0,0) = ecl::linear_algebra::Matrix<T,3,1>::Constant(value);
		elements(3,0) = 1.0;
	}
	/**
	 * @brief Initialises with the specified eigen style vector.
	 *
	 * This only uses a partial vector (the remaining element gets
	 * automatically filled in as 1.0).
	 *
	 * @param vec : input three dimensional eigen style vector.
	 */
	HomogeneousPoint(const ecl::linear_algebra::Matrix<T,3,1> &vec) {
		elements.template block<3,1>(0,0) = vec;
		elements[3] = 1.0;
	}
	/**
	 * @brief Initialises with the specified eigen style vector in homogeneous format.
	 *
	 * Takes the full vector in this case - last element must always be 1.0.
	 *
	 * @param vec : input three dimensional eigen style vector.
	 */
	HomogeneousPoint(const ecl::linear_algebra::Matrix<T,4,1> &vec) : elements(vec) {
		ecl_assert_throw(elements[3] == 1.0, ecl::StandardException(LOC,ecl::InvalidInputError));
	}
	/**
	 * @brief Initialises the point with the specified values.
	 *
	 * @param x_i : x co-ordinate.
	 * @param y_i : y co-ordinate.
	 * @param z_i : z co-ordinate.
	 */
	HomogeneousPoint(const T &x_i, const T &y_i, const T &z_i) {
		elements << x_i, y_i, z_i, 1.0;
	}

	virtual ~HomogeneousPoint() {}

    /*********************
    ** Assignment
    **********************/
    /**
     * Provides a comma initialisation facility. This initiates the comma initialiser
     * with an iterator to the underlying elements and then leaves the initialiser to
     * do the rest.
     *
     * @code
     * HomogeneousPoint<double> point; // At this point it is initialised with default values.
     * point << 1.0,2.0,3.0,1.0;
     * @endcode
     *
     * @param value : the first of three points + last point must be 1.0.
     * @return CommaInitialiser : eigen's comma initialiser mechanism.
     */
	ecl::linear_algebra::CommaInitializer< ecl::linear_algebra::Matrix<T,4,1> > operator<<(const T &value) {
		return elements.operator<<(value);
    }

	/**
	 * @brief Representation of the position vector in eigen vector format.
	 *
	 * Converts representation to an eigen vector reference - oft
	 * necessary in mathematics calculations.
	 * @return Matrix<T,4,1> : a reference to the underlying storage container.
	 */
	ecl::linear_algebra::Matrix<T,4,1>& positionVector() { return elements; }

	/**
	 * @brief Representation of the position vector in const eigen vector format.
	 *
	 * Converts representation to a const eigen vector reference - oft
	 * necessary in mathematics calculations.
	 * @return Matrix<T,4,1> : a reference to the underlying storage container.
	 */
	const ecl::linear_algebra::Matrix<T,4,1>& positionVector() const { return elements; }

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
	friend OutputStream& operator<<(OutputStream &ostream , const HomogeneousPoint<Type> &point);

private:
	ecl::linear_algebra::Matrix<T,4,1> elements;
};

typedef HomogeneousPoint<float> HomogeneousPointf;  /**< @brief Eigen style convenience handle for homogeneous points in float format. **/
typedef HomogeneousPoint<double> HomogeneousPointd;  /**< @brief Eigen style convenience handle for homogeneous points in double format. **/

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
OutputStream& operator<<(OutputStream &ostream , const HomogeneousPoint<Type> &point) {
	ostream << "[ " << point.x() << " " << point.y() << " " << point.z() << " ]";
	return ostream;
}
}  // namespace ecl

#endif /* ECL_GEOMETRY_HOMOGENEOUS_POINT_HPP_ */
