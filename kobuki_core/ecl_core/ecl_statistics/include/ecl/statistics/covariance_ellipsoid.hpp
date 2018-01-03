/**
 * @file /include/ecl/statistics/covariance_ellipsoid.hpp
 *
 * @brief Generates the ellipsoid for a multivariate guassian distribution.
 *
 * Generates the ellipsoid for a multivariate guassian distribution.
 *
 * Some of this code is adapted from the matlab script below. Note that we
 * don't do mahalanobis distance conversions - haven't found a dire need
 * for them yet - should do in the future?
 *
 * http://openslam.informatik.uni-freiburg.de/data/svn/tjtf/trunk/matlab/plotcov2.m
 *
 * @date October 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STATISTICS_COVARIANCE_ELLIPSOID_HPP_
#define ECL_STATISTICS_COVARIANCE_ELLIPSOID_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include <ecl/linear_algebra.hpp>
#include <ecl/config/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [CovarianceEllipsoid]
*****************************************************************************/
/**
 * @brief Parent template for covariance ellipsoids.
 *
 * Do not use this directly (yet). Use the specialisations instead.
 *
 * @sa CovarianceEllipsoid<double,2>.
 */
template<typename T, int N>
class ECL_PUBLIC CovarianceEllipsoid {
private:
	/**
	 * @brief Prevents usage of this template class directly.
	 *
	 * This class isn't yet intended for generic use - use the specialisations
	 * instead.
	 */
	CovarianceEllipsoid() {};
	virtual ~CovarianceEllipsoid() {};
};

/*****************************************************************************
** Interface [CovarianceEllipsoid2f]
*****************************************************************************/
/**
 * @brief Specialisation of covariance ellipsoids in two dimensions.
 *
 * This creates ellipsoids for 2x2 covariance matrices, i.e. the usual
 * sort for mobile robot navigation. It uses lovely dirty c-style code
 * to work out the underlying eigen properties rather than Eigen's solver -
 * tends to be a bit faster in only two dimensions.
 *
 * <b>Usage:</b>
 *
 * This class is also more conveniently typedef'd as CovarianceEllipsoid2f.
 *
 * You can either instantiate directly through the constructor or by
 * hand via the compute() method. Recomputing is also possible without
 * having to construct an entirely new instance.
 *
 * @code
 * CovarianceEllipsoid2f ellipsoid(M); // OR
 * CovarianceEllipsoid2f ellipsoid_hand;
 * ellipsoid_hand.compute(M);
 * @endcode
 *
 * Once generated, the ellipse properties can easily be extracted.
 *
 * @code
 * Vector2f lengths = ellipsoid.lengths(); // axis lengths.
 * float angle = ellipsoid.rotation(); // angle of rotation to the major axis
 * @endcode
 *
 * There are a couple more methods that can be accessed for more detailed
 * information about the ellipse, but the above two are usually sufficient
 * for drawing your own ellipses.
 *
 * @sa CovarianceEllipsoid.
 */
template<>
class ECL_PUBLIC CovarianceEllipsoid<float,2> {
public:
	/**
	 * @brief Default constructor, use with compute().
	 *
	 * This constructor does not do anything - you must use it together with the
	 * compute() function. Sometimes useful when using this class with containers.
	 */
	CovarianceEllipsoid();

	/**
	 * @brief Automatically generates ellipsoid information at construction.
	 *
	 * Using the specified input covariance matrix, this constructor
	 * automatically generates (indirectly calls compute()) the full ellipse
	 * information, making it instantly ready for use.
	 *
	 * @param M : the input covariance matrix (must be symmetric positive definite).
	 */
	CovarianceEllipsoid(const ecl::linear_algebra::Matrix2f& M);

	virtual ~CovarianceEllipsoid() {};

	/**
	 * @brief Computes the covariance ellipsoid for the specified matrix.
	 *
	 * This can either be called indirectly via the constructor or directly
	 * via this function. In either case, it resets the object and generates/computes
	 * the resulting ellipse from the specified input covariance matrix.
	 *
	 * @param M : the input covariance matrix (must be symmetric positive definite).
	 */
	void compute(const ecl::linear_algebra::Matrix2f& M);

	/**
	 * @brief Returns the precomputed ellipsoid minor/major axis lengths.
	 *
	 * Returns the precomputed ellipsoid minor/major axis lengths.
	 *
	 * @return Vector2d : the minor/major axis lengths.
	 */
	const ecl::linear_algebra::Vector2f& lengths() { return ellipse_lengths; }
	/**
	 * @brief Returns the ellipse axes/covariance eigen vectors.
	 *
	 * This returns the ellipse axes (aka covariance eigen vectors) as
	 * columns of a matrix.
	 * @return Matrix2d : the set of eigenvectors as column vectors in a matrix.
	 */
	const ecl::linear_algebra::Matrix2f& axes() { return ellipse_axes; }

	/**
	 * @brief Calculates the rotation of the ellipse.
	 *
	 * This determines the angle between the x axis and the major axis
	 * of the ellipse.
	 *
	 * @return double : the rotation angle.
	 */
	double rotation();
	/**
	 * @brief Specifies the intercepts of the ellipse with the cartesian axes.
	 *
	 * This determines the distance from the origin to the ellipse boundary
	 * in the directions of the usual cartesian axes. Note that the intercepts
	 * here are given as a two dimensional vector and this is not the (x,y)
	 * pair of an intercept itself.
	 *
	 * Rather this is both the x and y magnitudes for which the full set of four
	 * intercepts can be generated. These magnitudes are the distances from
	 * the centre of the ellipse to the boundary of the ellipse in both the
	 * x and y directions.
	 *
	 * @return Vector2d : x and y-axis intercept distances from the origin.
	 */
	ecl::linear_algebra::Vector2f intercepts();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	ecl::linear_algebra::Vector2f ellipse_lengths;
	ecl::linear_algebra::Matrix2f ellipse_axes;
};



/**
 * @brief Specialisation of covariance ellipsoids in three dimensions.
 *
 * This creates ellipsoids for 3x3 covariance matrices, i.e. the usual
 * sort for mobile robot navigation. It uses one of the eigen solvers.
 * Currently it seems (we have not checked it thorougly yet) to correlate
 * the order of eigenvalue-eigenvector pairs with the index of the matrix,
 * which is good!
 *
 * <b>Usage:</b>
 *
 * This class is also more conveniently typedef'd as CovarianceEllipsoid3f.
 *
 * You can either instantiate directly through the constructor or by
 * hand via the compute() method. Recomputing is also possible without
 * having to construct an entirely new instance.
 *
 * @code
 * CovarianceEllipsoid3f ellipsoid(M); // OR
 * CovarianceEllipsoid3f ellipsoid_hand;
 * ellipsoid_hand.compute(M);
 * @endcode
 *
 * Once generated, the ellipse properties can easily be extracted.
 *
 * @code
 * Vector3f lengths = ellipsoid.lengths(); // axis lengths.
 * @endcode
 *
 * @sa CovarianceEllipsoid.
 */
template<>
class ECL_PUBLIC CovarianceEllipsoid<float,3> {
public:
	/**
	 * @brief Default constructor, use with compute().
	 *
	 * This constructor does not do anything - you must use it together with the
	 * compute() function. Sometimes useful when using this class with containers.
	 */
	CovarianceEllipsoid();

	/**
	 * @brief Automatically generates ellipsoid information at construction.
	 *
	 * Using the specified input covariance matrix, this constructor
	 * automatically generates (indirectly calls compute()) the full ellipse
	 * information, making it instantly ready for use.
	 *
	 * @param M : the input covariance matrix (must be symmetric positive definite).
	 * @param sort : if true, ensure the resulting set is a right-hand configuration.
	 */
	CovarianceEllipsoid(const ecl::linear_algebra::Matrix3f& M, const bool sort = true);

	virtual ~CovarianceEllipsoid() {};

	/**
	 * @brief Computes the covariance ellipsoid for the specified matrix.
	 *
	 * This can either be called indirectly via the constructor or directly
	 * via this function. In either case, it resets the object and generates/computes
	 * the resulting ellipse from the specified input covariance matrix. By default
	 * it will sort and normalise the resulting eigenvectors so that they form a
	 * right handed system. You can turn this off by setting the sort argument to
	 * false.
	 *
	 * @param M : the input covariance matrix (must be symmetric positive definite).
	 * @param sort : if true, ensure the resulting set is a right-hand configuration.
	 */
	void compute(const ecl::linear_algebra::Matrix3f& M, const bool sort = true);

	/**
	 * @brief Returns the precomputed ellipsoid minor/major axis lengths.
	 *
	 * Returns the precomputed ellipsoid minor/major axis lengths.
	 *
	 * @return Vector2d : the minor/major axis lengths.
	 */
	const ecl::linear_algebra::Vector3f& lengths() { return ellipse_lengths; }
	/**
	 * @brief Returns the ellipse axes/covariance eigen vectors.
	 *
	 * This returns the ellipse axes (aka covariance eigen vectors) as
	 * columns of a matrix.
	 *
	 * @return Matrix2d : the set of eigenvectors as column vectors in a matrix.
	 */
	const ecl::linear_algebra::Matrix3f& axes() { return ellipse_axes; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	ecl::linear_algebra::Vector3f ellipse_lengths;
	ecl::linear_algebra::Matrix3f ellipse_axes;
};

/*****************************************************************************
** Interface [CovarianceEllipsoid2d]
*****************************************************************************/
/**
 * @brief Specialisation of covariance ellipsoids in two dimensions.
 *
 * This creates ellipsoids for 2x2 covariance matrices, i.e. the usual
 * sort for mobile robot navigation. It uses lovely dirty c-style code
 * to work out the underlying eigen properties rather than Eigen's solver -
 * tends to be a bit faster in only two dimensions.
 *
 * <b>Usage:</b>
 *
 * This class is also more conveniently typedef'd as CovarianceEllipsoid2d.
 *
 * You can either instantiate directly through the constructor or by
 * hand via the compute() method. Recomputing is also possible without
 * having to construct an entirely new instance.
 *
 * @code
 * CovarianceEllipsoid2d ellipsoid(M); // OR
 * CovarianceEllipsoid2d ellipsoid_hand;
 * ellipsoid_hand.compute(M);
 * @endcode
 *
 * Once generated, the ellipse properties can easily be extracted.
 *
 * @code
 * Vector2d lengths = ellipsoid.lengths(); // axis lengths.
 * double angle = ellipsoid.rotation(); // angle of rotation to the major axis
 * @endcode
 *
 * There are a couple more methods that can be accessed for more detailed
 * information about the ellipse, but the above two are usually sufficient
 * for drawing your own ellipses.
 *
 * @sa CovarianceEllipsoid.
 */
template<>
class ECL_PUBLIC CovarianceEllipsoid<double,2> {
public:
	/**
	 * @brief Default constructor, use with compute().
	 *
	 * This constructor does not do anything - you must use it together with the
	 * compute() function. Sometimes useful when using this class with containers.
	 */
	CovarianceEllipsoid();

	/**
	 * @brief Automatically generates ellipsoid information at construction.
	 *
	 * Using the specified input covariance matrix, this constructor
	 * automatically generates (indirectly calls compute()) the full ellipse
	 * information, making it instantly ready for use.
	 *
	 * @param M : the input covariance matrix (must be symmetric positive definite).
	 */
	CovarianceEllipsoid(const ecl::linear_algebra::Matrix2d& M);

	virtual ~CovarianceEllipsoid() {};

	/**
	 * @brief Computes the covariance ellipsoid for the specified matrix.
	 *
	 * This can either be called indirectly via the constructor or directly
	 * via this function. In either case, it resets the object and generates/computes
	 * the resulting ellipse from the specified input covariance matrix.
	 *
	 * @param M : the input covariance matrix (must be symmetric positive definite).
	 */
	void compute(const ecl::linear_algebra::Matrix2d& M);

	/**
	 * @brief Returns the precomputed ellipsoid minor/major axis lengths.
	 *
	 * Returns the precomputed ellipsoid minor/major axis lengths.
	 *
	 * @return Vector2d : the minor/major axis lengths.
	 */
	const ecl::linear_algebra::Vector2d& lengths() { return ellipse_lengths; }
	/**
	 * @brief Returns the ellipse axes/covariance eigen vectors.
	 *
	 * This returns the ellipse axes (aka covariance eigen vectors) as
	 * columns of a matrix.
	 * @return Matrix2d : the set of eigenvectors as column vectors in a matrix.
	 */
	const ecl::linear_algebra::Matrix2d& axes() { return ellipse_axes; }

	/**
	 * @brief Calculates the rotation of the ellipse.
	 *
	 * This determines the angle between the x axis and the major axis
	 * of the ellipse.
	 *
	 * @return double : the rotation angle.
	 */
	double rotation();
	/**
	 * @brief Specifies the intercepts of the ellipse with the cartesian axes.
	 *
	 * This determines the distance from the origin to the ellipse boundary
	 * in the directions of the usual cartesian axes. Note that the intercepts
	 * here are given as a two dimensional vector and this is not the (x,y)
	 * pair of an intercept itself.
	 *
	 * Rather this is both the x and y magnitudes for which the full set of four
	 * intercepts can be generated. These magnitudes are the distances from
	 * the centre of the ellipse to the boundary of the ellipse in both the
	 * x and y directions.
	 *
	 * @return Vector2d : x and y-axis intercept distances from the origin.
	 */
	ecl::linear_algebra::Vector2d intercepts();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	ecl::linear_algebra::Vector2d ellipse_lengths;
	ecl::linear_algebra::Matrix2d ellipse_axes;
};



/**
 * @brief Specialisation of covariance ellipsoids in three dimensions.
 *
 * This creates ellipsoids for 3x3 covariance matrices, i.e. the usual
 * sort for mobile robot navigation. It uses one of the eigen solvers.
 * Currently it seems (we have not checked it thorougly yet) to correlate
 * the order of eigenvalue-eigenvector pairs with the index of the matrix,
 * which is good!
 *
 * <b>Usage:</b>
 *
 * This class is also more conveniently typedef'd as CovarianceEllipsoid3d.
 *
 * You can either instantiate directly through the constructor or by
 * hand via the compute() method. Recomputing is also possible without
 * having to construct an entirely new instance.
 *
 * @code
 * CovarianceEllipsoid3d ellipsoid(M); // OR
 * CovarianceEllipsoid3d ellipsoid_hand;
 * ellipsoid_hand.compute(M);
 * @endcode
 *
 * Once generated, the ellipse properties can easily be extracted.
 *
 * @code
 * Vector3d lengths = ellipsoid.lengths(); // axis lengths.
 * @endcode
 *
 * @sa CovarianceEllipsoid.
 */
template<>
class ECL_PUBLIC CovarianceEllipsoid<double,3> {
public:
	/**
	 * @brief Default constructor, use with compute().
	 *
	 * This constructor does not do anything - you must use it together with the
	 * compute() function. Sometimes useful when using this class with containers.
	 */
	CovarianceEllipsoid();

	/**
	 * @brief Automatically generates ellipsoid information at construction.
	 *
	 * Using the specified input covariance matrix, this constructor
	 * automatically generates (indirectly calls compute()) the full ellipse
	 * information, making it instantly ready for use.
	 *
	 * @param M : the input covariance matrix (must be symmetric positive definite).
	 * @param sort : if true, ensure the resulting set is a right-hand configuration.
	 */
	CovarianceEllipsoid(const ecl::linear_algebra::Matrix3d& M, const bool sort = true);

	virtual ~CovarianceEllipsoid() {};

	/**
	 * @brief Computes the covariance ellipsoid for the specified matrix.
	 *
	 * This can either be called indirectly via the constructor or directly
	 * via this function. In either case, it resets the object and generates/computes
	 * the resulting ellipse from the specified input covariance matrix. By default
	 * it will sort and normalise the resulting eigenvectors so that they form a
	 * right handed system. You can turn this off by setting the sort argument to
	 * false.
	 *
	 * @param M : the input covariance matrix (must be symmetric positive definite).
	 * @param sort : if true, ensure the resulting set is a right-hand configuration.
	 */
	void compute(const ecl::linear_algebra::Matrix3d& M, const bool sort = true);

	/**
	 * @brief Returns the precomputed ellipsoid minor/major axis lengths.
	 *
	 * Returns the precomputed ellipsoid minor/major axis lengths.
	 *
	 * @return Vector2d : the minor/major axis lengths.
	 */
	const ecl::linear_algebra::Vector3d& lengths() { return ellipse_lengths; }
	/**
	 * @brief Returns the ellipse axes/covariance eigen vectors.
	 *
	 * This returns the ellipse axes (aka covariance eigen vectors) as
	 * columns of a matrix.
	 *
	 * @return Matrix2d : the set of eigenvectors as column vectors in a matrix.
	 */
	const ecl::linear_algebra::Matrix3d& axes() { return ellipse_axes; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	ecl::linear_algebra::Vector3d ellipse_lengths;
	ecl::linear_algebra::Matrix3d ellipse_axes;
};

/*****************************************************************************
** Convenience Typedefs
*****************************************************************************/
/**
 * @brief Convenience typedef for 2f covariance ellipsoids.
 *
 * Convenience typedef for 2f covariance ellipsoids.
 *
 * @sa ecl::CovarianceEllipsoid<float,2>
 */
typedef CovarianceEllipsoid<float,2> CovarianceEllipsoid2f;
/**
 * @brief Convenience typedef for 2d covariance ellipsoids.
 *
 * Convenience typedef for 2d covariance ellipsoids.
 *
 * @sa ecl::CovarianceEllipsoid<double,2>
 */
typedef CovarianceEllipsoid<double,2> CovarianceEllipsoid2d;
/**
 * @brief Convenience typedef for 3f covariance ellipsoids.
 *
 * Convenience typedef for 3f covariance ellipsoids.
 *
 * @sa ecl::CovarianceEllipsoid<float,3>
 */
typedef CovarianceEllipsoid<float,3> CovarianceEllipsoid3f;

/**
 * @brief Convenience typedef for 3d covariance ellipsoids.
 *
 * Convenience typedef for 3d covariance ellipsoids.
 *
 * @sa ecl::CovarianceEllipsoid<double,3>
 */
typedef CovarianceEllipsoid<double,3> CovarianceEllipsoid3d;


} // namespace ecl

#endif /* ECL_STATISTICS_COVARIANCE_ELLIPSOID_HPP_ */
