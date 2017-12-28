/**
 * @file /ecl_geometry/include/ecl/geometry/pose2d.hpp
 *
 * @brief Pose representations.
 *
 * These are particularly suited for mobile robot representaitons.
 *
 * @date September 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_LEGACY_POSE2D_HPP_
#define ECL_GEOMETRY_LEGACY_POSE2D_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/linear_algebra.hpp>

#include <ecl/config/macros.hpp>
#include <ecl/formatters.hpp>
#include <ecl/math/constants.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/mpl/enable_if.hpp>
#include <ecl/type_traits/traits.hpp>
#include <ecl/type_traits/fundamental_types.hpp>
#include "angle.hpp"

/*****************************************************************************
** Enums
*****************************************************************************/

namespace ecl {

/**
 * @brief Used by the traits to select the storage type for Pose2D classes.
 */
enum Pose2DStorageType {
        RotationAngleStorage, //!< RotationAngleStorage
        RotationMatrixStorage,//!< RotationMatrixStorage
};

} // namespace ecl

/*****************************************************************************
** Forward declarations
*****************************************************************************/

namespace ecl {

template <typename Float, enum Pose2DStorageType Storage, typename Enable> class LegacyPose2D;

} // namespace ecl

/*****************************************************************************
** Traits
*****************************************************************************/

namespace ecl {

/**
 * @brief Parent template for ecl traits of the pose classes.
 */
template <typename Float, enum Pose2DStorageType Storage, typename Enable>
class ecl_traits< LegacyPose2D<Float, Storage, Enable> > {};

/**
 * @brief Traits for the pose2D class with rotation matrix storage.
 */
template <typename Float, typename Enable>
class ecl_traits< LegacyPose2D<Float, RotationMatrixStorage, Enable> > {
public:
        typedef Float Scalar; /**< @brief Element type. **/
        typedef ecl::linear_algebra::Matrix<Float,2,2> RotationType;  /**< @brief Rotation storage type (matrix). **/
};

/**
 * @brief Traits for the pose2D class with scalar angle storage.
 */
template <typename Float, typename Enable>
class ecl_traits< LegacyPose2D<Float, RotationAngleStorage, Enable> > {
public:
        typedef Float Scalar; /**< @brief Element type. **/
        typedef Angle<Float> RotationType;  /**< @brief Rotation storage type (angle). **/
};

} // namespace ecl

/*****************************************************************************
** Operations
*****************************************************************************/

namespace ecl {
namespace geometry {

/**
 * @brief Parent template for the pose2D math classes.
 */
template <typename Float, enum Pose2DStorageType Storage> class Pose2DMath {};
/**
 * @brief Math functions/selectors for the pose2D class with rotation matrix storage.
 *
 * This is a mechanism whereby the pose2D class can automatically select the
 * appropriate math functionalities depending on storage, rotation matrix
 * in this case.
 */
template <typename Float>
class ECL_PUBLIC Pose2DMath<Float,RotationMatrixStorage> {
public:
        typedef ecl::linear_algebra::Matrix<Float,2,2> RotationMatrixType; /**@brief Rotation matrix type for pose2D. **/

        static RotationMatrixType Identity() { return RotationMatrixType::Identity(); } /**< @brief Zero rotation. **/
        static RotationMatrixType Rotation(const RotationMatrixType &rotation) { return rotation; } /**< @brief Arbitrary rotation converter. **/
        static RotationMatrixType Rotation(const Angle<Float>& angle) { return angle.rotationMatrix(); }  /**< @brief Arbitrary rotation converter. **/
        static Angle<Float> Heading(const RotationMatrixType &rotation) { return Angle<Float>(rotation); }  /**< @brief Arbitrary heading angle converter. **/
        static RotationMatrixType Product(const RotationMatrixType &rot1, const RotationMatrixType &rot2) { return rot1*rot2; } /**< @brief Overloaded product calculater for poses. **/
        static RotationMatrixType Product(const RotationMatrixType &rotation, const Angle<Float> &angle) { return rotation*angle.rotationMatrix(); } /**< @brief Overloaded product calculater for poses. **/
        static RotationMatrixType Inverse(const RotationMatrixType &rotation) { return rotation.transpose(); } /**< @brief Pose inverse, rotation matrix format. **/
};

/**
 * @brief Math functions/selectors for the pose2D class with rotation angle storage.
 *
 * This is a mechanism whereby the pose2D class can automatically select the
 * appropriate math functionalities depending on storage, rotation angle
 * in this case.
 */
template <typename Float>
class ECL_PUBLIC Pose2DMath<Float,RotationAngleStorage> {
public:
        typedef ecl::linear_algebra::Matrix<Float,2,2> RotationMatrixType; /**@brief Rotation matrix type for pose2D. **/
        static Angle<Float> Identity() { return Angle<Float>(0.0); } /**< @brief Zero rotation. **/
        static Angle<Float> Rotation(const Angle<Float>& radians) { return Angle<Float>(radians); }  /**< @brief Arbitrary rotation converter. **/
        static Angle<Float> Rotation(const RotationMatrixType &rotation) { return Pose2DMath<Float,RotationMatrixStorage>::Heading(rotation); } /**< @brief Arbitrary rotation converter. **/
        static Angle<Float> Heading(const Angle<Float> &angle) { return angle; }  /**< @brief Arbitrary heading angle converter. **/
        static Angle<Float> Product(const Angle<Float> &angle1, const Angle<Float> &angle2) { return angle1+angle2; } /**< @brief Overloaded product calculater for poses. **/
        static Angle<Float> Product(const Angle<Float> &angle, RotationMatrixType &rotation) { return angle + Angle<Float>(rotation); } /**< @brief Overloaded product calculater for poses. **/
        static Angle<Float> Inverse(const Angle<Float> &angle) { return Angle<Float>(-1*angle); } /**< @brief Pose inverse, angle format. **/
};

} // namespace geometry
} // namespace ecl


/*****************************************************************************
** Pose2D
*****************************************************************************/

namespace ecl {

/**
 * @brief Parent template definition for Pose2D.
 *
 * Do not use this directly. Use the specialisations instead.
 */
template <class Float, enum Pose2DStorageType Storage = RotationMatrixStorage, typename Enable = void>
class ECL_PUBLIC LegacyPose2D {
        typedef Float Scalar;
private:
        LegacyPose2D() {}; /**< @brief Prevents usage of this template class directly. **/
};

/**
 * @brief Representation for a 2D pose (3 degrees of freedom).
 *
 * This represents a transformation typical of that of an object in 2D,
 * e.g. a mobile robot on a horizontal plane. There are two storage
 * types for this object, the default is rotation matrix, but scalar
 * angular storage can be selected by supplying a second template
 * parameter to the type:
 *
 * @code
 * LegacyPose2D<double,RotationAngleStorage>
 * @endcode
 *
 * @tparam Float : must be a float type (e.g. float, double, float32, float64)
 * @tparam Storage : type of storage (RotationMatrixStorage, RotationAngleStorage).
 */
template<typename Float, enum Pose2DStorageType Storage>
class ECL_PUBLIC LegacyPose2D<Float, Storage, typename enable_if<is_float<Float> >::type> {
public:
        /******************************************
        ** Eigen Alignment
        *******************************************/
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html

        /******************************************
        ** Typedef
        *******************************************/
        typedef Float Scalar; /**< @brief Eigen style declaration of the element type. **/
        typedef geometry::Pose2DMath<Float,Storage> RotationMath; /**< @brief Rotational math specific to this storage type. **/
        typedef geometry::Pose2DMath<Float,RotationMatrixStorage> RotationMatrixMath; /**< @brief Rotational math specific to rotational matrices. **/
        typedef geometry::Pose2DMath<Float,RotationAngleStorage> RotationAngleMath; /**< @brief Rotational math specific to rotational angles. **/
        typedef typename ecl_traits< LegacyPose2D<Float,Storage,typename enable_if<is_float<Float> >::type> >::RotationType RotationType;  /**< @brief The type used for storage of the rotation/angle. **/
        typedef ecl::linear_algebra::Matrix<Float,2,2> RotationMatrix; /**< @brief The type used to represent rotation matrices. **/
        typedef ecl::linear_algebra::Matrix<Float,2,1> Translation;  /**< @brief The type used to represent translations. **/

        /******************************************
        ** Constructors
        *******************************************/
        /**
         * @brief Default constructor.
         *
         * Initialises the pose with zero rotation and zero translation. Might be worth
         * dropping the setting here (aka eigen style) if we need speed, not safety.
         * If we do so, maybe a debug mode is_initialised flag? Also, if we do so
         * make sure to update the Identity static function.
         */
        LegacyPose2D() : rot( RotationMath::Identity()), trans(Translation::Zero()) {}

        /**
         * @brief Construct the pose using scalars for position and rotation angle.
         *
         * @param  x,y  : position (origin) of the target frame
         * @param angle : initial heading angle (double is compatible)(radians).
         */
        LegacyPose2D(const Float &x, const Float &y, const Angle<Float> &angle) :
                rot( RotationMath::Rotation(angle)),
                trans( (Translation() << x,y).finished() )
        {}

        /**
         * @brief Construct the pose using a rotation matrix and a translation vector.
         *
         * This accepts arbitrary eigen types to use as a rotation matrix and
         * translation vector.
         *
         * @param R : 2x2 rotation matrix.
         * @param T : 2x1 translation vector (will compile error if incorrect size).
         * @tparam Rot   : any compatible eigen 2x2 matrix type (e.g. Matrix<Float,2,2>).
         * @tparam Trans : any compatible eigen 2x1 matrix type (e.g. Matrix<Float,2,1>).
         */
        template<typename Rot, typename Trans>
        LegacyPose2D(const ecl::linear_algebra::MatrixBase<Rot>& R, const ecl::linear_algebra::MatrixBase<Trans>& T) :
                rot( RotationMath::Rotation(R) ),
                trans(T)
        {}
        /**
         * @brief Construct the pose using a rotational angle and a translation vector.
         *
         * This accepts an angle (or its Float equivalent) and an arbitrary eigen type
         * to use as a translation vector.
         *
         * @param angle : initial heading angle (double is compatible)(radians).
         * @param T : 2x1 translation vector (will compile error if incorrect size).
         * @tparam Trans : any compatible eigen 2x1 matrix type (e.g. Matrix<Float,2,1>).
         */
        template<typename Trans>
        LegacyPose2D(const Angle<Float>& angle, const ecl::linear_algebra::MatrixBase<Trans>& T) :
                rot( RotationMath::Rotation(angle) ),
                trans(T)
        {}
        /**
         * @brief Copy constructor that works for copies from any pose type.
         *
         * This works regardless of whatever storage the incoming pose is in.
         *
         * @param pose : the pose to copy.
         */
        template <enum Pose2DStorageType Storage_>
        LegacyPose2D(const LegacyPose2D<Float,Storage_>& pose) :
                rot(RotationMath::Rotation(pose.rotation())),
                trans(pose.translation())
        {}

        virtual ~LegacyPose2D() {}

        /******************************************
        ** Assignment
        *******************************************/
        /**
         * @brief Assign from another Pose2D instance.
         * @param pose : another pose2D, storage type is irrelevant.
         * @return Pose2D<Float>& : reference handle to this object.
         */
        template <enum Pose2DStorageType Storage_>
        LegacyPose2D<Float,Storage>& operator=(const LegacyPose2D<Float,Storage_>& pose) {
                trans = pose.translation();
                rot = RotationMath::Rotation(pose.rotation());
                return *this;
        }

        /******************************************
        ** Eigen Style Setters
        *******************************************/
        /**
         * @brief Set the rotational component with a heading angle.
         *
         * This accepts an input heading angle to configure the internal rotational
         * storage.
         *
         * @param heading : configure the rotation from an input heading angle.
         */
        void rotation(const Angle<Float> &heading) {
                rot = RotationMath::Rotation(heading);
        }
        /**
         * @brief Set the rotational component with a rotation matrix.
         *
         * This accepts an input rotation matrix to configure the internal rotational
         * storage.
         *
         * @param rotation_matrix : input to copy across to the internal rotational storage.
         */
        void rotation(const RotationMatrix &rotation_matrix) {
                rot = RotationMath::Rotation(rotation_matrix);
        }
        /**
         * @brief Set the translation vector from a pair of x,y values.
         *
         * This accepts x, y values to set the pose translation.
         *
         * @param x : x translation.
         * @param y : y translation.
         */
        void translation(const Float &x, const Float &y) {
                this->trans << x, y;
        }
        /**
         * @brief Set the translation vector.
         *
         * This accepts an arbitrary eigen types to use as a translation vector. Eigen
         * will emit the appropriate compile time error if it is incompatible.
         *
         * @param T : 2x1 translation vector.
         * @tparam Trans : any compatible eigen 2x1 matrix type (e.g. Matrix<Float,2,1>).
         */
        template <typename Trans>
        void translation(const ecl::linear_algebra::MatrixBase<Trans>& T) {
                this->trans = T;
        }

        /**
         * @brief Set this pose to zero rotation and zero translation.
         */
        void setIdentity() {
                rot = RotationMath::Identity();
                trans << 0.0, 0.0;
        }

        /**
         * @brief Static function for returning the idenitity pose, eigen style.
         *
         * @return Pose2D<Float,Storage> : the zero rotation, zero translation pose.
         */
        static LegacyPose2D<Float,Storage> Identity() {
                return LegacyPose2D<Float,Storage>();
        }

        /******************************************
        ** Convenience Setters
        *******************************************/
        void setPose(const Float& x, const Float& y, const Angle<Float>& heading) { trans << x,y; rot = RotationMath::Rotation(heading); } /**< @brief Sets the pose with a triplet. **/
        void x(const Float& value) { trans[0] = value; }        //!< @brief Sets the x-coordinate.
        void y(const Float& value) { trans[1] = value; }        //!< @brief Sets the y-coordinate.
        void heading(const Angle<Float>& value) { rot = RotationMath::Rotation(value); } //!< @brief Sets the heading.

    /**
         * @brief Set the rotational component.
         *
         * This accepts arbitrary eigen types to use as a rotation matrix. Eigen
         * will emit the appropriate compile time error if it is incompatible.
         *
         * @param rotation_matrix : 2x2 rotation matrix.
         * @tparam Rot : any compatible eigen 2x2 matrix type (e.g. Matrix<Float,2,2>).
         */
        template <typename Rot>
        void rotationMatrix(const ecl::linear_algebra::MatrixBase<Rot>& rotation_matrix) {
                rot = RotationMath::Rotation(rotation_matrix);
        }

        /******************************************
        ** Eigen Style Accessors
        *******************************************/
        RotationType& rotation() { return rot; } /**> @brief Return a mutable handle to the rotational storage component. **/
        Translation& translation() { return trans; } /**> @brief Return a mutable handle to the translation vector. **/
        const RotationType& rotation() const { return rot; } /**> @brief Return a const handle to the rotational storage component. **/
        const Translation& translation() const { return trans; }  /**> @brief Return a const handle to the translation vector. **/

        /******************************************
        ** Convenience Accessors
        *******************************************/
        Float x() const { return trans[0]; }            //!< @brief Get the x-coordinate.
        Float y() const { return trans[1]; }            //!< @brief Get the y-coordinate.
        Angle<Float> heading() const {  return RotationMath::Heading(rot); }    //!< @brief Get the heading.
        RotationMatrix rotationMatrix() const { return RotationMatrixMath::Rotation(rot); } /**< Representation of the rotation in matrix form. **/

        /******************************************
        ** Operators
        *******************************************/
    /**
     * @brief Calculate the inverse pose.
     *
     * This calculates the reverse transformation between frames.
     *
     * @return Pose2D : the inverse pose with target and reference frames swapped.
     */
    LegacyPose2D<Float,Storage> inverse() const {
        LegacyPose2D<Float,Storage> inverse;
        inverse.rotation(RotationMath::Inverse(rot));
        inverse.translation(-1*(RotationMatrixMath::Rotation(inverse.rot)*trans));
        return inverse;
    }

    /**
     * @brief Combine this pose with the incoming pose.
     * @param pose : differential (wrt this pose's frame).
     * @return Pose2D<Float> : new instance representing the product of the poses.
     */
        template <enum Pose2DStorageType Storage_>
    LegacyPose2D<Float,Storage> operator*(const LegacyPose2D<Float,Storage_> &pose) const {
                return LegacyPose2D<Float,Storage>(RotationMath::Product(rot,pose.rot),trans + rotationMatrix()*pose.trans);
    }
    /**
     * @brief Transform this pose by the specified input pose.
     * @param pose : a pose differential (wrt this pose's frame).
     * @return Pose2D<Float>& : handle to the updated pose (wrt common/global frame).
     */
    LegacyPose2D<Float>& operator*=(const LegacyPose2D<Float> &pose) {
        // This probably needs checking...could also be (marginally) sped up.
        *this = (*this)*pose;
        return (*this);
    }
    /**
     * @brief Relative pose between this pose and another.
     *
     * Evaluates and returns the pose of this pose, relative to the specified pose.
     *
     * @code
     * Pose2D<double> a(1.0,1.0,1.57), b(1.0,2.0,3.14);
     * Pose2D<double> brela = b.relative(a);
     * std::cout << brela << std::endl; // [1.0, 0.0, 1.57]
     * @endcode
     *
     * @param pose : reference frame for the relative pose.
     * @return Pose2D<Float> : new instance representing the relative.
     */
        template <enum Pose2DStorageType Storage_>
    LegacyPose2D<Float,Storage> relative(const LegacyPose2D<Float,Storage_> &pose) const {
                return pose.inverse()*(*this);
    }

    /*********************
        ** Streaming
        **********************/
        template <typename OutputStream, typename Float_, enum Pose2DStorageType Storage_>
        friend OutputStream& operator<<(OutputStream &ostream , const LegacyPose2D<Float_, Storage_> &pose);

private:
        RotationType rot;
        Translation trans;
};

} // namespace ecl

/*****************************************************************************
** Insertion Operators
*****************************************************************************/

namespace ecl {

/**
 * @brief Insertion operator for output streams.
 *
 * Note that the output heading angle is formatted in degrees.
 *
 * @param ostream : stream satisfying the ecl stream concept.
 * @param pose : the inserted pose.
 * @return OutputStream : the returning stream handle.
 */
template <typename OutputStream, typename Float_, enum Pose2DStorageType Storage_>
OutputStream& operator<<(OutputStream &ostream , const LegacyPose2D<Float_,Storage_> &pose) {
        ecl::Format<Float_> format;
        format.width(6);
        format.precision(3);
        ostream << "[ ";
        ostream << format(pose.x()) << " ";
        ostream << format(pose.y()) << " ";
        ostream << format(pose.heading());
        ostream << " ]";
        ostream.flush();
  return ostream;
}


} // namespace ecl

// This is more convenient and less bughuntish than always assigning allocators with your vectors
// but currently fails on oneiric with gcc 4.6 (bugfixed in eigen, but not yet out in oneiric).
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(ecl::LegacyPose2D<float,ecl::RotationAngleStorage>)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(ecl::LegacyPose2D<float,ecl::RotationMatrixStorage>)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(ecl::LegacyPose2D<double,ecl::RotationAngleStorage>)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(ecl::LegacyPose2D<double,ecl::RotationMatrixStorage>)

#endif /* ECL_GEOMETRY_LEGACY_POSE2D_HPP_ */
