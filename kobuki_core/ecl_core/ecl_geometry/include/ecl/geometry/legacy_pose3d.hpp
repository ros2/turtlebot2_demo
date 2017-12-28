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

#ifndef ECL_GEOMETRY_LEGACY_POSE3D_HPP_
#define ECL_GEOMETRY_LEGACY_POSE3D_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/linear_algebra.hpp>

#include <ecl/config/macros.hpp>

#include "legacy_pose2d.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [Pose3D]
*****************************************************************************/

/**
 * @brief Parent template definition for Pose3D.
 *
 * Do not use this directly. Use the specialisations instead.
 */
template <class Float, typename Enable = void>
class ECL_LOCAL LegacyPose3D {
private:
        LegacyPose3D() {}; /**< @brief Prevents usage of this template class directly. **/
};

/**
 * @brief Representation for a 3D pose (6 degrees of freedom).
 *
 * This represents a transformation typical of that of an object in 3D,
 * that is, it has 3 translational degrees of freedom and 3 rotational
 * degrees of freedom. Default storage type for
 * rotations is the rotation matrix. We may think about a quaternion
 * storage type in the future.
 *
 * @tparam Float : must be a float type (e.g. float, double, float32, float64)
 */
template<typename Float>
class ECL_PUBLIC LegacyPose3D<Float, typename enable_if<is_float<Float> >::type> {
public:
        /******************************************
        ** Eigen Alignment
        *******************************************/
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html

        /******************************************
        ** Typedef
        *******************************************/
        typedef Float Scalar; /**< @brief Eigen style declaration of the element type. **/
        typedef ecl::linear_algebra::Matrix<Float,3,3> RotationMatrix; /**< @brief The type used to represent rotation matrices. **/
        typedef ecl::linear_algebra::Matrix<Float,3,1> Translation;  /**< @brief The type used to represent translations. **/

        /******************************************
        ** Constructors
        *******************************************/
        /**
         * @brief Initialise with zero rotation and zero translation.
         */
        LegacyPose3D() : rot(RotationMatrix::Identity()), trans(Translation::Zero()) {}

        /**
         * @brief Initialise with rotation matrix and translation.
         *
         * This accepts arbitrary eigen types to use as a rotation matrix and
         * translation vector.
         *
         * @param rotation : 3x3 rotation matrix.
         * @param translation : 3x1 translation vector (will compile error if incorrect size).
         * @tparam Rot   : any compatible eigen 3x3 matrix type (e.g. Matrix<Float,3,3>).
         * @tparam Trans : any compatible eigen 3x1 matrix type (e.g. Matrix<Float,3,1>).
         */
        template<typename Rot, typename Trans>
//      LegacyPose3D(const ecl::linear_algebra::MatrixBase<Rot>& rotation, const ecl::linear_algebra::MatrixBase<Trans>& translation) :
        LegacyPose3D(const ecl::linear_algebra::EigenBase<Rot>& rotation, const ecl::linear_algebra::EigenBase<Trans>& translation) :
                rot(rotation),
                trans(translation)
        {}

        /**
         * @brief Initialise from a 2D pose.
         *
         * Essentially converts the 2D pose to a 3D pose, note that any 2D pose object will do.
         * @param pose : the 2d pose.
         */
        template <enum Pose2DStorageType Storage_>
        LegacyPose3D(const LegacyPose2D<Float,Storage_>& pose) :
            rot(RotationMatrix::Identity()),
            trans(Translation::Zero())
        {
                rot.template block<2,2>(0,0) = pose.rotationMatrix();
                trans.template segment<2>(0) = pose.translation();
        }

        /**
         * @brief Initialise from an eigen AngleAxis and a translation.
         *
         * @param angle_axis : a 3d vector and scalar angle object from eigen.
         * @param translation : 3x1 translation vector (will compile error if incorrect size).
         * @tparam Trans : any compatible eigen 3x1 matrix type (e.g. Matrix<Float,3,1>).
         */
        template<typename Trans>
        LegacyPose3D(const ecl::linear_algebra::AngleAxis<Float>& angle_axis, const ecl::linear_algebra::MatrixBase<Trans>& translation) :
            rot(RotationMatrix::Identity()),
            trans(Translation::Zero())
        {
                /* TODO */
        }
        /**
         * @brief Initialise from an eigen Quaternion and a translation.
         *
         * @param quaternion : an eigen quaternion object.
         * @param translation : 3x1 translation vector (will compile error if incorrect size).
         * @tparam Trans : any compatible eigen 3x1 matrix type (e.g. Matrix<Float,3,1>).
         */
        template<typename Trans>
        LegacyPose3D(const ecl::linear_algebra::Quaternion<Float>& quaternion, const ecl::linear_algebra::MatrixBase<Trans>& translation) :
            rot(quaternion.toRotationMatrix()),
            trans(translation)
        {}

        /**
         * @brief Copy constructor for 3d poses.
         * @param pose : the pose to be copied.
         */
        LegacyPose3D(const LegacyPose3D<Float>& pose) :
                rot(pose.rotation()),
                trans(pose.translation())
        {}

        virtual ~LegacyPose3D() {}

        /******************************************
        ** Assignment
        *******************************************/
        /**
         * @brief Assign from another Pose2D instance.
         * @param pose : another Pose2D, storage type is irrelevant.
         * @return Pose2D<Float>& : reference handle to this object.
         */
        template <enum Pose2DStorageType Storage_>
        LegacyPose3D<Float>& operator=(const LegacyPose2D<Float,Storage_>& pose) {
                rot.template block<2,2>(0,0) = pose.rotationMatrix();
                (rot.template block<2,1>(0,2)) << 0.0, 0.0;
                (rot.template block<1,3>(2,0)) << 0.0, 0.0, 1.0;
                trans.template segment<2>(0) = pose.translation();
                trans[2] = 0.0;
                return *this;
        }

        /**
         * @brief Assign from another Pose2D instance.
         * @param pose : another Pose2D, storage type is irrelevant.
         * @return Pose2D<Float>& : reference handle to this object.
         */
        LegacyPose3D<Float>& operator=(const LegacyPose3D<Float>& pose) {
                rot = pose.rotation();
                trans = pose.translation();
                return *this;
        }
        /******************************************
        ** Eigen Style Setters
        *******************************************/
        /**
         * @brief Set the rotational component.
         *
         * This accepts a rotation matrix to set the rotational part of the pose.
         *
         * @param rotation : input rotation to set.
         */
        void rotation(const RotationMatrix &rotation) {
                rot = rotation;
        }
        /**
         * @brief Set the translation vector.
         *
         * This accepts an arbitrary eigen types to use as a translation vector. Eigen
         * will emit the appropriate compile time error if it is incompatible.
         *
         * @param translation : 3x1 translation vector.
         * @tparam Trans : any compatible eigen 3x1 matrix type (e.g. Matrix<Float,3,1>).
         */
        template <typename Trans>
        void translation(const ecl::linear_algebra::MatrixBase<Trans>& translation) {
                this->trans = translation;
        }
        /******************************************
        ** Convenience Setters
        *******************************************/
        // set from EigenBase (aka affine transforms)
        // set from Quaternions
        // set from AngleAxis

        /******************************************
        ** Eigen Style Accessors
        *******************************************/
        RotationMatrix& rotation() { return rot; } /**> @brief Return a mutable handle to the rotational storage component. **/
        Translation& translation() { return trans; } /**> @brief Return a mutable handle to the translation vector. **/
        const RotationMatrix& rotation() const { return rot; } /**> @brief Return a const handle to the rotational storage component. **/
        const Translation& translation() const { return trans; }  /**> @brief Return a const handle to the translation vector. **/

        /******************************************
        ** Convenience Accessors
        *******************************************/
        // get a Quaternion for the rotation part
        // get an AngleAxis for the rotation part
        // get an Affine Transform (4x4) for the big lebowski
        RotationMatrix rotationMatrix() const { return rot; }  /**< @brief Representation of the rotation in matrix form. **/

        /******************************************
        ** Operators
        *******************************************/
    /**
     * @brief Calculate the inverse pose.
     *
     * This calculates the reverse transformation between frames.
     *
     * @return Pose3D : the inverse pose with target and reference frames swapped.
     */
    LegacyPose3D<Float> inverse() const {
        LegacyPose3D<Float> inverse;
        inverse.rotation(rot.transpose());
        inverse.translation(-1*(inverse.rot*trans));
        return inverse;
    }
    /**
     * @brief Combine this pose with the incoming pose.
     * @param pose : differential (wrt this pose's frame).
     * @return Pose3D<Float> : new instance representing the product of the poses.
     */
    template <typename Float_>
    LegacyPose3D<Float> operator*(const LegacyPose3D<Float_> &pose) const {
                return LegacyPose3D<Float>(rotation()*pose.rotation(),translation() + rotation()*pose.translation());
    }

    // Do we need operator* for Pose2D rh values?
    // Probably better if we code like pose_3d*Pose3D(pose_2d) to avoid vague ambiguities.

    /**
     * @brief Transform this pose by the specified input pose.
     * @param pose : a pose differential (wrt this pose's frame).
     * @return Pose3D<Float>& : handle to the updated pose (wrt common/global frame).
     */
        template <typename Float_>
    LegacyPose3D<Float>& operator*=(const LegacyPose3D<Float_> &pose) {
        // This probably needs checking for aliasing...could also be (marginally) sped up.
        *this = (*this)*pose;
        return (*this);
    }
    /**
     * @brief Relative pose between this pose and another.
     *
     * Evaluates and returns the pose of this pose, relative to the specified pose.
     *
     * @param pose : reference frame for the relative pose.
     * @return Pose3D<Float> : new instance representing the relative.
     */
        template <typename Float_>
    LegacyPose3D<Float> relative(const LegacyPose3D<Float_> &pose) const {
                return pose.inverse()*(*this);
    }

        // how to do pose2d*pose3d without circular header? -> EigenBase? -> External operators?

    /*********************
        ** Streaming
        **********************/
        template <typename OutputStream, typename Float_>
        friend OutputStream& operator<<(OutputStream &ostream , const LegacyPose3D<Float_> &pose);

private:
        RotationMatrix rot;
        Translation trans;
};

/*****************************************************************************
** Insertion Operators
*****************************************************************************/
/**
 * @brief Insertion operator for output streams.
 *
 * Note that the output heading angle is formatted in degrees.
 *
 * @param ostream : stream satisfying the ecl stream concept.
 * @param pose : the inserted pose.
 * @return OutputStream : the returning stream handle.
 */
template <typename OutputStream, typename Float_>
OutputStream& operator<<(OutputStream &ostream , const LegacyPose3D<Float_> &pose) {
        ecl::Format<Float_> format;
        format.width(6);
        format.precision(3);
        for ( unsigned int i = 0; i < 3; ++i ) {
                ostream << "[ ";
                for ( unsigned int j = 0; j < 3; ++j ) {
                        ostream << format(pose.rot(i,j)) << " ";
                }
                ostream << "] [ ";
                ostream << format(pose.trans(i)) << " ]\n";
        }
        ostream.flush();
  return ostream;
}

} // namespace ecl

// This is more convenient and less bughuntish than always assigning allocators with your vectors
// but currently fails on oneiric with gcc 4.6 (bugfixed in eigen, but not yet out in oneiric).
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(ecl::LegacyPose3D<float>)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(ecl::LegacyPose3D<double>)

#endif /* ECL_GEOMETRY_LEGACY_POSE3D_HPP_ */
