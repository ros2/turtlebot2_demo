/**
 * @file /ecl_core_apps/src/benchmarks/eigen3_transforms.cpp
 *
 * @brief Benchmark various aspects of eigen's geometry module.
 *
 * @date September 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <vector>
#include <ecl/config/macros.hpp>
#include <ecl/threads/priority.hpp>
#include <ecl/time/duration.hpp>
#include <ecl/time/cpuwatch.hpp>
#include <ecl/time/stopwatch.hpp>
#include <ecl/time/time_data.hpp>
#include <ecl/linear_algebra.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl;
using namespace Eigen;
using std::vector;

/*****************************************************************************
** Pose2D
*****************************************************************************/

class NewPose2D {
public:
	NewPose2D(){}
	NewPose2D(const double &x, const double &y, const double& theta) :
		rotation(Rotation2D<double>(theta).toRotationMatrix())
	{
		translation << x, y;
	}
	NewPose2D operator*(const NewPose2D& pose) const {
		NewPose2D new_pose;
		new_pose.translation = translation+rotation*pose.translation;
		new_pose.rotation = rotation*pose.rotation;
		return new_pose;
	}
	NewPose2D inverse() const {
		NewPose2D p;
		p.translation = rotation*translation;
		p.rotation = -1*rotation;
		return p;
	}
	void print() {
		Rotation2D<double> angle(0.0);
		angle.fromRotationMatrix(rotation);
		std::cout << "[ " << translation[0] << "," << translation[1] << "," << angle.angle() << "]" << std::endl;
	}

	Matrix2d rotation;
	Vector2d translation;
};

class Pose2D {
public:
	Pose2D(const double &x_=0.0, const double &y_=0.0, const double&theta=0.0) :
		x(x_), y(y_), rotation(theta)
	{}
	Pose2D(const Pose2D& pose) : x(pose.x), y(pose.y), rotation(pose.rotation) {}
	Pose2D inverse() const {
		double s = sin(rotation);
		double c = cos(rotation);
		return Pose2D(c*x-s*y, s*x+c*y, -rotation);
	}

	Pose2D operator*(const Pose2D& pose) const {
		double s = sin(rotation);
		double c = cos(rotation);
		Pose2D new_pose(x+c*pose.x-s*pose.y,
				        y+s*pose.x + c*pose.y,
				        rotation+pose.rotation);
		while (new_pose.rotation > +3.14) { new_pose.rotation -= 2*3.14; }
		while (new_pose.rotation < -3.14) { new_pose.rotation += 2*3.14; }
		return new_pose;
	}

	void print() {
		std::cout << "[ " << x << "," << y << "," << rotation << "]" << std::endl;
	}
	double x, y;
	double rotation;
};

/*****************************************************************************
** Globals
*****************************************************************************/

const unsigned int REPEAT = 1000;
const double increment = 1.0/static_cast<double>(REPEAT);

double random_float() {
	return static_cast<double>(rand()%99)/100.0;
}

Matrix3d rotation3D(const double &alpha, const double &beta, const double &gamma) {
	Matrix3d Ralpha, Rbeta, Rgamma;
	Ralpha << 1.0, 0.0, 0.0,
			  0.0, cos(alpha), -sin(alpha),
			  0.0, sin(alpha), cos(alpha);
	Rbeta <<  cos(beta), 0.0, sin(beta),
			  0.0, 1.0, 0.0,
			  -sin(beta), 0.0, cos(beta);
	Rgamma << cos(gamma), -sin(gamma), 0.0,
			  sin(gamma), cos(gamma), 0.0,
			  0.0, 0.0, 1.0;
	return Ralpha*Rbeta*Rgamma;
}


template <typename MatrixType>
ECL_DONT_INLINE Duration product(const MatrixType &a, const MatrixType &b, MatrixType &c) {
	StopWatch stopwatch;
	c = a*a;
	c = a*b;
	c = b*b;
	return stopwatch.split();
}

template <typename MatrixType>
ECL_DONT_INLINE  Duration inverse(const MatrixType &a, const MatrixType &b, MatrixType &c) {
	StopWatch stopwatch;
	c = a.inverse();
	c = b.inverse();
	c = a.inverse();
	return stopwatch.split();
}

template<typename Pose>
ECL_DONT_INLINE TimeData pose2DTest(bool inverse_test = false) {
	TimeData times;
	Pose p1, p2, p3;
	for ( unsigned int i = 0; i < REPEAT; ++i ) {
		p1 = Pose(i*increment, 2*i*increment, 3.14*i*increment);
		p2 = Pose(1.2*i*increment, 1.2*2*i*increment, 1.14*i*increment);
		if ( inverse_test ) {
			times.push_back(inverse(p1,p2,p3));
		} else {
			times.push_back(product(p1,p2,p3));
		}
	}
	return times;
}

template <typename MatrixType>
ECL_DONT_INLINE TimeData transform2DTest(bool inverse_test = false) {
	TimeData times;
	MatrixType t1 = MatrixType::Identity(), t2(t1), t3(t1);
	for ( unsigned int i = 0; i < REPEAT; ++i ) {
		t1.linear() = Rotation2D<double>(3.14*i*increment).toRotationMatrix();
		Vector2d v; v << i*increment, 2*i*increment;
		t1.translation() = v;
		t2.linear() = Rotation2D<double>(1.14*i*increment).toRotationMatrix();
		t2.translation() = 1.2*v;
		if ( inverse_test ) {
			times.push_back(inverse(t1,t2,t3));
		} else {
			times.push_back(product(t1,t2,t3));
		}
	}
	return times;
}

template <typename MatrixType>
ECL_DONT_INLINE TimeData transform3DTest(bool inverse_test = false) {
	TimeData times;
	MatrixType t1 = MatrixType::Identity(), t2(t1), t3(t1);
	for ( unsigned int i = 0; i < REPEAT; ++i ) {
		t1.linear() = rotation3D(3.14*i*increment,-3.14*i*increment, 1.0+i*increment );
		Vector3d v; v << i*increment, 2*i*increment, 3*i*increment;
		t1.translation() = v;
		t2.linear() = rotation3D(3.14*i*increment,-3.14*i*increment, 1.0+i*increment );
		t2.translation() = 1.2*v;
		if ( inverse_test ) {
			times.push_back(inverse(t1,t2,t3));
		} else {
			times.push_back(product(t1,t2,t3));
		}
	}
	return times;
}

ECL_DONT_INLINE TimeData rotTrans3DTest(bool inverse_test = false) {
	StopWatch stopwatch;
	TimeData times;
	Matrix3d rot1 = Matrix3d::Identity(), rot2(rot1), rot3;
	Vector3d trans1, trans2, trans3;
	for ( unsigned int i = 0; i < REPEAT; ++i ) {
		rot1 = rotation3D(3.14*i*increment,-3.14*i*increment, 1.0+i*increment );
		rot1 = rotation3D(1.14*i*increment,-1.14*i*increment, 1.2+i*increment );
		trans1 << i*increment, 2*i*increment, 3*i*increment;
		trans1 << 2*i*increment, 1*i*increment, 3*i*increment;
		if ( inverse_test ) {
			stopwatch.restart();
			rot3 = rot2*rot1;
			trans3 = rot2*trans1 + trans2;
			rot3 = rot1*rot1;
			trans3 = rot1*trans1 + trans1;
			rot3 = rot2*rot2;
			trans3 = rot2*trans2 + trans2;
			times.push_back(stopwatch.split());
		} else { // inverse
			stopwatch.restart();
		    rot3 = rot1.transpose();
		    trans3 = rot3*trans1*-1;
		    rot3 = rot2.transpose();
		    trans3 = rot3*trans2*-1;
		    rot2 = rot3.transpose();
		    trans2 = rot2*trans3*-1;
			times.push_back(stopwatch.split());
		}
	}
	return times;
}

ECL_DONT_INLINE TimeData matrix3DTest(bool inverse_test = false) {
	TimeData times;
	Matrix4d t1 = Matrix4d::Identity(), t2(t1), t3(t1);
	for ( unsigned int i = 0; i < REPEAT; ++i ) {
		t1.block<3,3>(0,0) = rotation3D(3.14*i*increment,-3.14*i*increment, 1.0+i*increment );
		Vector3d v; v << i*increment, 2*i*increment, 3*i*increment;
		t1.block<3,1>(0,3) = v;
		t2.block<3,3>(0,0) = rotation3D(3.14*i*increment,-3.14*i*increment, 1.0+i*increment );
		t2.block<3,1>(0,3) = 1.2*v;
		if ( inverse_test ) {
			times.push_back(inverse(t1,t2,t3));
		} else {
			times.push_back(product(t1,t2,t3));
		}
	}
	return times;
}

ECL_DONT_INLINE vector<Duration> trigTest() {
	StopWatch stopwatch;
	vector<Duration> times;
	double angle1 = 0.0, angle2 = 1.0, d, e;
	const unsigned long repeats = 1000000000L;
	double inc = 3.14/static_cast<double>(repeats);
	stopwatch.restart();
	for ( unsigned long i = 0; i < repeats; ++i ) {
		angle1 += inc;
		angle2 -= inc;
		d = sin(angle1);
		e = cos(angle2);
	}
	times.push_back(stopwatch.split());
	stopwatch.restart();
	for ( unsigned long i = 0; i < repeats; ++i ) {
		angle1 += inc;
		angle2 -= inc;
		d = angle1*angle2;
		e = 3.315*angle1;
	}
	times.push_back(stopwatch.split());
	return times;
}
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	try {
		ecl::set_priority(ecl::RealTimePriority4);
	} catch ( ecl::StandardException &e ) {
		// dont worry about it.
	}

	TimeData times;
    const bool inverse_test = true; // Just a flag

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Sizes" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Matrix4d     : " << sizeof(Matrix4d) << std::endl;
    std::cout << "3x3+3x1      : " << sizeof(Matrix3d) + sizeof(Vector3d) << std::endl;
    std::cout << "Affine3D     : " << sizeof(Transform<double,3,Affine>) << std::endl;
    std::cout << "AffineCom3D  : " << sizeof(Transform<double,3,AffineCompact>) << std::endl;
    std::cout << "Isometr3D    : " << sizeof(Transform<double,3,Isometry>) << std::endl;
    std::cout << "Rotation2D   : " << sizeof(Rotation2D<double>) << std::endl;
    std::cout << "Translation3D: " << sizeof(Translation<double,3>) << std::endl;
    std::cout << "Quaternion   : " << sizeof(Quaternion<double>) << std::endl;
//
	std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                     Trig vs *" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    vector<Duration> trig_times = trigTest();
    std::cout << "Cos+Sin: " << trig_times[0] << std::endl;
    std::cout << "Two *'s: " << trig_times[1] << std::endl;

	std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Products 2D" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    times.clear(); times = pose2DTest<Pose2D>();
    std::cout << "Pose2D       : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = pose2DTest<NewPose2D>();
    std::cout << "NewPose2D    : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform2DTest< Transform<double,2,Affine> >();
    std::cout << "Affine       : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform2DTest< Transform<double,2,AffineCompact> >();
    std::cout << "CompactAffine: " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform2DTest< Transform<double,2,Isometry> >();
    std::cout << "Isometry     : " << times.average() << " " << times.stdDev() <<  std::endl;
    std::cout << "CompactIso   : N/A" <<  std::endl;

	std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Inverse 2D" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    times.clear(); times = pose2DTest<Pose2D>(inverse_test);
    std::cout << "Pose2D       : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = pose2DTest<NewPose2D>(inverse_test);
    std::cout << "NewPose2D    : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform2DTest< Transform<double,2,Affine> >(inverse_test);
    std::cout << "Affine       : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform2DTest< Transform<double,2,AffineCompact> >(inverse_test);
    std::cout << "CompactAffine: " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform2DTest< Transform<double,2,Isometry> >(inverse_test);
    std::cout << "Isometry     : " << times.average() << " " << times.stdDev() <<  std::endl;
    std::cout << "CompactIso   : N/A" <<  std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Products 3D" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    times.clear(); times = rotTrans3DTest();
    std::cout << "Rot+Tra      : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = matrix3DTest();
    std::cout << "Matrix4d     : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform3DTest< Transform<double,3,Affine> >();
    std::cout << "Affine       : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform3DTest< Transform<double,3,AffineCompact> >();
    std::cout << "CompactAffine: " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform3DTest< Transform<double,3,Isometry> >();
    std::cout << "Isometry     : " << times.average() << " " << times.stdDev() <<  std::endl;
    std::cout << "CompactIso   : N/A" <<  std::endl;

	std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Inverse 3D" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    times.clear(); times = rotTrans3DTest(inverse_test);
    std::cout << "Rot+Tra      : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = matrix3DTest(inverse_test);
    std::cout << "Matrix4d     : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform3DTest< Transform<double,3,Affine> >(inverse_test);
    std::cout << "Affine       : " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform3DTest< Transform<double,3,AffineCompact> >(inverse_test);
    std::cout << "CompactAffine: " << times.average() << " " << times.stdDev() <<  std::endl;
    times.clear(); times = transform3DTest< Transform<double,3,Isometry> >(inverse_test);
    std::cout << "Isometry     : " << times.average() << " " << times.stdDev() <<  std::endl;
    std::cout << "CompactIso   : N/A" <<  std::endl;

    std::cout << std::endl;

//	for ( unsigned int i = 0; i < times.data().size(); ++i ) {
//		std::cout << times.data()[i] << std::endl;
//	}

//	std::cout << std::endl;
//    std::cout << "***********************************************************" << std::endl;
//    std::cout << "                    Pose Unit Tests" << std::endl;
//    std::cout << "***********************************************************" << std::endl;
//    std::cout << std::endl;
//
//    Pose2D p1(1.0,0.0,1.57), p2(0.0,1.0,1.57), p3;
//    NewPose2D o1(1.0,0.0,1.57), o2(0.0,1.0,1.57), o3;
//    p3 = p2*p1;
//    o3 = o2*o1;
//    p3.print();
//    o3.print();
//    p3 = p2.inverse();
//    o3 = o2.inverse();
//    p3.print();
//    o3.print();
	return 0;
}
