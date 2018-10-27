/**
   @author
*/

#ifndef UTIL_H
#define UTIL_H


#include <memory>
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>   


using namespace Eigen;

namespace ljnoid {

const double PI = 3.14159265358979323846;
const double PI_2 = 1.57079632679489661923;

const double TO_DEGREE = 180.0 / PI;
const double TO_RADIAN = PI / 180.0;

inline double degree(double rad) { return TO_DEGREE * rad; }
inline double radian(double deg) { return TO_RADIAN * deg; }
inline float degree(float rad) { return (float)TO_DEGREE * rad; }
inline float radian(float deg) { return (float)TO_RADIAN * deg; }
inline double radian(int deg) { return TO_RADIAN * deg; }

/*
  Since version 3.2, the behavior of Eigen's eulerAngles function was slightly modified;
  The returned angles are in the ranges [0:pi]x[0:pi]x[-pi:pi].
  This is not good for using the returned angles to interpolate attitdues.
  Now our own implementation is used for getting R-P-Y angles.
*/
/*
  template<typename Derived>
  inline Eigen::Matrix<typename Eigen::MatrixBase<Derived>::Scalar, 3, 1>
  rpyFromRot(const Eigen::MatrixBase<Derived>& R) {
  Vector3 ea = R.eulerAngles(2, 1, 0);
  return Vector3(ea[2], ea[1], ea[0]); // exchange element order to be our conventional one !
  }
*/

 Vector3d rpyFromRot(const Matrix3d& R);

 Matrix3d rotFromRpy(double r, double p, double y);
 Matrix3d rotFromXyzEuler(double r, double p, double y);
 Vector3d xyzEulerFromRot(const Matrix3d& R);

Eigen::Matrix<double, 6, 6> GeometricJ2AnalyticalJ(const Matrix3d& R);


inline Matrix3d rotFromRpy(const Vector3d& rpy) {
    return rotFromRpy(rpy[0], rpy[1], rpy[2]);
}



 Vector3d omegaFromRot(const Matrix3d& R);

inline Matrix3d hat(const Vector3d& x) {
    Matrix3d M;
    M <<  0.0, -x(2),   x(1),
        x(2),   0.0,  -x(0),
        -x(1),  x(0),   0.0;
    return M;
}
std::string str(const Vector3d& v);
std::string str(const Vector3f& v);
bool toVector3(const std::string& s, Vector3d& out_v);
bool toVector3(const std::string& s, Vector3f& out_v);


void normalizeRotation(Matrix3d& R);


}

#endif
