
#include "Util.h"
#include <boost/format.hpp>


using namespace boost;
// using namespace Eigen;

namespace ljnoid {

Matrix3d rotFromRpy(double r, double p, double y)  //R_xyz(r,p,y) fixed frame 
{
    const double cr = cos(r);
    const double sr = sin(r);
    const double cp = cos(p);
    const double sp = sin(p);
    const double cy = cos(y);
    const double sy = sin(y);

    Matrix3d R;
    R << cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy,
        cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy,
        -sp  , sr*cp           , cr*cp;

    return R;
}

Matrix3d rotFromXyzEuler(double r, double p, double y)
{
    const double cr = cos(r);
    const double sr = sin(r);
    const double cp = cos(p);
    const double sp = sin(p);
    const double cy = cos(y);
    const double sy = sin(y);

    Matrix3d R;
    R << cp*cy,               -cp*sy,                  sp,
        sr*sp*cy + cr*sy,   -sr*sp*sy + cr*cy,       -sr*cp,
        -cr*sp*cy + sr*sy,  cr*sp*sy + sr*cy,        cr*cp;

    return R;
}

Vector3d xyzEulerFromRot(const Matrix3d& R)
{
    double roll, pitch, yaw;

    
    if(R(0,2) > 0.998)
    {
        pitch=90*PI/180;
        roll=0.0;
        yaw=atan2(R(2,1),-R(2,0));

    }
    else if(R(0,2) < -0.998)
    {
        pitch=-90*PI/180;
        roll=0.0;
        yaw=atan2(-R(2,1),R(2,0));

    }
    else 
    {
        pitch=atan2(R(0,2),sqrt(R(0,0)*R(0,0)+R(0,1)*R(0,1)));
        roll=atan2(-R(1,2)/cos(pitch),R(2,2)/cos(pitch));
        yaw=atan2(-R(0,1)/cos(pitch),R(0,0)/cos(pitch));
    }
    
    
    
    return Vector3d(roll, pitch, yaw);
}

Eigen::Matrix<double, 6, 6> GeometricJ2AnalyticalJ(const Matrix3d& R)
{
    Vector3d euler;
    euler=xyzEulerFromRot(R);

    Matrix3d Balpha;
    Balpha<< 1.0,0.0,sin(euler.y()),   //xyz euler
            0.0,cos(euler.x()),-cos(euler.y())*sin(euler.x()),
            0.0,sin(euler.x()),cos(euler.y())*cos(euler.x());

    Eigen::Matrix<double, 6, 6> B=Eigen::Matrix<double, 6, 6>::Identity();
    B.block<3,3>(3,3)=Balpha.inverse();


    Eigen::Matrix<double, 6, 6> B1;         //from [Jv;Jw] to [Jw;Jv]
    B1.block<3,3>(0,0)=B.block<3,3>(3,3);
    B1.block<3,3>(0,3)=B.block<3,3>(3,0);
    B1.block<3,3>(3,0)=B.block<3,3>(0,3);
    B1.block<3,3>(3,3)=B.block<3,3>(0,0);

    return B1;

}
   
Vector3d rpyFromRot(const Matrix3d& R)
{
    double roll, pitch, yaw;
    
    if((fabs(R(0,0)) < fabs(R(2,0))) && (fabs(R(1,0)) < fabs(R(2,0)))) {
        // cos(p) is nearly = 0
        double sp = -R(2,0);
        if (sp < -1.0) {
            sp = -1.0;
        } else if (sp > 1.0) {
            sp = 1.0;
        }
        pitch = asin(sp); // -pi/2< p < pi/2
            
        roll = atan2(sp * R(0,1) + R(1,2),  // -cp*cp*sr*cy
                     sp * R(0,2) - R(1,1)); // -cp*cp*cr*cy
            
        if (R(0,0) > 0.0) { // cy > 0
            (roll < 0.0) ? (roll += PI) : (roll -= PI);
        }
        const double sr = sin(roll);
        const double cr = cos(roll);
        if(sp > 0.0){
            yaw = atan2(sr * R(1,1) + cr * R(1,2), //sy*sp
                        sr * R(0,1) + cr * R(0,2));//cy*sp
        } else {
            yaw = atan2(-sr * R(1,1) - cr * R(1,2),
                        -sr * R(0,1) - cr * R(0,2));
        }
    } else {
        yaw = atan2(R(1,0), R(0,0));
        const double sa = sin(yaw);
        const double ca = cos(yaw);
        pitch = atan2(-R(2,0), ca * R(0,0) + sa * R(1,0));
        roll = atan2(sa * R(0,2) - ca * R(1,2), -sa * R(0,1) + ca * R(1,1));
    }
    return Vector3d(roll, pitch, yaw);
}


Vector3d omegaFromRot(const Matrix3d& R)
{
    double alpha = (R(0,0) + R(1,1) + R(2,2) - 1.0) / 2.0;

    if(fabs(alpha - 1.0) < 1.0e-6) {   //th=0,2PI;
        return Vector3d::Zero();

    } else {
        double th = acos(alpha);
        double s = sin(th);

        if (s < std::numeric_limits<double>::epsilon()) {   //th=PI
            return Vector3d( sqrt((R(0,0)+1)*0.5)*th, sqrt((R(1,1)+1)*0.5)*th, sqrt((R(2,2)+1)*0.5)*th );
        }

        double k = -0.5 * th / s;

        return Vector3d((R(1,2) - R(2,1)) * k,
                       (R(2,0) - R(0,2)) * k,
                       (R(0,1) - R(1,0)) * k);
    }
}

std::string str(const Vector3d& v)
{
    return str(format("%1%  %2%  %3%") % v[0] % v[1] % v[2]);
}


std::string str(const Vector3f& v)
{
    return str(format("%1%  %2%  %3%") % v[0] % v[1] % v[2]);
}


template<class VectorType>
static bool toVector3_(const std::string& s, VectorType& out_v)
{
    const char* nptr = s.c_str();
    char* endptr;
    for(int i=0; i < 3; ++i){
        out_v[i] = strtod(nptr, &endptr);
        if(endptr == nptr){
            return false;
        }
        nptr = endptr;
        while(isspace(*nptr)){
            nptr++;
        }
        if(*nptr == ','){
            nptr++;
        }
    }
    return true;
}    


bool toVector3(const std::string& s, Vector3d& out_v)
{
    return toVector3_(s, out_v);
}


bool toVector3(const std::string& s, Vector3f& out_v)
{
    return toVector3_(s, out_v);
}


void normalizeRotation(Matrix3d& R)
{
    Matrix3d::ColXpr x = R.col(0);
    Matrix3d::ColXpr y = R.col(1);
    Matrix3d::ColXpr z = R.col(2);
    x.normalize();
    z = x.cross(y).normalized();
    y = z.cross(x);
}



}

