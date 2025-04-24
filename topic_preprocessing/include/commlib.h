#ifndef TOPIC_COMMLIB_H
#define TOPIC_COMMLIB_H

#include <iostream>
#include <ros/ros.h>
#include <mutex>
#include <condition_variable>
#include <pcl_ros/point_cloud.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
using namespace Eigen;
using namespace std;
#define G_m_s2 (9.8015)

struct PointXYZIRT{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

/**
* RoboSense point definition
*/
namespace RSPoint{
    struct EIGEN_ALIGN16 PointXYZIRT{
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring = 0;
        double timestamp = 0;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}POINT_CLOUD_REGISTER_POINT_STRUCT(RSPoint::PointXYZIRT,
        (float, x, x)(float, y, y)(float, z, z)(float , intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

/**
* Velodyne point definition
*/
namespace VELOPoint {
    struct EIGEN_ALIGN16 PointXYZIRT {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}POINT_CLOUD_REGISTER_POINT_STRUCT(VELOPoint::PointXYZIRT,
       (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp)
)

/**
 * Livox point definition
 */
namespace LIVOXPoint {
    struct EIGEN_ALIGN16 PointXYZIRT {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}POINT_CLOUD_REGISTER_POINT_STRUCT(LIVOXPoint::PointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp)
)

/**
 * Ouster point definition
 */
namespace OUSTPoint {
    struct EIGEN_ALIGN16 PointXYZIRT {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t  ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}POINT_CLOUD_REGISTER_POINT_STRUCT(OUSTPoint::PointXYZIRT,
       (float, x, x)(float, y, y)(float, z, z)
       (float, intensity, intensity)(uint32_t, t, t)
       (uint16_t, reflectivity, reflectivity)
       (uint8_t, ring, ring)
       (uint16_t, ambient, ambient)
       (uint32_t, range, range)
)
/// @brief LiDAR type selection.
enum LID_TYPE{RSM1 = 1, LIVOX, VELO, OUSTER, RSMECH};

template<typename T>
Matrix<T, 3, 1> RzyxToEuler(const Matrix<T, 3, 3> &Rzyx){
    Eigen::Quaterniond quaternion(Rzyx);
    quaternion.normalize();

    T sqw = quaternion.w() * quaternion.w();
    T sqx = quaternion.x() * quaternion.x();
    T sqy = quaternion.y() * quaternion.y();
    T sqz = quaternion.z() * quaternion.z();
    T unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
    T test = quaternion.w()*quaternion.y() - quaternion.z()*quaternion.x();

    Matrix<T, 3, 1> Euler;

    if (test > 0.49999*unit) { // singularity at north pole
        Euler << 2 * std::atan2(quaternion.x(), quaternion.w()), M_PI/2, 0;
        return Euler;
    }
    if (test < -0.49999*unit) { // singularity at south pole
        Euler << -2 * std::atan2(quaternion.x(), quaternion.w()), -M_PI/2, 0;
        return Euler;
    }

    Euler <<
          std::atan2(2*quaternion.x()*quaternion.w()+2*quaternion.y()*quaternion.z() , -sqx - sqy + sqz + sqw),
            std::asin (2*test/unit),
            std::atan2(2*quaternion.z()*quaternion.w()+2*quaternion.y()*quaternion.x() , sqx - sqy - sqz + sqw);
    return Euler;
}

template<typename T>
Matrix<T, 3, 3> EulerToRzyx(const Matrix<T, 3, 1> &Euler){
    T srx = sin(Euler[0]/180.0 * M_PI);
    T crx = cos(Euler[0]/180.0 * M_PI);
    T sry = sin(Euler[1]/180.0 * M_PI);
    T cry = cos(Euler[1]/180.0 * M_PI);
    T srz = sin(Euler[2]/180.0 * M_PI);
    T crz = cos(Euler[2]/180.0 * M_PI);
    Matrix<T, 3, 3> Rzyx;
    Rzyx << cry * crz, srx * sry * crz - crx * srz, srx * srz + crx * sry * crz,
            cry * srz, crx * crz + srx * sry * srz, crx * sry * srz - srx * crz,
            -sry     , srx * cry                  , crx * cry;

    return Rzyx;
}
#endif //TOPIC_COMMLIB_H
