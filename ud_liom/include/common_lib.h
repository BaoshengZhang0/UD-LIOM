#ifndef COMMON_LIB_H
#define COMMON_LIB_H
#include <math.h>
#include <time.h>
#include <numeric>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
using namespace Eigen;

#define PI_M (3.14159265358)
#define G_m_s2 (9.8015)         // Gravaty const in Beijing/China
#define LIDAR_SP_LEN    (2)
#define NUM_MATCH_POINTS    (5)

#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a,b)  Matrix<double, (a), (b)>
#define VD(a)    Matrix<double, (a), 1>
#define MF(a,b)  Matrix<float, (a), (b)>
#define VF(a)    Matrix<float, (a), 1>

extern M3D Eye3d;
extern M3F Eye3f;
extern V3D Zero3d;
extern V3F Zero3f;
struct UTM{
    double time;
    double E;
    double N;
    double U;
};

/**
 * Store aligned LiDAR and IMU data
 */
struct MeasureGroup{
    MeasureGroup(){
        lidar_beg_time = 0.0;
        this->lidar_surf.reset(new PointCloudXYZI());
        this->lidar_edge.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZI::Ptr lidar_surf;
    PointCloudXYZI::Ptr lidar_edge;
    deque<sensor_msgs::Imu::ConstPtr> imu;
    V3D gnss_pose_cur;
    V3D gnss_pose_last;
    M3D imu_rot_cur;
    M3D imu_rot_last;
};

struct UnDistState{
    double time;
    V3D trans;
    M3D rot;
};
template<typename T>
T rad2deg(T radians){
  return radians * 180.0 / PI_M;
}

template<typename T>
T deg2rad(T degrees){
  return degrees * PI_M / 180.0;
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template<typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num){
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++){
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);
    
    for (int j = 0; j < point_num; j++){
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold){
            return false;
        }
    }
    normvec.normalize();
    return true;
}

inline float calc_dist(PointType p1, PointType p2){
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}
template<typename T>
inline T calc_covar(const vector<T> vs){
    T meanval(accumulate(vs.begin(), vs.end(), 0.0) / vs.size());
    T conval = 0.0;
    for(auto v : vs){
        conval += pow(v - meanval, 2);
    }
    conval /= vs.size();
    return conval;
}

template<typename T>
inline bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point_near, const T &threshold){
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;
    for (int j = 0; j < NUM_MATCH_POINTS; j++){
        A(j,0) = point_near[j].x;
        A(j,1) = point_near[j].y;
        A(j,2) = point_near[j].z;
    }
//    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
    Matrix<T, 3, 1> normvec = A.householderQr().solve(b);
    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;
    for (int j = 0; j < NUM_MATCH_POINTS; j++){
        if (fabs(pca_result(0) * point_near[j].x + pca_result(1) * point_near[j].y + pca_result(2) * point_near[j].z + pca_result(3)) > threshold){
            return false;
        }
    }
    return true;
}

template<typename T>
inline bool esti_line(Matrix<T, 3, 1> &point_a, Matrix<T, 3, 1> &point_b, const PointType &point_world, PointVector &point_near, T &pd2){
    vector<T> xs, ys;
    for(auto p : point_near){
        xs.push_back(p.x);
        ys.push_back(p.y);
    }
//    if(calc_covar(xs) > 0.1 || calc_covar(ys) > 0.1) return false;
    Matrix<T, 3, 1> center;
    center.setZero();
    for(auto p : point_near){
        center.x() += p.x;
        center.y() += p.y;
        center.z() += p.z;
    }
    center /= point_near.size();
    Matrix<T, 3, 3> covMat;
    covMat.setZero();
    for(auto p : point_near){
        Eigen::Matrix<T, 3, 1> tmpZeroMean;
        tmpZeroMean.x() = p.x - center.x();
        tmpZeroMean.y() = p.y - center.y();
        tmpZeroMean.z() = p.z - center.z();
        covMat += tmpZeroMean * tmpZeroMean.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
    Matrix<T, 3, 1> unit_direction = saes.eigenvectors().col(2);
    if (saes.eigenvalues()[2] > 4 * saes.eigenvalues()[1]){
        Matrix<T, 3, 1> point_on_line = center;
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;
        Matrix<T, 3, 1> point_o;
        point_o.x() = point_world.x;
        point_o.y() = point_world.y;
        point_o.z() = point_world.z;

        Matrix<T, 3, 1> oa = point_o - point_a;
        Matrix<T, 3, 1> ob = point_o - point_b;
        Matrix<T, 3, 1> ab = point_a - point_b;
        pd2 = (oa.cross(ob)).norm() / ab.norm();
        return true;
    }
    return false;
}

template<typename T>
inline bool esti_line(Matrix<T, 4, 1> &lca_result, const PointType &point_sel, PointVector &point_near, const T &threshold){
//    double rale = atan2(point_near.front().y - point_near.back().y, point_near.front().x - point_near.back().x);
//    rale = atan2(point_near.front().z - point_near.back().z, point_near.front().x - point_near.back().x);
//    if(fabs(rale) < fabs(M_PI/2.2)) return false;
    vector<T> xs, ys;
    for(auto p : point_near){
        xs.push_back(p.x);
        ys.push_back(p.y);
    }
    if(calc_covar(xs) > 0.1 || calc_covar(ys) > 0.1) return false;
    Matrix<T, 3, 1> center;
    center.setZero();
    for(auto p : point_near){
        center.x() += p.x;
        center.y() += p.y;
        center.z() += p.z;
    }
    center /= point_near.size();
    Matrix<T, 3, 3> covMat;
    covMat.setZero();
    for(auto p : point_near){
        Eigen::Matrix<T, 3, 1> tmpZeroMean;
        tmpZeroMean.x() = p.x - center.x();
        tmpZeroMean.y() = p.y - center.y();
        tmpZeroMean.z() = p.z - center.z();
        covMat += tmpZeroMean * tmpZeroMean.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
    Matrix<T, 3, 1> unit_direction = saes.eigenvectors().col(2);
    if (saes.eigenvalues()[2] > threshold * saes.eigenvalues()[1]){
        Matrix<T, 3, 1> point_on_line = center;
        Matrix<T, 3, 1> point_a, point_b;
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;

        T x0 = point_sel.x;
        T y0 = point_sel.y;
        T z0 = point_sel.z;
        T x1 = point_a[0];
        T y1 = point_a[1];
        T z1 = point_a[2];
        T x2 = point_b[0];
        T y2 = point_b[1];
        T z2 = point_b[2];
      float m11 = ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1));
      float m22 = ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1));
      float m33 = ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1));
      float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);
      float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                       (z1 - z2) * (z1 - z2));
      float la = ((y1 - y2) * m11 + (z1 - z2) * m22) / a012 / l12;
      float lb = -((x1 - x2) * m11 - (z1 - z2) * m33) / a012 / l12;
      float lc = -((x1 - x2) * m22 + (y1 - y2) * m33) / a012 / l12;
      T ld2 = a012 / l12;
      T s = 1.8 * fabs(ld2);
      if (s < 0.9 && ld2 != 0) {
          lca_result[0] = s * la;
          lca_result[1] = s * lb;
          lca_result[2] = s * lc;
          lca_result[3] = s * ld2;
      }
      return true;
    }
    return false;
}

template<typename T>
inline bool esti_line(const V3D &unit_direction, Matrix<T, 4, 1> &lca_result, const PointType &point_sel, PointVector &point_near){
    Matrix<T, 3, 1> center;
    center.setZero();//置0
    for(auto p : point_near){
        center.x() += p.x;
        center.y() += p.y;
        center.z() += p.z;
    }
    center /= point_near.size();

    if (true){
        Matrix<T, 3, 1> point_on_line = center;
        Matrix<T, 3, 1> point_a, point_b;
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;
//        double ss = 0.0;
//        for(auto p : point_near)
//        {
//            V3D pa(V3D{p.x - point_a.x(), p.y - point_a.y(), p.z - point_a.z()});
//            V3D pb(V3D{p.x - point_b.x(), p.y - point_b.y(), p.z - point_b.z()});
//            ss += abs(pa.transpose() * pb)/(pa.norm() * pb.norm());
//        }
//        if(ss > 4.0) return false;
        T x0 = point_sel.x;
        T y0 = point_sel.y;
        T z0 = point_sel.z;
        T x1 = point_a[0];
        T y1 = point_a[1];
        T z1 = point_a[2];
        T x2 = point_b[0];
        T y2 = point_b[1];
        T z2 = point_b[2];

        T a1 = x0 - x1, b1 = y0 - y1, c1 = z0 - z1;
        T a2 = x0 - x2, b2 = y0 - y2, c2 = z0 - z2;

        T m11 = b1*c2 - b2*c1, m22 = c1*a2 - a1*c2, m33 = a1*b2 - b1*a2;
        T a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

        T l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                         (z1 - z2) * (z1 - z2));

        T la = ((y1 - y2) * m33 - (z1 - z2) * m22) / a012 / l12;
        T lb = ((z1 - z2) * m11 - (x1 - x2) * m33) / a012 / l12;
        T lc = ((x1 - x2) * m22 - (y1 - y2) * m11) / a012 / l12;

        T ld2 = a012 / l12;
        T s = 1.8 * fabs(ld2);
        if (s < 0.9 && ld2 != 0) {
            lca_result[0] = s * la;
            lca_result[1] = s * lb;
            lca_result[2] = s * lc;
            lca_result[3] = s * ld2;
        }
        return true;
    }
    return false;
}

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
//    Eigen::Matrix<T, 3, 1> eulerZYX = Rzyx.eulerAngles(2, 1, 0);
//    swap(eulerZYX.x(), eulerZYX.z());
//    return eulerZYX;
}

template<typename T>
Matrix<T, 3, 3> EulerToRzyx(const Matrix<T, 3, 1> &Euler){
    T srx = sin(Euler[0]);
    T crx = cos(Euler[0]);
    T sry = sin(Euler[1]);
    T cry = cos(Euler[1]);
    T srz = sin(Euler[2]);
    T crz = cos(Euler[2]);
//    Matrix<T, 3, 3> Rx, Ry, Rz, Rzyx;
//    Rx << 1, 0, 0, 0, crx, -srx, 0, srx, crx;
//    Ry << cry, 0, sry, 0, 1, 0, -sry, 0, cry;
//    Rz << crz, -srz, 0, srz, crz, 0, 0, 0, 1;
//    Rzyx << Rz * Ry * Rx;
    Matrix<T, 3, 3> Rzyx;
    Rzyx << cry * crz, srx * sry * crz - crx * srz, srx * srz + crx * sry * crz,
            cry * srz, crx * crz + srx * sry * srz, crx * sry * srz - srx * crz,
            -sry     , srx * cry                  , crx * cry;
    return Rzyx;

//    Eigen::Matrix<T, 3, 3> rotationMatrix =
//            (Eigen::AngleAxis<T>(Euler(2), Eigen::Vector3d::UnitZ()) *
//             Eigen::AngleAxis<T>(Euler(1), Eigen::Vector3d::UnitY()) *
//             Eigen::AngleAxis<T>(Euler(0), Eigen::Vector3d::UnitX())).toRotationMatrix();
//
//    return rotationMatrix;
}

template<typename T>
Matrix<T, 3, 3> EulerToRxyz(const Matrix<T, 3, 1> &Euler){
    T srx = sin(Euler[0]);
    T crx = cos(Euler[0]);
    T sry = sin(Euler[1]);
    T cry = cos(Euler[1]);
    T srz = sin(Euler[2]);
    T crz = cos(Euler[2]);
    Matrix<T, 3, 3> Rx, Ry, Rz, Rxyz;
    Rx << 1, 0, 0, 0, crx, -srx, 0, srx, crx;
    Ry << cry, 0, sry, 0, 1, 0, -sry, 0, cry;
    Rz << crz, -srz, 0, srz, crz, 0, 0, 0, 1;
    Rxyz << Rx * Ry * Rz;
    return Rxyz;
}

template<typename T>
Matrix<T, 3, 1> QuaternionToEuler(const Eigen::Quaternion<T> &Qua_){
    Eigen::Quaternion<T> Qua = Qua_;
    Qua.normalize();
    T sqw = Qua.w() * Qua.w();
    T sqx = Qua.x() * Qua.x();
    T sqy = Qua.y() * Qua.y();
    T sqz = Qua.z() * Qua.z();
    T unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
    T test = Qua.w()*Qua.y() - Qua.z()*Qua.x();

    Matrix<T, 3, 1> Euler;

    if (test > 0.49999*unit) { // singularity at north pole
        Euler << 2 * std::atan2(Qua.x(), Qua.w()), M_PI/2, 0;
        return Euler;
    }
    if (test < -0.49999*unit) { // singularity at south pole
        Euler << -2 * std::atan2(Qua.x(), Qua.w()), -M_PI/2, 0;
        return Euler;
    }

    Euler <<
          std::atan2(2*Qua.x()*Qua.w()+2*Qua.y()*Qua.z() , -sqx - sqy + sqz + sqw),
            std::asin (2*test/unit),
            std::atan2(2*Qua.z()*Qua.w()+2*Qua.y()*Qua.x() , sqx - sqy - sqz + sqw);
    return Euler;
}

template<typename T>
Eigen::Quaterniond EulerToQuaternion(const Matrix<T, 3, 1> &Euler){
    T srx = sin(Euler[0]);
    T crx = cos(Euler[0]);
    T sry = sin(Euler[1]);
    T cry = cos(Euler[1]);
    T srz = sin(Euler[2]);
    T crz = cos(Euler[2]);
//    Matrix<T, 3, 3> Rx, Ry, Rz, Rzyx;
//    Rx << 1, 0, 0, 0, crx, -srx, 0, srx, crx;
//    Ry << cry, 0, sry, 0, 1, 0, -sry, 0, cry;
//    Rz << crz, -srz, 0, srz, crz, 0, 0, 0, 1;
//    Rzyx << Rz * Ry * Rx;
    Matrix<T, 3, 3> Rzyx;
    Rzyx << cry * crz, srx * sry * crz - crx * srz, srx * srz + crx * sry * crz,
            cry * srz, crx * crz + srx * sry * srz, crx * sry * srz - srx * crz,
            -sry     , srx * cry                  , crx * cry;

    Eigen::Quaternion<T> quaternion(Rzyx);
    quaternion.normalize();
    return quaternion;
}

template<typename T>
Matrix<T, 3, 3> QuaternionToRzyx(const Eigen::Quaternion<T> &Qua){
//    Matrix<T, 3, 3> Rzyx;
//    T r00 = 2 * (Qua.w() * Qua.w() + Qua.x() * Qua.x()) - 1;
//    T r01 = 2 * (Qua.x() * Qua.y() - Qua.w() * Qua.z());
//    T r02 = 2 * (Qua.x() * Qua.z() + Qua.w() * Qua.y());
//
//    T r10 = 2 * (Qua.x() * Qua.y() + Qua.w() * Qua.z());
//    T r11 = 2 * (Qua.w() * Qua.w() + Qua.y() * Qua.y()) - 1;
//    T r12 = 2 * (Qua.y() * Qua.z() - Qua.w() * Qua.x());
//
//    T r20 = 2 * (Qua.x() * Qua.z() - Qua.w() * Qua.y());
//    T r21 = 2 * (Qua.y() * Qua.z() + Qua.w() * Qua.x());
//    T r22 = 2 * (Qua.w() * Qua.w() + Qua.z() * Qua.z()) - 1;
//
//    Rzyx << r00, r01, r02, r10, r11, r12, r20, r21, r22;

//    Matrix<T, 3, 3> Rzyx;
//    Rzyx = Qua.matrix();
    Matrix<T, 3, 3> Rzyx;
    Rzyx = Qua.toRotationMatrix();
    return Rzyx;
}

template<typename T>
Eigen::Quaterniond RzyxToQuaternion(const Matrix<T, 3, 3> &Rzyx){
    Eigen::Quaternion<T> quaternion(Rzyx);
    quaternion.normalize();
    return quaternion;
}

template<typename T>
Matrix<T, 3, 1> EulerG2L(const Matrix<T, 3, 1> &Euler){
    if(Euler.z() > 180.0){
        Euler.z() = 360.0 - Euler.z();
    }else{
        Euler.z() = -Euler.z();
    }
    return Euler;
}

#endif
