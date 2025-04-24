#ifndef LIDARFACTOR_H
#define LIDARFACTOR_H

#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <assert.h>
#include <cmath>
#include "utils/math_tools.h"

struct LidarEdgeFactor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LidarEdgeFactor(Eigen::Vector3d curr_point_,
                    Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_,
                    Eigen::Quaterniond qlb_,
                    Eigen::Vector3d tlb_,
                    double s_): curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_) {
        qlb = qlb_;
        tlb = tlb_;
        s = s_;
    }

    template <typename T> bool operator()(const T *t, const T *q, T *residual) const {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
        Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

        Eigen::Quaternion<T> q_last_curr{q[0], q[1], q[2], q[3]};

        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        Eigen::Quaternion<T> q_l_b{T(qlb.w()), T(qlb.x()), T(qlb.y()), T(qlb.z())};
        Eigen::Matrix<T, 3, 1> t_l_b{T(tlb.x()), T(tlb.y()), T(tlb.z())};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;

        Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
        Eigen::Matrix<T, 3, 1> de = lpa - lpb;

        residual[0] = nu.norm() / de.norm();
        residual[0] *= T(s);

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_,
                                       Eigen::Quaterniond qlb_,
                                       Eigen::Vector3d tlb_,
                                       double s_) {
        return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 1, 3, 4>(new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, qlb_, tlb_, s_)));
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    Eigen::Quaterniond qlb;
    Eigen::Vector3d tlb;
    double s;
};

struct LidarLineFactor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LidarLineFactor(Eigen::Vector3d curr_point_,
                    Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_,
                    double s_): curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_) {
        s = s_;
    }

    template <typename T> bool operator()(const T *q, const T *t, T *residual) const {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
        Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

        Eigen::Quaternion<T> q_last_curr{q[0], q[1], q[2], q[3]};

        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;

        Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
        Eigen::Matrix<T, 3, 1> de = lpa - lpb;

        residual[0] = nu.norm() / de.norm();
//        residual[0] *= T(s);

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_,
                                       double s_) {
        return (new ceres::AutoDiffCostFunction<LidarLineFactor, 1, 4, 3>(new LidarLineFactor(curr_point_, last_point_a_, last_point_b_, s_)));
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double s;
};

struct LidarPlaneNormFactor {
    LidarPlaneNormFactor(Eigen::Vector3d curr_point_,
                         Eigen::Vector3d plane_unit_norm_,
                         Eigen::Quaterniond qlb_,
                         Eigen::Vector3d tlb_,
                         double negative_OA_dot_norm_,
                         double score_): curr_point(curr_point_),
        plane_unit_norm(plane_unit_norm_),
        qlb(qlb_),
        tlb(tlb_),
        negative_OA_dot_norm(negative_OA_dot_norm_),
        score(score_) {}

    template <typename T> bool operator()(const T *t, const T *q, T *residual) const {
        Eigen::Quaternion<T> q_w_curr{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        Eigen::Quaternion<T> q_l_b{T(qlb.w()), T(qlb.x()), T(qlb.y()), T(qlb.z())};
        Eigen::Matrix<T, 3, 1> t_l_b{T(tlb.x()), T(tlb.y()), T(tlb.z())};

        point_w = q_l_b.inverse() * (cp - t_l_b);
        point_w = q_w_curr * point_w + t_w_curr;

        Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
        residual[0] = T(score) * (norm.dot(point_w) + T(negative_OA_dot_norm));
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d plane_unit_norm_,
                                       const Eigen::Quaterniond qlb_,
                                       const Eigen::Vector3d tlb_,
                                       const double negative_OA_dot_norm_,
                                       const double score_) {
        return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactor, 1, 3, 4>(new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, qlb_, tlb_, negative_OA_dot_norm_, score_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    Eigen::Quaterniond qlb;
    Eigen::Vector3d tlb;
    double negative_OA_dot_norm, score;
};

struct LidarPlaneNormIncreFactorWORot{
    LidarPlaneNormIncreFactorWORot(Eigen::Quaterniond Qi, Eigen::Vector3d curr_point_,
                              Eigen::Vector3d plane_unit_norm_,
                              double negative_OA_dot_norm_,
                              double s_): Qi_(Qi),
                                            curr_point(curr_point_),
                                          plane_unit_norm(plane_unit_norm_),
                                          negative_OA_dot_norm(negative_OA_dot_norm_),
                                          score(s_){}

    template <typename T> bool operator()(const T *t, T *residual) const {
        Eigen::Matrix<double, 3, 1> t_inc{((ceres::Jet<double, 3>)t[0]).a,
                                     ((ceres::Jet<double, 3>)t[1]).a,
                                     ((ceres::Jet<double, 3>)t[2]).a};
        Eigen::Matrix<double, 3, 1> cp{double(curr_point.x()), double(curr_point.y()), double(curr_point.z())};
        Eigen::Matrix<double, 3, 1> point_w;
        point_w = Qi_ * cp + t_inc;

        Eigen::Matrix<double, 3, 1> norm(double(plane_unit_norm.x()), double(plane_unit_norm.y()), double(plane_unit_norm.z()));
        residual[0] = (norm.dot(point_w) + T(negative_OA_dot_norm)) ;// (score * score + 1.0);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Quaterniond Qi,
                                       const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d plane_unit_norm_,
                                       const double negative_OA_dot_norm_,
                                       const double s_) {
        return (new ceres::AutoDiffCostFunction<LidarPlaneNormIncreFactorWORot, 1, 3>(new LidarPlaneNormIncreFactorWORot(Qi, curr_point_, plane_unit_norm_, negative_OA_dot_norm_, s_)));
    }

    Eigen::Quaterniond Qi_;
    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
    double score;
};

struct LidarPlaneNormIncreFactor {
    LidarPlaneNormIncreFactor(Eigen::Vector3d curr_point_,
                              Eigen::Vector3d plane_unit_norm_,
                              double negative_OA_dot_norm_,
                              double s_): curr_point(curr_point_),
        plane_unit_norm(plane_unit_norm_),
        negative_OA_dot_norm(negative_OA_dot_norm_),
        score(s_){}

    template <typename T> bool operator()(const T *q, const T *t, T *residual) const {
        Eigen::Quaternion<T> q_inc{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 1> t_inc{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = q_inc * cp + t_inc;

        Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
        residual[0] = (norm.dot(point_w) + T(negative_OA_dot_norm)) / (score * score + 1.0);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                       const Eigen::Vector3d plane_unit_norm_,
                                       const double negative_OA_dot_norm_,
                                       const double s_) {
        return (new ceres::AutoDiffCostFunction<LidarPlaneNormIncreFactor, 1, 4, 3>(new LidarPlaneNormIncreFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_, s_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
    double score;
};

template <typename T>
class LidarPlaneNormFactorHandDiff : public ceres::SizedCostFunction<1, 3, 3> {
public:

    LidarPlaneNormFactorHandDiff(Eigen::Matrix<T, 3, 1> pointOri_,
                                 Eigen::Matrix<T, 4, 1> pabcd_,
                                 T s_):
            pointOri(pointOri_),
            pabcd(pabcd_),
            s_(s_){}
    virtual ~LidarPlaneNormFactorHandDiff() {}

    virtual bool Evaluate(T const* const* parameters,
                          T* residuals,
                          T** jacobians) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> transformR(parameters[0]);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> transformT(parameters[1]);

        T srx = sin(transformR[0]);
        T crx = cos(transformR[0]);
        T sry = sin(transformR[1]);
        T cry = cos(transformR[1]);
        T srz = sin(transformR[2]);
        T crz = cos(transformR[2]);
//        T tx = transformT[0];
//        T ty = transformT[1];
//        T tz = transformT[2];
        Eigen::Matrix<T, 3, 3> Rzxy, dR_dx, dR_dy, dR_dz;
        Rzxy << cry * crz, srx * sry * crz - crx * srz, srx * srz + crx * sry * crz,
                cry * srz, crx * crz + srx * sry * srz, crx * sry * srz - srx * crz,
                -sry     , srx * cry                  , crx * cry;

        dR_dx << 0, crx * sry * crz + srx * srz, crx * srz - srx * sry * crz,
                 0, crx * sry * srz -srx * crz , -srx * sry * srz - crx * crz,
                 0, crx * cry                  , -srx * cry;

        dR_dy << -sry * crz, srx * cry * crz, crx * cry * crz,
                 -sry * srz, srx * cry * srz, crx * cry * srz,
                 -cry      , -srx * sry     , -crx * sry;

        dR_dz << -cry * srz, -srx * sry * srz - crx * crz, srx * crz - crx * sry * srz,
                 cry * crz , -crx * srz + srx * sry * crz, crx * sry * crz + srx * srz,
                 0         , 0                           , 0;

        Eigen::Matrix<T, 3, 1> pointWor;
        pointWor = Rzxy * pointOri + transformT;

        residuals[0] = pabcd[0] * pointWor.x() + pabcd[1] * pointWor.y() + pabcd[2] * pointWor.z() + pabcd[3];

        Eigen::Matrix<T, 3, 1> coeff = Eigen::Matrix<T, 3, 1>{pabcd[0], pabcd[1], pabcd[2]} / (s_ * s_ + 1.0);
        if(jacobians != nullptr)
        {
            if(jacobians[0] != nullptr) {
                jacobians[0][0] = (dR_dx * pointOri).transpose() * coeff;
                jacobians[0][1] = (dR_dy * pointOri).transpose() * coeff;
                jacobians[0][2] = (dR_dz * pointOri).transpose() * coeff;
            }
            if(jacobians[1] != nullptr){
                jacobians[1][0] = coeff.transpose() * Eigen::Matrix<T, 3, 1>{1,0,0};
                jacobians[1][1] = coeff.transpose() * Eigen::Matrix<T, 3, 1>{0,1,0};
                jacobians[1][2] = coeff.transpose() * Eigen::Matrix<T, 3, 1>{0,0,1};
            }
        }
        return true;
    }
    Eigen::Matrix<T, 3, 1> pointOri;
    Eigen::Matrix<T, 4, 1> pabcd;
    T s_;
};
#endif // LIDARFACTOR_H
