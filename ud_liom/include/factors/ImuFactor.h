#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <assert.h>
#include <cmath>
#include "utils/math_tools.h"
#include "factors/Preintegration.h"
struct ImuAutoFactor{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuAutoFactor(const Eigen::Quaterniond Qi, const Eigen::Vector3d Pi, double *SpeedBiasLast, Preintegration *pre_integration):
            Qi_(Qi), Pi_(Pi), pre_integration_(pre_integration){
        for(int i = 0; i < 9; ++i){
            SpeedBiasi_[i] = SpeedBiasLast[i];
        }
        g_vec_ = pre_integration_->g_vec_;
        Qi_.normalize();
    }

    template <typename T> bool operator()(const T *qj, const T *pj, const T *spbj, T *residuals) const {
//        Eigen::Quaternion<double> Qi((T)Qi_.vec()[0], (T)Qi_.vec()[1], (T)Qi_.vec()[2], (T)Qi_.vec()[3]);
//        Eigen::Matrix<double, 3, 1> Pi = Eigen::Matrix<T, 3, 1>{(T)Pi_[0], (T)Pi_[1], (T)Pi_[2]};
        Eigen::Matrix<double, 3, 1> Vi{SpeedBiasi_[0], SpeedBiasi_[1], SpeedBiasi_[2]};
        Eigen::Matrix<double, 3, 1> Bai{SpeedBiasi_[3], SpeedBiasi_[4], SpeedBiasi_[5]};
        Eigen::Matrix<double, 3, 1> Bgi{SpeedBiasi_[6], SpeedBiasi_[7], SpeedBiasi_[8]};

        Eigen::Quaternion<double> Qj(((ceres::Jet<double, 16>)qj[0]).a,
                                     ((ceres::Jet<double, 16>)qj[1]).a,
                                     ((ceres::Jet<double, 16>)qj[2]).a,
                                     ((ceres::Jet<double, 16>)qj[3]).a);
        Qj.normalize();
        Eigen::Matrix<double, 3, 1> Pj(((ceres::Jet<double, 16>)pj[0]).a,
                                       ((ceres::Jet<double, 16>)pj[1]).a,
                                       ((ceres::Jet<double, 16>)pj[2]).a);

        Eigen::Matrix<double, 3, 1> Vj(((ceres::Jet<double, 16>)spbj[0]).a,
                                       ((ceres::Jet<double, 16>)spbj[1]).a,
                                       ((ceres::Jet<double, 16>)spbj[2]).a);

        Eigen::Matrix<double, 3, 1> Baj(((ceres::Jet<double, 16>)spbj[3]).a,
                                        ((ceres::Jet<double, 16>)spbj[4]).a,
                                        ((ceres::Jet<double, 16>)spbj[5]).a);

        Eigen::Matrix<double, 3, 1> Bgj(((ceres::Jet<double, 16>)spbj[6]).a,
                                        ((ceres::Jet<double, 16>)spbj[7]).a,
                                        ((ceres::Jet<double, 16>)spbj[8]).a);

        Eigen::Matrix<double, 15, 1> residual;
        residual = pre_integration_->evaluate(Pi_, Qi_, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);
//        residual = pre_integration_->evaluate(Pi_, Qi_, Vi, Pj, Qj, Vj);
//        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration_->covariance_.inverse()).matrixL().transpose();
//        residual = residual * 0.000001;
        for(int i = 0; i < residual.size(); ++i){
            residuals[i] = (T)residual[i];
        }
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Quaterniond Qi, const Eigen::Vector3d Pi, double *SpeedBiasLast, Preintegration *pre_integration){
        return (new ceres::AutoDiffCostFunction<ImuAutoFactor, 15, 4, 3, 9>(new ImuAutoFactor(Qi, Pi, SpeedBiasLast, pre_integration)));
    }

    Eigen::Quaterniond Qi_;
    Eigen::Vector3d Pi_, Pimu_;
    Eigen::Matrix<double, 9, 1> SpeedBiasi_;
    Preintegration *pre_integration_;
    Eigen::Vector3d g_vec_;
};

struct ImuAutoFactorIncr{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuAutoFactorIncr(const Eigen::Quaterniond Q_last, const Eigen::Vector3d P_last,
                      const Eigen::Quaterniond Q_diff, const Eigen::Vector3d P_diff):
                      Q_last_(Q_last), P_last_(P_last), Q_diff_(Q_diff), P_diff_(P_diff){
    }
    template <typename T> bool operator()(const T *qj, const T *pj, T *residuals) const {
        Eigen::Quaternion<double> Qj(((ceres::Jet<double, 7>)qj[0]).a,
                                     ((ceres::Jet<double, 7>)qj[1]).a,
                                     ((ceres::Jet<double, 7>)qj[2]).a,
                                     ((ceres::Jet<double, 7>)qj[3]).a);
        Qj.normalize();
        Eigen::Matrix<double, 3, 1> Pj(((ceres::Jet<double, 7>)pj[0]).a,
                                       ((ceres::Jet<double, 7>)pj[1]).a,
                                       ((ceres::Jet<double, 7>)pj[2]).a);
        Eigen::Matrix<double, 6, 1> residual;
        residual.block<3, 1>(O_P, 0) = 1.0 * (Pj - P_diff_ - P_last_);
        residual.block<3, 1>(O_R, 0) = 1.0 * (Q_diff_.inverse() * (Q_last_.inverse() * Qj)).normalized().vec();
        for(int i = 0; i < residual.size(); ++i){
            residuals[i] = (T)residual[i];
        }
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Quaterniond Q_last, const Eigen::Vector3d P_last,
                                       const Eigen::Quaterniond Q_diff, const Eigen::Vector3d P_diff){
        return (new ceres::AutoDiffCostFunction<ImuAutoFactorIncr, 6, 4, 3>(new ImuAutoFactorIncr(Q_last, P_last, Q_diff, P_diff)));
    }
    Eigen::Quaterniond Q_last_, Q_diff_;
    Eigen::Vector3d P_last_, P_diff_;
};

struct ImuAutoFactorAbs{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuAutoFactorAbs(const Eigen::Quaterniond Qi):
            Qi_(Qi){
    }

    template <typename T> bool operator()(const T *qj, T *residuals) const {

        Eigen::Quaternion<double> Qj(((ceres::Jet<double, 4>)qj[0]).a,
                                     ((ceres::Jet<double, 4>)qj[1]).a,
                                     ((ceres::Jet<double, 4>)qj[2]).a,
                                     ((ceres::Jet<double, 4>)qj[3]).a);
        Qj.normalize();
        Eigen::Matrix<double, 3, 1> residual;
        residual.block<3, 1>(0, 0) = 2.0 * (Qi_.inverse() * Qj).normalized().vec();
        for(int i = 0; i < residual.size(); ++i){
            residuals[i] = (T)residual[i];
        }
        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Quaterniond Qi){
        return (new ceres::AutoDiffCostFunction<ImuAutoFactorAbs, 3, 4>(new ImuAutoFactorAbs(Qi)));
    }

    Eigen::Quaterniond Qi_;
};
