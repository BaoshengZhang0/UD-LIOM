#ifndef _IMU_DIST_H
#define _IMU_DIST_H

#include "common_lib.h"
#include "factors/Preintegration.h"
/// *************IMU Process and undistortion

class IMUProcessing{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUProcessing();
    ~IMUProcessing();
    void setExtrinsic(const M3D &Lidar_R_wrt_IMU, const V3D &Lidar_T_wrt_IMU);
//    void setTransform(const M3D &transformCurR, const V3D &transformCurT);
    void setUndistFlag(const bool imu_rot_un, const bool imu_trans_un);
    void Process(const MeasureGroup &meas, PointCloudXYZI::Ptr surf_point, PointCloudXYZI::Ptr edge_point);
    void PreIntegration(const MeasureGroup &meas);
    void processIMU(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);

    bool flagImuInit;
    bool flagInsPoseInit;
    Eigen::Quaterniond orienta_initQ;
    bool imu_rot_un_, imu_trans_un_;
    int imu_count;
    int frame_count = 0;
    double last_timestamp_imu = -1.0;
    M3D imuR;
    Eigen::Quaterniond imuQ;
    V3D imuT;
    M3D lastR, curR;
    V3D lastT, curT;

    vector<Preintegration*> pre_integrations;
    vector<vector<double>> para_speed_bias;

    vector<Eigen::Vector3d> Ps;
    vector<Eigen::Vector3d> Vs;
    vector<Eigen::Matrix3d> Rs;
    vector<Eigen::Vector3d> Bas;
    vector<Eigen::Vector3d> Bgs;
    deque<sensor_msgs::Imu::Ptr> imu_buffer;
    vector<UnDistState> undist_IMU;

    V3D global_rot_init;
    M3D global_rot_cur;
    M3D global_rot_last;
private:
    void TransformToStartIMU(PointType *p, const M3D unR, const V3D unT);
    void IMU_Reset();
    void UndistortPcl(const MeasureGroup &meas, PointCloudXYZI &surf_point, PointCloudXYZI &edge_point);
    M3D IMU_R_wrt_Car_;
    V3D IMU_t_wrt_Car_;

    bool first_imu_frame_;
    int counterState;

    double cur_time_imu;
    Eigen::Vector3d acc_0, gyr_0, g;
};
#endif
