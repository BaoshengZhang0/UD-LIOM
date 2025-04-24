#include "IMU_Processing.h"
IMUProcessing::IMUProcessing(){
    flagImuInit = false;
    imu_rot_un_ = true;
    imu_trans_un_ = true;
    imu_count = 0;
    cur_time_imu = -1.0;

    acc_0 = Zero3d;
    gyr_0 = Zero3d;

    Rs.push_back(Eigen::Matrix3d::Identity());
    Ps.push_back(Eigen::Vector3d::Zero());
    Vs.push_back(Eigen::Vector3d(0, 0, 0));
    Bas.push_back(Eigen::Vector3d{0.0, 0.0, 0.0});
    Bgs.push_back(Eigen::Vector3d(0.0, 0.0001, 0.0));
    pre_integrations.push_back(new Preintegration(acc_0, gyr_0, Bas[0], Bgs[0]));

    curR = lastR = Eigen::Matrix3d::Identity();
    curT = lastT = Eigen::Vector3d::Zero();

    vector<double> tmpSpeedBias;
    tmpSpeedBias.push_back(0.0);
    tmpSpeedBias.push_back(0.0);
    tmpSpeedBias.push_back(0.0);
    tmpSpeedBias.push_back(0.0);
    tmpSpeedBias.push_back(0.0);
    tmpSpeedBias.push_back(0.0);
    tmpSpeedBias.push_back(0.0);
    tmpSpeedBias.push_back(0.0);
    tmpSpeedBias.push_back(0.0);
    para_speed_bias.push_back(tmpSpeedBias);
    g = Eigen::Vector3d(0, 0, G_m_s2);
}

IMUProcessing::~IMUProcessing(){}

void IMUProcessing::setExtrinsic(const M3D &IMU_R_wrt_Car, const V3D &IMU_T_wrt_Car){
    IMU_R_wrt_Car_ = IMU_R_wrt_Car;
    IMU_t_wrt_Car_ = IMU_T_wrt_Car;
}

void IMUProcessing::setUndistFlag(const bool imu_rot_un, const bool imu_trans_un){
    imu_rot_un_ = imu_rot_un;
    imu_trans_un_ = imu_trans_un;
}

void IMUProcessing::TransformToStartIMU(PointType *p, const M3D unR, const V3D unT){
    V3D point{p->x, p->y, p->z};
    if(imu_rot_un_){
        point = unR.inverse() * point;
        if(imu_trans_un_){
            point -= unT;
        }
    }
}

void IMUProcessing::UndistortPcl(const MeasureGroup &meas, PointCloudXYZI &surf_point, PointCloudXYZI &edge_point){
    /*** sort point clouds by offset time ***/
    surf_point = *(meas.lidar_surf);
    sort(surf_point.points.begin(), surf_point.points.end(),[] (PointType &x, PointType &y) {return (x.curvature < y.curvature);});
    /*** undistort each lidar point (backward propagation) ***/
    if (surf_point.points.begin() == surf_point.points.end()) return;
    int cloudSize = surf_point.points.size();

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < cloudSize; ++ i) {
        PointType point;
        point.x = surf_point.points[i].x;
        point.y = surf_point.points[i].y;
        point.z = surf_point.points[i].z;
        double pointTime = surf_point.points[i].curvature / float(1000.0) + meas.lidar_beg_time;
        point.intensity = int(surf_point.points[i].intensity) + pointTime;

        int imuFrameFront = 0;
        while(pointTime > undist_IMU[imuFrameFront].time){
            if(++imuFrameFront == undist_IMU.size()){
                break;
            }
        }
        M3D unR;
        V3D unT;
        if(imuFrameFront == undist_IMU.size()){
            unR = undist_IMU.back().rot;
            unT = undist_IMU.back().trans;
        }else{
            double dt1 = pointTime - undist_IMU[imuFrameFront-1].time,
                    dt2 = undist_IMU[imuFrameFront].time - pointTime;
            unR = EulerToRzyx<double>(dt2/(dt1 + dt2) * RzyxToEuler(undist_IMU[imuFrameFront-1].rot) +
                                      dt1/(dt1 + dt2) * RzyxToEuler(undist_IMU[imuFrameFront].rot));
            unT = dt2/(dt1 + dt2) * undist_IMU[imuFrameFront-1].trans + dt1/(dt1 + dt2) * undist_IMU[imuFrameFront].trans;
        }
        TransformToStartIMU(&point, unR, unT);
        surf_point.points[i] = point;
    }

    edge_point = *(meas.lidar_edge);
    sort(edge_point.points.begin(), edge_point.points.end(),[] (PointType &x, PointType &y) {return (x.curvature < y.curvature);});
    /*** undistort each lidar point (backward propagation) ***/
    if (edge_point.points.begin() == edge_point.points.end()) return;
    cloudSize = edge_point.points.size();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < cloudSize; ++ i) {
        PointType point;
        point.x = edge_point.points[i].x;
        point.y = edge_point.points[i].y;
        point.z = edge_point.points[i].z;
        double pointTime = edge_point.points[i].curvature / float(1000.0) + meas.lidar_beg_time;
        point.intensity = int(edge_point.points[i].intensity) + pointTime;

        int imuFrameFront = 0;
        while(pointTime > undist_IMU[imuFrameFront].time){
            if(++imuFrameFront == undist_IMU.size())
                break;
        }
        M3D unR;
        V3D unT;
        if(imuFrameFront == undist_IMU.size()){
            unR = undist_IMU.back().rot;
            unT = undist_IMU.back().trans;
        }else{
            double dt1 = pointTime - undist_IMU[imuFrameFront-1].time,
                    dt2 = undist_IMU[imuFrameFront].time - pointTime;
            unR = EulerToRzyx<double>(dt2/(dt1 + dt2) * RzyxToEuler(undist_IMU[imuFrameFront-1].rot) +
                                      dt1/(dt1 + dt2) * RzyxToEuler(undist_IMU[imuFrameFront].rot));
            unT = dt2/(dt1 + dt2) * undist_IMU[imuFrameFront-1].trans + dt1/(dt1 + dt2) * undist_IMU[imuFrameFront].trans;
        }
        TransformToStartIMU(&point, unR, unT);
        edge_point.points[i] = point;
    }
}

void IMUProcessing::PreIntegration(const MeasureGroup &meas){
    double dx, dy, dz, rx, ry, rz;
    double qx, qy, qz, qw;
    uint imu_size = meas.imu.size();
    for(int imu_idx = 0; imu_idx < imu_size-1; ++imu_idx){
        double t = meas.imu[imu_idx]->header.stamp.toSec();
        if (cur_time_imu < 0)
            cur_time_imu = t;
        double dt = t - cur_time_imu;
        cur_time_imu = t;
        dx = meas.imu[imu_idx]->linear_acceleration.x;
        dy = meas.imu[imu_idx]->linear_acceleration.y;
        dz = meas.imu[imu_idx]->linear_acceleration.z;

        if(dx > 10.0) dx = 10.0;
        if(dy > 10.0) dy = 10.0;
        if(dz > 15.0) dz = 15.0;

        if(dx < -10.0) dx = -10.0;
        if(dy < -10.0) dy = -10.0;
        if(dz < -15.0) dz = -15.0;

        rx = meas.imu[imu_idx]->angular_velocity.x;
        ry = meas.imu[imu_idx]->angular_velocity.y;
        rz = meas.imu[imu_idx]->angular_velocity.z;
        processIMU(dt, Eigen::Vector3d(dx, dy, dz) , Eigen::Vector3d(rx, ry, rz));
    }
    double dt1 = meas.lidar_end_time - cur_time_imu;
    double dt2 = meas.imu.back()->header.stamp.toSec() - meas.lidar_end_time;
    double w1 = dt2 / (dt1 + dt2);
    double w2 = dt1 / (dt1 + dt2);
    dx = w1 * dx + w2 * meas.imu.back()->linear_acceleration.x;
    dy = w1 * dy + w2 * meas.imu.back()->linear_acceleration.y;
    dz = w1 * dz + w2 * meas.imu.back()->linear_acceleration.z;
    if(dx > 10.0) dx = 10.0;
    if(dy > 10.0) dy = 10.0;
    if(dz > 15.0) dz = 15.0;

    if(dx < -10.0) dx = -10.0;
    if(dy < -10.0) dy = -10.0;
    if(dz < -15.0) dz = -15.0;
    rx = w1 * rx + w2 * meas.imu.back()->angular_velocity.x;
    ry = w1 * ry + w2 * meas.imu.back()->angular_velocity.y;
    rz = w1 * rz + w2 * meas.imu.back()->angular_velocity.z;
    processIMU(dt1, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));
    cur_time_imu = meas.lidar_end_time;
    undist_IMU.back().time = cur_time_imu;
    imuR = Rs.back();
    imuT = Ps.back();
    if(flagInsPoseInit && imu_size > 1){
        Eigen::Quaternion<double> Q1{meas.imu[imu_size-2]->orientation.w, meas.imu[imu_size-2]->orientation.x,
                                     meas.imu[imu_size-2]->orientation.y, meas.imu[imu_size-2]->orientation.z};
        Eigen::Quaternion<double> Q2{meas.imu.back()->orientation.w, meas.imu.back()->orientation.x,
                                     meas.imu.back()->orientation.y, meas.imu.back()->orientation.z};
        V3D Eular1 = QuaternionToEuler<double>(Q1);
        V3D Eular2 = QuaternionToEuler<double>(Q2);
        V3D Eular = w1 * Eular1 + w2 * Eular2;
        imuQ = EulerToQuaternion(Eular);
    }
    lastR = curR;
    lastT = curT;
    curR = undist_IMU.back().rot;
    curT = undist_IMU.back().trans;
    return;
}

void IMUProcessing::processIMU(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity){
    if(pre_integrations.size()-1 < frame_count) {
        pre_integrations.push_back(new Preintegration(acc_0, gyr_0, Bas.back(), Bgs.back()));
        pre_integrations.back()->g_vec_ = -g;
        Bas.push_back(Bas.back());
        Bgs.push_back(Bgs.back());
        Rs.push_back(Rs.back());
        Ps.push_back(Ps.back());
        Vs.push_back(Vs.back());

        UnDistState undist;
        undist.time = 0.0;
        undist_IMU.clear();
        undist_IMU.push_back(undist);
        undist_IMU.back().rot = M3D::Identity();
        undist_IMU.back().trans = Zero3d;
    }
    Eigen::Vector3d un_acc_0 = Rs.back().inverse() * (acc_0 - Bas.back()) - g;
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs.back();
//    Rs.back() *= deltaQ(un_gyr * dt).toRotationMatrix();
    Rs.back() *= EulerToRzyx(V3D{un_gyr * dt});
    Eigen::Vector3d un_acc_1 = Rs.back().inverse() * (linear_acceleration - Bas.back()) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Ps.back() += dt * Vs.back() + 0.5 * dt * dt * un_acc;
    Vs.back() += dt * un_acc;
    pre_integrations.back()->push_back(dt, linear_acceleration, angular_velocity);
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;

    undist_IMU.push_back(undist_IMU.back());
    undist_IMU.back().time = cur_time_imu;
//    undist_IMU.back().rot *= deltaQ(un_gyr * dt).toRotationMatrix();
    undist_IMU.back().rot *= EulerToRzyx(V3D{un_gyr * dt});
    undist_IMU.back().trans += /*dt * Vs.back() +*/ 0.5 * dt * dt * un_acc;
}

void IMUProcessing::Process(const MeasureGroup &meas, PointCloudXYZI::Ptr surf_point, PointCloudXYZI::Ptr edge_point)
{
    ++frame_count;
    if(meas.imu.empty()) {
        cerr << "No IMU input!" << endl;
        return;
    }
    if(meas.lidar_surf->empty() || meas.lidar_edge->empty() ) {
        ROS_WARN("No Point Cloud input!");
        return;
    }

//    vector<double> observations_imus;
//    for(auto msg : meas.imu){
//        observations_imus.push_back(msg->linear_acceleration.z);
//    }
    PreIntegration(meas);
    UndistortPcl(meas, *surf_point, *edge_point);
}
