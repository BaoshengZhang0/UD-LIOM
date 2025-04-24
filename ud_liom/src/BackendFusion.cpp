#include "BackendFusion.h"
BackendFusion::BackendFusion(const shared_ptr<LiDARProcessing> &p_lidar, const shared_ptr<IMUProcessing> &p_imu){
    MaxIterationNum = 10;
    sizeLBA = 30;
    poseCurR = poseLastR = M3D::Identity();
    poseCurT = poseLastT = V3D::Zero();
    poseCurQ = poseLastQ = Eigen::Quaterniond(M3D::Identity());
    down_body_surf.reset(new PointCloudXYZI());
    down_body_edge.reset(new PointCloudXYZI());
    down_world_surf.reset(new PointCloudXYZI());
    down_world_edge.reset(new PointCloudXYZI());
    kd_tree_local.reset(new pcl::KdTreeFLANN<PointType>());
    last_feats_down_world.reset(new PointCloudXYZI());
    local_map.reset(new PointCloudXYZI());
    edge_W.reset(new PointCloudXYZI());
    surf_W.reset(new PointCloudXYZI());
    edge_B.reset(new PointCloudXYZI());
    surf_B.reset(new PointCloudXYZI());
    localMap_edge.reset(new PointCloudXYZI());
    localMap_surf.reset(new PointCloudXYZI());

    this->p_lidar = p_lidar;
    this->p_imu = p_imu;
    imu_enable = true;
    gtsam::ISAM2Params isamPara;
    isamPara.relinearizeThreshold = 0.1;
    isamPara.relinearizeSkip = 1;
//    isamPara.factorization = gtsam::ISAM2Params::CHOLESKY;
    isam = new gtsam::ISAM2(isamPara);
    idx_cur_key = 0;
    odom_prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8).finished());
    odom_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    local_map_flag = false;
    flg_rst = false;
}

void BackendFusion::setFeatDownPoint(const PointCloudXYZI::Ptr down_point_surf, const PointCloudXYZI::Ptr down_point_edge){
    *down_body_surf = *down_point_surf;
    *down_body_edge = *down_point_edge;
    size_down_surf = down_point_surf->size();
    size_down_edge = down_point_edge->size();
    nearest_surf_points.resize(size_down_surf);
    nearest_edge_points.resize(size_down_edge);
}

void BackendFusion::savePose(){
    poseLastR = poseCurR;
    poseLastT = poseCurT;
    poseLastQ = poseCurQ;
    timeLast = timeCur;
}

bool BackendFusion::FusionOptimizationCeres(){
    if(imu_enable){
        if(p_imu->flagInsPoseInit){
            poseCurQ = p_imu->orienta_initQ.inverse()*p_imu->imuQ;
            poseCurR = QuaternionToRzyx(poseCurQ);
        }else{
            poseCurR = EulerToRzyx(V3D{RzyxToEuler(poseLastR).transpose() + RzyxToEuler(p_imu->lastR).transpose()});
            poseCurQ = Eigen::Quaterniond(poseCurR).normalized();
        }
        poseCurT += p_imu->lastT;
    }
    double transformInc[7] = {poseCurQ.w(), poseCurQ.x(), poseCurQ.y(), poseCurQ.z()
            , poseCurT.x(), poseCurT.y(), poseCurT.z()};
    MaxIterationNum = 3;
    for(iterCount = 0; iterCount < MaxIterationNum; ++iterCount) {
        ceres::LossFunction *lossFunction = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *quatParameterization = new ceres:: QuaternionParameterization();
        ceres::Problem problem;

        problem.AddParameterBlock(transformInc, 4, quatParameterization);
        problem.AddParameterBlock(transformInc + 4, 3);
        /** closest surface search and residual computation **/
        surf_W->points.resize(size_down_surf);
        surf_B->points.resize(size_down_surf);
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i = 0; i < size_down_surf; i++) {
            PointType &point_body = down_body_surf->points[i];
            PointType point_world;
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(poseCurR * p_body + poseCurT);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;
            surf_B->points[i] = point_body;
            surf_W->points[i] = point_world;
            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            auto &points_near = nearest_surf_points[i];
            point_selected[i] = true;
            /** Find the closest surfaces in the map **/
            ikdtree_surf.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
            if (!point_selected[i]) {
                continue;
            }
            VF(4) pabcd;
            point_selected[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f)){
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
                if (s > 0.9){
                    point_selected[i] = true;
                    normvec->points[i].x = pabcd(0);
                    normvec->points[i].y = pabcd(1);
                    normvec->points[i].z = pabcd(2);
                    normvec->points[i].intensity = pabcd(3);
                    res_last[i] = abs(pd2);
                }
            }
        }
        effct_feat_num = 0;
        for (int i = 0; i < size_down_surf; i++) {
            if (point_selected[i]) {
                laserCloudOri->points[effct_feat_num] = down_body_surf->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                res_effect[effct_feat_num] = res_last[i];
                ++ effct_feat_num;
            }
        }
        if (effct_feat_num < 1) {
            ROS_WARN("No Effective Surf Points! \n");
            return false;
        }
        for (int i = 0; i < effct_feat_num; i++) {
            Eigen::Vector3d currentPt(laserCloudOri->points[i].x,
                                      laserCloudOri->points[i].y,
                                      laserCloudOri->points[i].z);
            Eigen::Vector3d norm(corr_normvect->points[i].x,
                                 corr_normvect->points[i].y,
                                 corr_normvect->points[i].z);
            double normInverse = corr_normvect->points[i].intensity;

            ceres::CostFunction *costFunction = LidarPlaneNormIncreFactor::Create(currentPt, norm, normInverse, res_effect[i]);
            if(iterCount == 0)
                problem.AddResidualBlock(costFunction, nullptr, transformInc, transformInc + 4);
            else
                problem.AddResidualBlock(costFunction, lossFunction, transformInc, transformInc + 4);
        }

//        edge_W->points.resize(size_down_edge);
//        edge_B->points.resize(size_down_edge);
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i = 0; i < size_down_edge; i++) {
            PointType &point_body = down_body_edge->points[i];
            PointType point_world;
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(poseCurR * p_body + poseCurT);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;
//            edge_B->points[i] = point_body;
//            edge_W->points[i] = point_world;
            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            auto &points_near = nearest_edge_points[i];
            point_selected[i] = true;
            /** Find the closest surfaces in the map **/
            ikdtree_edge.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
            if (!point_selected[i]) {
                continue;
            }
            VF(4) pabcd;
            point_selected[i] = false;
            Matrix<double, 3, 1> point_a;
            Matrix<double, 3, 1> point_b;
            double pd2;
            if (esti_line(point_a, point_b, point_world, points_near, pd2)){
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
                if (true){
                    point_selected[i] = true;
                    endpointa->points[i].x = point_a.x();
                    endpointa->points[i].y = point_a.y();
                    endpointa->points[i].z = point_a.z();
                    endpointb->points[i].x = point_b.x();
                    endpointb->points[i].y = point_b.y();
                    endpointb->points[i].z = point_b.z();
                    res_last[i] = abs(pd2);
                }
            }
        }
        effct_feat_num = 0;
        for (int i = 0; i < size_down_edge; i++) {
            if (point_selected[i]) {
                laserCloudOri->points[effct_feat_num] = down_body_edge->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];

                endpoint_a->points[effct_feat_num] = endpointa->points[i];
                endpoint_b->points[effct_feat_num] = endpointb->points[i];

                res_effect[effct_feat_num] = res_last[i];
                ++ effct_feat_num;
            }
        }
        if (effct_feat_num < 1) {
            ROS_WARN("No Effective Edge Points! \n");
            return false;
        }
        for (int i = 0; i < effct_feat_num; i++) {
            Eigen::Vector3d currentPt(laserCloudOri->points[i].x,
                                      laserCloudOri->points[i].y,
                                      laserCloudOri->points[i].z);
            Eigen::Vector3d norm(corr_normvect->points[i].x,
                                 corr_normvect->points[i].y,
                                 corr_normvect->points[i].z);
            double normInverse = corr_normvect->points[i].intensity;

            Eigen::Vector3d point_a(endpoint_a->points[i].x,
                                    endpoint_a->points[i].y,
                                    endpoint_a->points[i].z);
            Eigen::Vector3d point_b(endpoint_b->points[i].x,
                                    endpoint_b->points[i].y,
                                    endpoint_b->points[i].z);

            ceres::CostFunction *costFunction = LidarLineFactor::Create(currentPt, point_a, point_b, res_effect[i]);
            if(iterCount == 0)
                problem.AddResidualBlock(costFunction, nullptr, transformInc, transformInc + 4);
            else
                problem.AddResidualBlock(costFunction, lossFunction, transformInc, transformInc + 4);
        }

        if(imu_enable){
            ceres::CostFunction *costFunctionImu = ImuAutoFactorIncr::Create(poseLastQ,
                                                                         poseLastT,
                                                                         Quaterniond(p_imu->lastR).normalized(),
                                                                         p_imu->lastT);
            if(iterCount == 0)
                problem.AddResidualBlock(costFunctionImu, nullptr, transformInc,transformInc + 4);
            else
                problem.AddResidualBlock(costFunctionImu, lossFunction, transformInc,transformInc + 4);
        }

        if(imu_enable){
            ceres::CostFunction *costFunctionImu = ImuAutoFactorAbs::Create(poseLastQ *
                    Quaterniond(p_imu->lastR).normalized());
            if(iterCount == 0)
                problem.AddResidualBlock(costFunctionImu, nullptr, transformInc);
            else
                problem.AddResidualBlock(costFunctionImu, lossFunction, transformInc);
        }

        ceres::Solver::Options solverOptions;
        solverOptions.linear_solver_type = ceres::SPARSE_SCHUR;
        solverOptions.max_num_iterations = 3;
        solverOptions.max_solver_time_in_seconds = 0.03;
        solverOptions.num_threads = 3;
        solverOptions.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        solverOptions.minimizer_progress_to_stdout = false;
        solverOptions.check_gradients = false;
        solverOptions.gradient_check_relative_precision = 1e-2;
        ceres::Solver::Summary summary;
        ceres::Solve( solverOptions, &problem, &summary );
        if(transformInc[0] < 0) {
            Eigen::Quaterniond tmpQ(transformInc[0],
                                    transformInc[1],
                                    transformInc[2],
                                    transformInc[3]);
            if (tmpQ.w() < 0) {
                Eigen::Quaternion<double> resultQ(-tmpQ.w(), -tmpQ.x(), -tmpQ.y(), -tmpQ.z());
                tmpQ = resultQ;
            }
            transformInc[0] = tmpQ.w();
            transformInc[1] = tmpQ.x();
            transformInc[2] = tmpQ.y();
            transformInc[3] = tmpQ.z();
        }
        poseCurR = Eigen::Quaterniond(transformInc[0], transformInc[1], transformInc[2], transformInc[3]).normalized().toRotationMatrix();
        poseCurQ = Eigen::Quaterniond(poseCurR).normalized();
        poseCurT = V3D{transformInc[4], transformInc[5], transformInc[6]};

        double ave_cost = summary.final_cost / effct_feat_num;
        if(ave_cost > 0.05) {
            flg_rst = true;
        }

        if(ave_cost > 0.005){
            if(ave_cost < 0.01)
                MaxIterationNum = MaxIterationNum >= 5 ? 5 : ++ MaxIterationNum;
            else
                MaxIterationNum = MaxIterationNum >= MAX_NUM_ITERATION ? MAX_NUM_ITERATION : ++ MaxIterationNum;
        }
    }

    edge_Wscans.push_back(*edge_W);
    surf_Wscans.push_back(*surf_W);
    edge_Bscans.push_back(*edge_B);
    surf_Bscans.push_back(*surf_B);
    poseR_record.push_back(poseCurR);
    poseT_record.push_back(poseCurT);
    poseQ_record.push_back(poseCurQ);

    if(edge_LBA.empty() || RzyxToEuler(M3D(poseLastR.inverse() * poseCurR)).norm() > 0.005 || (poseCurT - pose_LBA.back().second).norm() > 5.0){
        edge_LBA.push_back(*edge_B);
        surf_LBA.push_back(*surf_B);
        pose_LBA.push_back(make_pair(make_pair(poseCurQ, poseCurR), poseCurT));
        if(imu_enable){
            local_pose_imu_LBA.push_back(make_pair(p_imu->lastR, p_imu->lastT));
        }
    }
    if(pose_LBA.size() > sizeLBA){
        edge_LBA.pop_front();
        surf_LBA.pop_front();
        pose_LBA.pop_front();
        if(imu_enable) local_pose_imu_LBA.pop_front();
    }
    return true;
}

void BackendFusion::LocalBAOptimizationCeres(){
    vector<array<double, 7>> transformIncs;
    for(auto const &pose : pose_LBA) {
        transformIncs.push_back(
                array<double, 7>{pose.first.first.w(), pose.first.first.x(), pose.first.first.y(), pose.first.first.z(),
                                 pose.second.x(), pose.second.y(), pose.second.z()});
    }
    MaxIterationNum = 3;
    for(iterCount = 0; iterCount < MaxIterationNum; ++iterCount) {
        ceres::LocalParameterization *quatParameterization = new ceres::QuaternionParameterization();
        ceres::Problem problem;

        PointVector PointToInsertEdge, PointToInsertSurf;
        PointType point_world;
        for(int i = 0; i < pose_LBA.size(); ++ i){
            for(auto p : edge_LBA[i]){
                V3D p_body(p.x, p.y, p.z);
                V3D p_global(pose_LBA[i].first.second * p_body + poseCurT);
                point_world.x = p_global(0);
                point_world.y = p_global(1);
                point_world.z = p_global(2);
                point_world.intensity = p.intensity;
                PointToInsertEdge.push_back(point_world);
            }
            for(auto p : surf_LBA[i]){
                V3D p_body(p.x, p.y, p.z);
                V3D p_global(pose_LBA[i].first.second * p_body + poseCurT);
                point_world.x = p_global(0);
                point_world.y = p_global(1);
                point_world.z = p_global(2);
                point_world.intensity = p.intensity;
                PointToInsertSurf.push_back(point_world);
            }
        }
        ikdtree_edgeLBA.Build(PointToInsertEdge);
//        ikdtree_edgeLBA.Delete_Points(PointToInsertEdge);
//        ikdtree_edgeLBA.Add_Points(PointToInsertEdge, true);
        ikdtree_surfLBA.Build(PointToInsertSurf);
//        ikdtree_surfLBA.Delete_Points(PointToInsertSurf);
//        ikdtree_surfLBA.Add_Points(PointToInsertSurf, true);
        for (int iBA = 0; iBA < transformIncs.size(); ++iBA) {
            problem.AddParameterBlock(transformIncs[iBA].begin(), 4, quatParameterization);
            problem.AddParameterBlock(transformIncs[iBA].begin() + 4, 3);

            int size_point = edge_LBA[iBA].points.size();
            nearest_edge_points.resize(size_point);
            #ifdef MP_EN
                omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
            #endif
            for (int i = 0; i < size_point; ++ i) {
                PointType &point_body = edge_LBA[iBA].points[i];
                PointType point_world;
                V3D p_body(point_body.x, point_body.y, point_body.z);
                V3D p_global(pose_LBA[iBA].first.second * p_body + pose_LBA[iBA].second);
                point_world.x = p_global(0);
                point_world.y = p_global(1);
                point_world.z = p_global(2);
                point_world.intensity = point_body.intensity;
                vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                auto &points_near = nearest_edge_points[i];
                point_selected[i] = true;
                /** Find the closest surfaces in the map **/
                ikdtree_edgeLBA.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
                if (!point_selected[i]) {
                    continue;
                }
                VF(4) pabcd;
                point_selected[i] = false;
                if (esti_plane(pabcd, points_near, 0.1f)){
                    float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
                    if (s > 0.9){
                        point_selected[i] = true;
                        normvec->points[i].x = pabcd(0);
                        normvec->points[i].y = pabcd(1);
                        normvec->points[i].z = pabcd(2);
                        normvec->points[i].intensity = pabcd(3);
                        res_last[i] = abs(pd2);
                    }
                }
            }
            effct_feat_num = 0;
            for (int i = 0; i < size_point; ++ i) {
                if (point_selected[i]) {
                    laserCloudOri->points[effct_feat_num] = edge_LBA[iBA].points[i];
                    corr_normvect->points[effct_feat_num] = normvec->points[i];
                    res_effect[effct_feat_num] = res_last[i];
                    ++effct_feat_num;
                }
            }
            if (effct_feat_num < 1) {
                ROS_WARN("No Effective Edge Points! \n");
                continue;
            }
            for (int i = 0; i < effct_feat_num; i++) {
                Eigen::Vector3d currentPt(laserCloudOri->points[i].x,
                                      laserCloudOri->points[i].y,
                                      laserCloudOri->points[i].z);
                Eigen::Vector3d norm(corr_normvect->points[i].x,
                                 corr_normvect->points[i].y,
                                 corr_normvect->points[i].z);
                double normInverse = corr_normvect->points[i].intensity;
                ceres::CostFunction *costFunction = LidarPlaneNormIncreFactor::Create(currentPt, norm, normInverse,
                                                                                  res_effect[i]);
                problem.AddResidualBlock(costFunction, nullptr, transformIncs[iBA].begin(), transformIncs[iBA].begin() + 4);
            }

            size_point = surf_LBA[iBA].points.size();
            nearest_surf_points.resize(size_point);
            #ifdef MP_EN
                omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
            #endif
            for (int i = 0; i < size_point; ++ i) {
                PointType &point_body = surf_LBA[iBA].points[i];
                PointType point_world;
                V3D p_body(point_body.x, point_body.y, point_body.z);
                V3D p_global(pose_LBA[iBA].first.second * p_body + pose_LBA[iBA].second);
                point_world.x = p_global(0);
                point_world.y = p_global(1);
                point_world.z = p_global(2);
                point_world.intensity = point_body.intensity;
                vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                auto &points_near = nearest_surf_points[i];
                point_selected[i] = true;
                /** Find the closest surfaces in the map **/
                ikdtree_surfLBA.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
                if (!point_selected[i]) {
                    continue;
                }
                VF(4) pabcd;
                point_selected[i] = false;
                if (esti_plane(pabcd, points_near, 0.1f)){
                    float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
                    if (s > 0.9){
                        point_selected[i] = true;
                        normvec->points[i].x = pabcd(0);
                        normvec->points[i].y = pabcd(1);
                        normvec->points[i].z = pabcd(2);
                        normvec->points[i].intensity = pabcd(3);
                        res_last[i] = abs(pd2);
                    }
                }
            }
            effct_feat_num = 0;
            for (int i = 0; i < size_point; ++ i) {
                if (point_selected[i]) {
                    laserCloudOri->points[effct_feat_num] = surf_LBA[iBA].points[i];
                    corr_normvect->points[effct_feat_num] = normvec->points[i];
                    res_effect[effct_feat_num] = res_last[i];
                    ++effct_feat_num;
                }
            }
            if (effct_feat_num < 1) {
                ROS_WARN("No Effective Surf Points! \n");
                continue;
            }
            for (int i = 0; i < effct_feat_num; i++) {
                Eigen::Vector3d currentPt(laserCloudOri->points[i].x,
                                          laserCloudOri->points[i].y,
                                          laserCloudOri->points[i].z);
                Eigen::Vector3d norm(corr_normvect->points[i].x,
                                     corr_normvect->points[i].y,
                                     corr_normvect->points[i].z);
                double normInverse = corr_normvect->points[i].intensity;
                ceres::CostFunction *costFunction = LidarPlaneNormIncreFactor::Create(currentPt, norm, normInverse,
                                                                                      res_effect[i]);
                problem.AddResidualBlock(costFunction, nullptr, transformIncs[iBA].begin(), transformIncs[iBA].begin() + 4);
            }
            //imu factor insert
            if(imu_enable && iBA > 1){
                ceres::CostFunction *costFunctionImu = ImuAutoFactorIncr::Create( pose_LBA[iBA-1].first.first,
                                                                                  pose_LBA[iBA-1].second,
                                                                                 Quaterniond(local_pose_imu_LBA[iBA].first).normalized(),
                                                                                  local_pose_imu_LBA[iBA].second);
                problem.AddResidualBlock(costFunctionImu, nullptr, transformIncs[iBA].begin(), transformIncs[iBA].begin() + 4);
            }
        }
        ceres::Solver::Options solverOptions;
        solverOptions.linear_solver_type = ceres::SPARSE_SCHUR;
        solverOptions.max_num_iterations = 3;
        solverOptions.max_solver_time_in_seconds = 0.1;
        solverOptions.num_threads = 3;
        solverOptions.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        solverOptions.minimizer_progress_to_stdout = false;
        solverOptions.check_gradients = false;
        solverOptions.gradient_check_relative_precision = 1e-2;
        ceres::Solver::Summary summary;
        ceres::Solve( solverOptions, &problem, &summary );
        for(int pose = 10; pose < transformIncs.size(); ++ pose){
            if(transformIncs[pose][0] < 0) {
                Eigen::Quaterniond tmpQ(transformIncs[pose][0],
                                        transformIncs[pose][1],
                                        transformIncs[pose][2],
                                        transformIncs[pose][3]);
                if (tmpQ.w() < 0) {
                    Eigen::Quaternion<double> resultQ(-tmpQ.w(), -tmpQ.x(), -tmpQ.y(), -tmpQ.z());
                    tmpQ = resultQ;
                }
                transformIncs[pose][0] = tmpQ.w();
                transformIncs[pose][1] = tmpQ.x();
                transformIncs[pose][2] = tmpQ.y();
                transformIncs[pose][3] = tmpQ.z();
            }

            poseCurR = Eigen::Quaterniond(transformIncs[pose][0], transformIncs[pose][1], transformIncs[pose][2], transformIncs[pose][3]).normalized().toRotationMatrix();
            poseCurQ = Eigen::Quaterniond(poseCurR).normalized();
            poseCurT = V3D{transformIncs[pose][4], transformIncs[pose][5], transformIncs[pose][6]};
            pose_LBA[pose].first.first = poseCurQ;
            pose_LBA[pose].first.second = poseCurR;
            pose_LBA[pose].second = poseCurT;
        }
    }
}