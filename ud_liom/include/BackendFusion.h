#ifndef LIO_BACKENDFUSION_H
#define LIO_BACKENDFUSION_H
#include <omp.h>
#include <mutex>
#include <thread>
#include <csignal>
#include <condition_variable>
#include <ikd-Tree/ikd_Tree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <nav_msgs/Path.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include "factors/LidarKeyframeFactor.h"
#include "factors/ImuFactor.h"
#include "LiDAR_Processing.h"
#include "IMU_Processing.h"

static int MAX_NUM_ITERATION = 10;

class BackendFusion {
public:
    BackendFusion(const shared_ptr<LiDARProcessing> &p_lidar, const shared_ptr<IMUProcessing> &p_imu);
    ~BackendFusion(){};
    bool FeatureLMOptimizationCeresAutoDiff();
    bool FeatureLMOptimizationCeresHandDiff();
    bool FusionOptimizationCeres();
    void LocalBAOptimizationCeres();
    void setFeatDownPoint(const PointCloudXYZI::Ptr down_body_surf, const PointCloudXYZI::Ptr down_body_edge);
    void savePose();
    void OptimizationFactorGraph();
    void OptimizationGNSSFactorGraph();

    M3D poseCurR, poseLastR, poseFusionR;
    V3D poseCurT, poseLastT, poseFusionT;
    Eigen::Quaterniond poseCurQ, poseLastQ, poseFusionQ;

    vector<M3D> poseR_record;
    vector<V3D> poseT_record;
    vector<Eigen::Quaterniond> poseQ_record;

    V3D UTMCur, UTMLast;
    double timeCur, timeLast;

    PointCloudXYZI::Ptr normvec;
    PointCloudXYZI::Ptr laserCloudOri;

    PointCloudXYZI::Ptr endpointa;
    PointCloudXYZI::Ptr endpointb;
    PointCloudXYZI::Ptr endpoint_a;
    PointCloudXYZI::Ptr endpoint_b;

    PointCloudXYZI::Ptr corr_normvect;
    int size_down_surf, size_down_edge;
    PointCloudXYZI::Ptr down_body_surf, down_body_edge;
    PointCloudXYZI::Ptr down_world_surf, down_world_edge;
    vector<PointVector>  nearest_surf_points, nearest_edge_points;
    bool point_selected[100000] = {0};
    KD_TREE<PointType> ikdtree_surf, ikdtree_edge;
    pcl::KdTreeFLANN<PointType >::Ptr kd_tree_local;
    PointCloudXYZI::Ptr last_feats_down_world;
    PointCloudXYZI::Ptr local_map;

    PointCloudXYZI::Ptr edge_W, edge_B;
    PointCloudXYZI::Ptr surf_W, surf_B;
    vector<PointCloudXYZI> edge_Wscans, edge_Bscans;
    vector<PointCloudXYZI> surf_Wscans, surf_Bscans;
    //Local BA
    deque<PointCloudXYZI> edge_LBA;
    deque<PointCloudXYZI> surf_LBA;
    deque<pair<pair<Eigen::Quaterniond, M3D>, V3D>> pose_LBA;
    PointCloudXYZI::Ptr localMap_edge;
    PointCloudXYZI::Ptr localMap_surf;
    KD_TREE<PointType> ikdtree_surfLBA, ikdtree_edgeLBA;
    deque<pair<M3D, V3D>> local_pose_imu_LBA;

    float res_last[100000] = {0.0};
    float res_effect[100000] = {0.0};
    int iterCount;
    int effct_feat_num;
    int MaxIterationNum;
    bool imu_enable;
    V3D imu_pre_rot;
    shared_ptr<LiDARProcessing> p_lidar;
    shared_ptr<IMUProcessing> p_imu;
    double *SpeedBiasLast = new double[9];
    double *SpeedBias = new double[9];
    MeasureGroup Measure;
    mutex mtx_buffer;
    condition_variable sig_buffer;
    bool local_map_flag;
    bool flg_rst;

    nav_msgs::Path path_fusion;
    geometry_msgs::PoseStamped msg_fusion_pose;
private:
    gtsam::NonlinearFactorGraph GtsamGraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;

    gtsam::noiseModel::Diagonal::shared_ptr odom_prior_noise;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise;
    int idx_cur_key;
    gtsam::KeyVector keysToMarginalize;
    vector<int> idxSW;
    int sizeLBA;
};


#endif //LIO_BACKENDFUSION_H
