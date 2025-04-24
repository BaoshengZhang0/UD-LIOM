#include <nav_msgs/Path.h>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <csignal>
#include <condition_variable>
#include <future>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <livox_ros_driver/CustomMsg.h>
#include <ikd-Tree/ikd_Tree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "factors/LidarKeyframeFactor.h"
#include "factors/Preintegration.h"
#include "factors/ImuFactor.h"
#include "LiDAR_Processing.h"
#include "IMU_Processing.h"
#include "BackendFusion.h"

#ifndef _PREPROCESS_H
#define _PREPROCESS_H

mutex mtx_buffer;
condition_variable sig_buffer;

double last_timestamp_lidar = 0.0, last_timestamp_imu = -1.0, last_timestamp_gnss = -1.0;
double filter_size_surf_min = 0, filter_size_map_min = 0;
double filter_radius_search = 2.0;
int filter_min_neighbors = 1;
double cube_len = 50, move_scale = 0.2, lidar_end_time = 0;
int size_down_surf = 0, size_down_edge = 0, size_pre_surf = 0, size_pre_edge = 0;
float edge_diffrange = 1.0, surf_diffrange = 10.0;
bool lidar_pushed, flg_exit = false, flg_inited = false;
vector<BoxPointType> cub_needrm;
vector<double> ext_R_IMU2Car(3, 0.0),   ext_t_IMU2Car(3, 0.0);
vector<double> ext_R_Lidar2Car(3, 0.0), ext_t_Lidar2Car(3, 0.0);

PointCloudXYZI::Ptr undistort_surf(new PointCloudXYZI()), undistort_edge(new PointCloudXYZI());
PointCloudXYZI::Ptr preprocess_surf(new PointCloudXYZI()), preprocess_edge(new PointCloudXYZI());
PointCloudXYZI::Ptr preprocess_last_surf(new PointCloudXYZI()), preprocess_last_edge(new PointCloudXYZI());
PointCloudXYZI::Ptr down_body_surf(new PointCloudXYZI()), down_body_edge(new PointCloudXYZI());
PointCloudXYZI::Ptr down_world_surf(new PointCloudXYZI()), down_world_edge(new PointCloudXYZI());

PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr endpointa(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr endpointb(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr endpoint_a(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr endpoint_b(new PointCloudXYZI(100000, 1));

pcl::VoxelGrid<PointType> downSizeFilterScanVox;
pcl::VoxelGrid<PointType> downSizeFilterMapVox;

pcl::RadiusOutlierRemoval<PointType> outlierFilterRad;
pcl::StatisticalOutlierRemoval<PointType> outlierFilterStat;

MeasureGroup MeasuresCur;
deque<MeasureGroup* > DeMeasures;
nav_msgs::Path path;
geometry_msgs::PoseStamped msg_body_pose;
shared_ptr<LiDARProcessing> p_lidar(new LiDARProcessing());
shared_ptr<IMUProcessing> p_imu(new IMUProcessing());
shared_ptr<BackendFusion> fusion_back(new BackendFusion(p_lidar, p_imu));
string root_path = ROOT_DIR;
std::ofstream path_data(root_path + "/path.txt");

PointCloudXYZI::Ptr GLOAB_MAP(new PointCloudXYZI);

void sigHandle(int sig){
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

void initSetting(){
    downSizeFilterScanVox.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMapVox.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    outlierFilterRad.setRadiusSearch(0.5);
    p_imu->setExtrinsic(EulerToRzyx(V3D{ext_R_IMU2Car[0], ext_R_IMU2Car[1], ext_R_IMU2Car[2]}),
                        V3D{ext_t_IMU2Car[0], ext_t_IMU2Car[1], ext_t_IMU2Car[2]});
    p_imu->setUndistFlag(true, false);
    p_lidar->setExtrinsic(EulerToRzyx(V3D{ext_R_Lidar2Car[0], ext_R_Lidar2Car[1], ext_R_Lidar2Car[2]}),
                          V3D{ext_t_Lidar2Car[0], ext_t_Lidar2Car[1], ext_t_Lidar2Car[2]});
    path.header.stamp    = ros::Time::now();
    path.header.frame_id = "ud_liom";
    fusion_back->path_fusion.header.stamp = ros::Time::now();
    fusion_back->path_fusion.header.frame_id = "ud_liom";
    preprocess_surf->clear();
    preprocess_edge->clear();
}

void pointBodyToWorld(PointType const * const pi, PointType * const po){
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(fusion_back->poseCurR * p_body + fusion_back->poseCurT);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType &pi, PointType &po, const M3D &R, const V3D &t){
    V3D p_body(pi.x, pi.y, pi.z);
    V3D p_global(R * p_body + t);
    po.x = p_global(0);
    po.y = p_global(1);
    po.z = p_global(2);
    po.intensity = pi.intensity;
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

void lasermap_fov_segment(){
    cub_needrm.clear();
//    kdtree_delete_counter = 0;
    V3D pos_LiD = fusion_back->poseCurT;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= move_scale * cube_len || dist_to_map_edge[i][1] <= move_scale * cube_len) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = 0.5 * cube_len;
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= move_scale * cube_len){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= move_scale * cube_len){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;
    if(cub_needrm.size() > 0) {
        fusion_back->ikdtree_surf.Delete_Point_Boxes(cub_needrm);
        fusion_back->ikdtree_edge.Delete_Point_Boxes(cub_needrm);
    }
}

void calculateSmoothness(PointCloudXYZI::Ptr &ptr_surf, PointCloudXYZI::Ptr &ptr_edge){
    for(auto ring : p_lidar->rings_buffer){
        int point_num = ring->size();
        for(int i = 5; i < point_num-5; ++i){
            if(isnan(ring->points[i].normal_x)) continue;
            float diffRange = ring->points[i-5].normal_x + ring->points[i-4].normal_x
                              + ring->points[i-3].normal_x + ring->points[i-2].normal_x
                              + ring->points[i-1].normal_x - ring->points[i].normal_x * 10
                              + ring->points[i+1].normal_x + ring->points[i+2].normal_x
                              + ring->points[i+3].normal_x + ring->points[i+4].normal_x
                              + ring->points[i+5].normal_x;
            if(!isnan(diffRange)){
                if(abs(diffRange) < surf_diffrange)
                    ptr_surf->points.push_back(ring->points[i]);
                else if(abs(diffRange) >= edge_diffrange)
                    ptr_edge->points.push_back(ring->points[i]);
            }
        }
        ring->clear();
    }
}

void pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg){
    if(!flg_inited) return;
    mtx_buffer.lock();
    double time_sec = msg->header.stamp.toSec();
    PointCloudXYZI::Ptr ptr_surf(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_edge(new PointCloudXYZI());
    if (time_sec < last_timestamp_lidar){
        ROS_ERROR("LiDAR msg time error, clear buffer");
        p_lidar->surf_buffer.clear();
        p_lidar->edge_buffer.clear();
    }
    p_lidar->process(msg);
    calculateSmoothness(ptr_surf, ptr_edge);
    p_lidar->surf_buffer.push_back(ptr_surf);
    p_lidar->edge_buffer.push_back(ptr_edge);
    p_lidar->time_buffer.push_back(time_sec);
    last_timestamp_lidar = time_sec;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in){
    if(!fusion_back->imu_enable) return;
    ++ p_imu->imu_count;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    double timestamp = msg->header.stamp.toSec();
    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu){
        ROS_WARN("IMU msg time error, clear buffer");
        p_imu->imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    p_imu->imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    if(!p_imu->flagImuInit){
        if(!p_imu->flagInsPoseInit)
            p_imu->flagImuInit = true;
        else if(msg->orientation.x*msg->orientation.x+
        msg->orientation.y*msg->orientation.y+
        msg->orientation.z*msg->orientation.z+
        msg->orientation.w*msg->orientation.w > 0.1){
            p_imu->orienta_initQ.x() = msg->orientation.x;
            p_imu->orienta_initQ.y() = msg->orientation.y;
            p_imu->orienta_initQ.z() = msg->orientation.z;
            p_imu->orienta_initQ.w() = msg->orientation.w;
            p_imu->flagImuInit = true;
        }
    }
}

void sync_packages(){
    while(true){
        static int conuter_scan = 0;
        static double lidar_mean_scantime = 0.1;
        mtx_buffer.lock();
        if (p_lidar->surf_buffer.empty() || p_lidar->edge_buffer.empty()) {
            mtx_buffer.unlock();
            sig_buffer.notify_all();
            continue;
        }
        MeasureGroup *meas = new MeasureGroup();
        if (!lidar_pushed) {
            *(meas->lidar_surf) = *(p_lidar->surf_buffer.front());
            *(meas->lidar_edge) = *(p_lidar->edge_buffer.front());
            meas->lidar_beg_time = p_lidar->time_beg_buffer.front() / float(1000);
            if (meas->lidar_surf->points.size() <= 1 && meas->lidar_edge->points.size() <= 1) {
                lidar_end_time = meas->lidar_beg_time + lidar_mean_scantime;
                ROS_WARN("Too few input point cloud!\n");
            } else if (max(meas->lidar_surf->points.back().curvature, meas->lidar_edge->points.back().curvature) / double(1000) < 0.5 * lidar_mean_scantime) {
                lidar_end_time = meas->lidar_beg_time + lidar_mean_scantime;
            } else {
                conuter_scan++;
                lidar_end_time = meas->lidar_beg_time + max(meas->lidar_surf->points.back().curvature, meas->lidar_edge->points.back().curvature) / double(1000);
                lidar_mean_scantime +=
                        (max(meas->lidar_surf->points.back().curvature, meas->lidar_edge->points.back().curvature) / double(1000) - lidar_mean_scantime) / conuter_scan;
            }
            meas->lidar_end_time = lidar_end_time;
            lidar_pushed = true;
        }
        if (fusion_back->imu_enable) {
            if (last_timestamp_imu < meas->lidar_end_time) {
                mtx_buffer.unlock();
                sig_buffer.notify_all();
                continue;
            }
            /*** push imu data, and pop from imu buffer ***/
            double imu_time = p_imu->imu_buffer.front()->header.stamp.toSec();
            meas->imu.clear();
            while ((!p_imu->imu_buffer.empty()) && (imu_time < meas->lidar_end_time)) {
                imu_time = p_imu->imu_buffer.front()->header.stamp.toSec();
                if (imu_time >= meas->lidar_end_time) break;
                meas->imu.push_back(p_imu->imu_buffer.front());
                p_imu->imu_buffer.pop_front();
            }
            if (!p_imu->imu_buffer.empty()) {
                meas->imu.push_back(p_imu->imu_buffer.front());
            }
        }
        p_lidar->surf_buffer.pop_front();
        p_lidar->edge_buffer.pop_front();
        p_lidar->time_buffer.pop_front();
        p_lidar->time_beg_buffer.pop_front();
        p_lidar->time_end_buffer.pop_front();
        lidar_pushed = false;
        DeMeasures.push_back(meas);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }
}

bool sys_reset(){
    if(!fusion_back->flg_rst) return false;
    fusion_back->flg_rst = false;
    fusion_back->ikdtree_surf.Root_Node = nullptr;
    fusion_back->poseCurT = fusion_back->poseLastT;
    fusion_back->poseCurR = fusion_back->poseLastR;
    fusion_back->poseCurQ = fusion_back->poseLastQ;
    return true;
}

void map_incremental(){
    if(fusion_back->ikdtree_surf.Root_Node == nullptr || fusion_back->ikdtree_edge.Root_Node == nullptr) return;
    PointVector PointToAdd_surf, PointToAdd_edge;
    PointVector PointNoNeedDownsample_surf, PointNoNeedDownsample_edge;
    PointToAdd_surf.reserve(size_down_surf);
    PointToAdd_edge.reserve(size_down_surf);
    PointNoNeedDownsample_surf.reserve(size_down_surf);
    PointNoNeedDownsample_edge.reserve(size_down_surf);

    for (int i = 0; i < size_down_surf; i++) {
        /* transform to world frame */
        pointBodyToWorld(&(down_body_surf->points[i]), &(down_world_surf->points[i]));
        /* decide if need add to map */
        if (!fusion_back->nearest_surf_points[i].empty())
        {
            const PointVector &points_near = fusion_back->nearest_surf_points[i];
            bool need_add = true;
            PointType mid_point;
            mid_point.x = floor(down_world_surf->points[i].x/filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(down_world_surf->points[i].y/filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(down_world_surf->points[i].z/filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(down_world_surf->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
            fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
            fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample_surf.push_back(down_world_surf->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd_surf.push_back(down_world_surf->points[i]);
        }
        else
        {
            PointToAdd_surf.push_back(down_world_surf->points[i]);
        }
    }

    for (int i = 0; i < size_down_edge; i++) {
        /* transform to world frame */
        pointBodyToWorld(&(down_body_edge->points[i]), &(down_world_edge->points[i]));
        /* decide if need add to map */
        if (!fusion_back->nearest_surf_points[i].empty())
        {
            const PointVector &points_near = fusion_back->nearest_edge_points[i];
            bool need_add = true;
            PointType mid_point;
            mid_point.x = floor(down_world_edge->points[i].x/filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(down_world_edge->points[i].y/filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(down_world_edge->points[i].z/filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(down_world_edge->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample_edge.push_back(down_world_edge->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd_edge.push_back(down_world_edge->points[i]);
        }
        else
        {
            PointToAdd_edge.push_back(down_world_edge->points[i]);
        }
    }
    mtx_buffer.lock();
    fusion_back->ikdtree_surf.Add_Points(PointToAdd_surf, true);
    fusion_back->ikdtree_surf.Add_Points(PointNoNeedDownsample_surf, false);
    fusion_back->ikdtree_edge.Add_Points(PointToAdd_edge, true);
    fusion_back->ikdtree_edge.Add_Points(PointNoNeedDownsample_edge, false);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

template<typename T>
void set_posestamp(T & out){
    out.pose.position.x = fusion_back->poseCurT.x();
    out.pose.position.y = fusion_back->poseCurT.y();
    out.pose.position.z = fusion_back->poseCurT.z();
    out.pose.orientation.x = fusion_back->poseCurQ.x();
    out.pose.orientation.y = fusion_back->poseCurQ.y();
    out.pose.orientation.z = fusion_back->poseCurQ.z();
    out.pose.orientation.w = fusion_back->poseCurQ.w();
}

void pc_process(const MeasureGroup &meas, PointCloudXYZI::Ptr surf_point, PointCloudXYZI::Ptr edge_point){
    *surf_point = *(meas.lidar_surf);
    *edge_point = *(meas.lidar_edge);
}

void publish_path(const ros::Publisher &pubPath){
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "ud_liom";
    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 3 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
        return;
    }
}

void publish_point(const ros::Publisher &pubPoint){
    GLOAB_MAP->clear();
    PointCloudXYZI::Ptr points_edge(new PointCloudXYZI());
    PointCloudXYZI::Ptr points_surf(new PointCloudXYZI());
    for(auto p : *fusion_back->Measure.lidar_edge){
        PointType p_e;
        pointBodyToWorld(p, p_e, fusion_back->poseCurR, fusion_back->poseCurT);
        points_edge->points.push_back(p_e);
    }
    for(auto p : *fusion_back->Measure.lidar_surf){
        PointType p_s;
        pointBodyToWorld(p, p_s, fusion_back->poseCurR, fusion_back->poseCurT);
        points_surf->points.push_back(p_s);
    }
    *GLOAB_MAP += *points_edge;
    *GLOAB_MAP += *points_surf;

    sensor_msgs::PointCloud2 laserCloudmsg, laserCloudmsg_surf, laserCloudmsg_edge;
    pcl::toROSMsg(*GLOAB_MAP, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "ud_liom";
    pubPoint.publish(laserCloudmsg);
}

void paramsGetting(ros::NodeHandle &nh){
    nh.param<double>("preprocess/blind", p_lidar->blind, 0.01);
    nh.param<int>("preprocess/max_iteration", MAX_NUM_ITERATION, 10);
    nh.param<int>("preprocess/scan_line", p_lidar->N_SCANS, 16);
    nh.param<int>("preprocess/scan_rate", p_lidar->SCAN_RATE, 10);
    nh.param<int>("preprocess/point_filter_num", p_lidar->point_filter_num, 2);
    nh.param<float>("preprocess/edge_diffrange", edge_diffrange, 1.0);
    nh.param<float>("preprocess/surf_diffrange", surf_diffrange, 10.0);
    nh.param<int>("preprocess/downsampleRate", p_lidar->downsampleRate, 1);
    nh.param<bool>("preprocess/imu_enable", fusion_back->imu_enable, false);
    nh.param<bool>("preprocess/ins_pose_init", p_imu->flagInsPoseInit, false);
    nh.param<double>("mapping/filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("mapping/filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("mapping/filter_radius_search",filter_radius_search,2.0);
    nh.param<int>("mapping/filter_min_neighbors",filter_min_neighbors,1);
    nh.param<double>("mapping/cube_side_length", cube_len, 200);
    nh.param<double>("mapping/move_scale", move_scale, 0.2);
    nh.param<vector<double>>("mapping/ext_R_IMU2Car", ext_R_IMU2Car, vector<double>());
    nh.param<vector<double>>("mapping/ext_t_IMU2Car", ext_t_IMU2Car, vector<double>());
    nh.param<vector<double>>("mapping/ext_R_Lidar2Car", ext_R_Lidar2Car, vector<double>());
    nh.param<vector<double>>("mapping/ext_t_Lidar2Car", ext_t_Lidar2Car, vector<double>());
}

void estimating(const ros::Publisher &pubPoint, const ros::Publisher &pubPath){
    bool state;
    while(true){
        mtx_buffer.lock();
        state = !preprocess_surf->empty() && !preprocess_edge->empty() && flg_inited;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        if(!state) {
            continue;
        }
        mtx_buffer.lock();
        *down_body_surf = *preprocess_surf;
        *preprocess_last_surf = *preprocess_surf;
        *down_body_edge = *preprocess_edge;
        *preprocess_last_edge = *preprocess_edge;
        preprocess_surf->clear();
        preprocess_edge->clear();
        fusion_back->Measure = MeasuresCur;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        size_down_surf = down_body_surf->size();
        size_down_edge = down_body_edge->size();
        if (size_down_surf < 5 && size_down_edge < 5){
            ROS_WARN("No point, skip this scan!");
            continue;
        }
        down_world_surf->resize(size_down_surf);
        fusion_back->down_world_surf->resize(size_down_surf);
        for (int i = 0; i < size_down_surf; i++) {
            pointBodyToWorld(&(down_body_surf->points[i]), &(fusion_back->down_world_surf->points[i]));
        }
        down_world_edge->resize(size_down_edge);
        fusion_back->down_world_edge->resize(size_down_edge);
        for (int i = 0; i < size_down_edge; i++) {
            pointBodyToWorld(&(down_body_edge->points[i]), &(fusion_back->down_world_edge->points[i]));
        }
        /*** iterated state estimation ***/
        fusion_back->setFeatDownPoint(down_body_surf, down_body_edge);
        fusion_back->savePose();
        if(!fusion_back->FusionOptimizationCeres()){
            ROS_WARN("No features, skip this scan!");
            continue;
        }
        /*** add the feature points to map kdtree ***/
        map_incremental();
        /******* Publish msg *******/
        publish_point(pubPoint);
        publish_path(pubPath);
        mtx_buffer.lock();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        V3D rot_cur = RzyxToEuler(fusion_back->poseCurR) / M_PI * 180.0;
        V3D rot_last = RzyxToEuler(fusion_back->poseLastR) / M_PI * 180.0;
        if(rot_cur.z() > 0.0){
            rot_cur.z() = 360.0 - rot_cur.z();
        }else{
            rot_cur.z() = -rot_cur.z();
        }
        if(rot_last.z() > 0.0){
            rot_last.z() = 360.0 - rot_last.z();
        }else{
            rot_last.z() = -rot_last.z();
        }
    }
}
#endif
