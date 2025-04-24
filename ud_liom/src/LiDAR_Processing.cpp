#include "LiDAR_Processing.h"
LiDARProcessing::LiDARProcessing()
        : blind(1.0), point_filter_num(2)
{
    flg_first_scan = true;
    N_SCANS = 5;
    SCAN_RATE = 10;
    given_offset_time = false;
    lidar_count = 0;
    lidar_begin_time = 0.0;
    lidar_end_time = 0.0;
    first_lidar_time = 0.0;
    last_timestamp_lidar = -1.0;
//    lidar_buffer.clear();
    surf_buffer.clear();
    edge_buffer.clear();
    time_buffer.clear();
    time_beg_buffer.clear();
    time_end_buffer.clear();
    ext_R_Lidar2Car = M3D::Identity();
    ext_t_Lidar2Car = V3D::Zero();
}
LiDARProcessing::~LiDARProcessing() {}

void LiDARProcessing::setExtrinsic(const M3D &Lidar_R_wrt_Car, const V3D &Lidar_T_wrt_Car){
    ext_R_Lidar2Car = Lidar_R_wrt_Car;
    ext_t_Lidar2Car = Lidar_T_wrt_Car;
}

void LiDARProcessing::process(const sensor_msgs::PointCloud2::ConstPtr &msg){
    pcl::PointCloud<PointXYZIRT>::Ptr pl_in(new pcl::PointCloud<PointXYZIRT>());
    pcl::fromROSMsg(*msg, *pl_in);
    pl_in->is_dense = false;
    double scan_time = msg->header.stamp.toSec() * float(1000); //ms
    int pl_size = pl_in->points.size();
    for (int i = 0; i < pl_size; i += point_filter_num){
        if(pl_in->points[i].ring % downsampleRate != 0) continue;
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        V3D tmp_add{added_pt.x, added_pt.y, added_pt.z};
        V3D tmp_orig{pl_in->points[i].x, pl_in->points[i].y, pl_in->points[i].z};
        tmp_add = ext_R_Lidar2Car.inverse() * tmp_orig - ext_t_Lidar2Car;

        added_pt.x = tmp_add.x();
        added_pt.y = tmp_add.y();
        added_pt.z = tmp_add.z();

        added_pt.intensity = pl_in->points[i].intensity;
        added_pt.curvature = pl_in->points[i].timestamp;  // curvature unit: ms
        rings_buffer[pl_in->points[i].ring]->push_back(added_pt);
        rings_buffer[pl_in->points[i].ring]->back().normal_x = added_pt.x * added_pt.x + added_pt.y * added_pt.y;
    }
    lidar_begin_time = scan_time + pl_in->points.front().timestamp;
    lidar_end_time = scan_time + pl_in->points.back().timestamp;
    time_beg_buffer.push_back(lidar_begin_time);
    time_end_buffer.push_back(lidar_end_time);
}

void LiDARProcessing::oust_handler(const sensor_msgs::PointCloud2::ConstPtr &msg){
    pcl::PointCloud<OUSTPoint::PointXYZIRT>::Ptr pl_in(new pcl::PointCloud<OUSTPoint::PointXYZIRT>());
    pcl::fromROSMsg(*msg, *pl_in);
    pl_in->is_dense = false;
    double scan_time = msg->header.stamp.toSec() * float(1000);
    int pl_size = pl_in->points.size();

    for (int i = 0; i < pl_size; i += point_filter_num){
        if(pl_in->points[i].ring % downsampleRate != 0) continue;
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        V3D tmp_add{added_pt.x, added_pt.y, added_pt.z};
        V3D tmp_orig{pl_in->points[i].x, pl_in->points[i].y, pl_in->points[i].z};
        tmp_add = ext_R_Lidar2Car.inverse() * tmp_orig - ext_t_Lidar2Car;

        added_pt.x = tmp_add.x();
        added_pt.y = tmp_add.y();
        added_pt.z = tmp_add.z();

        added_pt.intensity = pl_in->points[i].intensity;
        added_pt.curvature = pl_in->points[i].t / float(1000000);  // curvature unit: ms
        rings_buffer[pl_in->points[i].ring]->push_back(added_pt);
        rings_buffer[pl_in->points[i].ring]->back().normal_x = added_pt.x * added_pt.x + added_pt.y * added_pt.y;
    }
    lidar_begin_time = scan_time + pl_in->points.front().t / float(1000000);
    lidar_end_time = scan_time + pl_in->points.back().t / float(1000000);
    time_beg_buffer.push_back(lidar_begin_time);
    time_end_buffer.push_back(lidar_end_time);
}

void LiDARProcessing::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg){
    pl_surf->clear();
    pcl::PointCloud<VELOPoint::PointXYZIRT> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int pl_size = pl_orig.points.size();

    if (pl_size == 0) return;
    pl_surf->reserve(pl_size);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/
    if (pl_orig.points[pl_size - 1].timestamp > 0){
        given_offset_time = true;
    }else {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end  = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = pl_size - 1; i > 0; i--)
        {
            if (pl_orig.points[i].ring == layer_first)
            {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }
    for (int i = 0; i < pl_size; ++i){
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;

        V3D tmp_add{added_pt.x, added_pt.y, added_pt.z};
        V3D tmp_orig{pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z};
        tmp_add = ext_R_Lidar2Car.inverse() * tmp_orig - ext_t_Lidar2Car;

        added_pt.x = tmp_add.x();
        added_pt.y = tmp_add.y();
        added_pt.z = tmp_add.z();

        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) * float(1000);  // curvature unit: ms // cout<<added_pt.curvature<<endl;
        if (!given_offset_time){
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
            if (is_first[layer])
            {
                yaw_fp[layer]=yaw_angle;
                is_first[layer]=false;
                added_pt.curvature = 0.0;
                yaw_last[layer]=yaw_angle;
                time_last[layer]=added_pt.curvature;
                continue;
            }
            // compute offset time
            if (yaw_angle <= yaw_fp[layer]){
                added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
            }else {
                added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
            }
            if (added_pt.curvature < time_last[layer])  added_pt.curvature += 360.0/omega_l;
            yaw_last[layer] = yaw_angle;
            time_last[layer]=added_pt.curvature;
        }
        if(i % point_filter_num == 0 && added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind)){
            pl_surf->points.push_back(added_pt);
        }
    }
    lidar_begin_time = msg->header.stamp.toSec() * float(1000) + (pl_orig.points.front().timestamp - pl_orig.points[0].timestamp) * float(1000);
    lidar_end_time = msg->header.stamp.toSec() * float(1000) + (pl_orig.points.back().timestamp - pl_orig.points[0].timestamp) * float(1000);
    if(flg_first_scan){
        flg_first_scan = false;
        first_lidar_time = lidar_begin_time;
    }
    time_beg_buffer.push_back(lidar_begin_time);
    time_end_buffer.push_back(lidar_end_time);
}

void LiDARProcessing::rsm1_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<RSPoint::PointXYZIRT>::Ptr pl_in(new pcl::PointCloud<RSPoint::PointXYZIRT>());
    pcl::fromROSMsg(*msg, *pl_in);
    pl_in->is_dense = false;
    int pl_size = pl_in->points.size();
    for (int i = 0; i < pl_size; i += point_filter_num){
//        if(pl_in->points[i].ring % downsampleRate != 0) continue;
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        V3D tmp_add{added_pt.x, added_pt.y, added_pt.z};
        V3D tmp_orig{pl_in->points[i].x, pl_in->points[i].y, pl_in->points[i].z};
        tmp_add = ext_R_Lidar2Car.inverse() * tmp_orig - ext_t_Lidar2Car;

        added_pt.x = tmp_add.x();
        added_pt.y = tmp_add.y();
        added_pt.z = tmp_add.z();

        added_pt.intensity = pl_in->points[i].intensity;
        added_pt.curvature = (pl_in->points[i].timestamp - pl_in->points[0].timestamp) * float(1000);  // curvature unit: ms

        rings_buffer[pl_in->points[i].ring-1]->push_back(added_pt);
        rings_buffer[pl_in->points[i].ring-1]->back().normal_x = added_pt.x * added_pt.x + added_pt.y * added_pt.y;
    }
    lidar_begin_time = pl_in->points.front().timestamp * float(1000);
    lidar_end_time = pl_in->points.back().timestamp * float(1000);
    time_beg_buffer.push_back(lidar_begin_time);
    time_end_buffer.push_back(lidar_end_time);
}

void LiDARProcessing::rsmech_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<RSPoint::PointXYZIRT>::Ptr pl_in(new pcl::PointCloud<RSPoint::PointXYZIRT>());
    pcl::fromROSMsg(*msg, *pl_in);
    pl_in->is_dense = false;
    int pl_size = pl_in->points.size();

    for (int i = 0; i < pl_size; i += point_filter_num){
        if(pl_in->points[i].ring % downsampleRate != 0) continue;
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        V3D tmp_add{added_pt.x, added_pt.y, added_pt.z};
        V3D tmp_orig{pl_in->points[i].x, pl_in->points[i].y, pl_in->points[i].z};
        tmp_add = ext_R_Lidar2Car.inverse() * tmp_orig - ext_t_Lidar2Car;

        added_pt.x = tmp_add.x();
        added_pt.y = tmp_add.y();
        added_pt.z = tmp_add.z();

        added_pt.intensity = pl_in->points[i].intensity;
        added_pt.curvature = (pl_in->points[i].timestamp - pl_in->points[0].timestamp) * float(1000);  // curvature unit: ms

        rings_buffer[pl_in->points[i].ring]->push_back(added_pt);
        rings_buffer[pl_in->points[i].ring]->back().normal_x = added_pt.x * added_pt.x + added_pt.y * added_pt.y;
    }
    lidar_begin_time = pl_in->points.front().timestamp * float(1000);
    lidar_end_time = pl_in->points.back().timestamp * float(1000);
    time_beg_buffer.push_back(lidar_begin_time);
    time_end_buffer.push_back(lidar_end_time);
}
