#ifndef _LIDAR_PROCESSING_H
#define _LIDAR_PROCESSING_H

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_cloud.h>
#include "common_lib.h"
using namespace std;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

struct PointXYZIRT{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

namespace RSPoint{
    struct EIGEN_ALIGN16 PointXYZIRT{
        PCL_ADD_POINT4D;
        u_int8_t intensity;
        uint16_t ring = 0;
        double timestamp = 0;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}POINT_CLOUD_REGISTER_POINT_STRUCT(RSPoint::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(u_int8_t, intensity, intensity)
                                          (uint16_t, ring, ring)(double, timestamp, timestamp))

namespace VELOPoint {
    struct EIGEN_ALIGN16 PointXYZIRT {
        PCL_ADD_POINT4D;
        float intensity;
        float timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}POINT_CLOUD_REGISTER_POINT_STRUCT(VELOPoint::PointXYZIRT,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, intensity, intensity)
                                           (float, timestamp, time)
                                           (uint16_t, ring, ring)
)

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
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, intensity, intensity)
                                           // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                           (std::uint32_t, t, t)
                                           (std::uint16_t, reflectivity, reflectivity)
                                           (std::uint8_t, ring, ring)
                                           (std::uint16_t, ambient, ambient)
                                           (std::uint32_t, range, range)
)

template<typename T>
bool has_nan(T point){
    if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z)){
        return true;
    }else {
        return false;
    }
}

template<typename T_in_p, typename T_out_p>
void handle_pc_msg(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                   const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        T_out_p new_point;
        new_point.x = pc_in->points[point_id].x;
        new_point.y = pc_in->points[point_id].y;
        new_point.z = pc_in->points[point_id].z;
        new_point.intensity = pc_in->points[point_id].intensity;
        pc_out->points.push_back(new_point);
    }
}
template<typename T_in_p, typename T_out_p>
void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
    }
}

template<typename T_in_p, typename T_out_p>
void add_time(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        pc_out->points[valid_point_id++].timestamp = float(pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
    }
}

static pcl::PointCloud<PointType>::Ptr pl_full(new pcl::PointCloud<PointType>()),
pl_surf(new pcl::PointCloud<PointType>()),
pl_corn(new pcl::PointCloud<PointType>());

class LiDARProcessing{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LiDARProcessing();
    ~LiDARProcessing();
//    void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
    void process(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void setExtrinsic(const M3D &Lidar_R_wrt_Car, const V3D &Lidar_T_wrt_Car);
    double lidar_begin_time, lidar_end_time, first_lidar_time, last_timestamp_lidar;
    bool flg_first_scan;
    int point_filter_num, N_SCANS, SCAN_RATE;
    double blind;
    bool given_offset_time;
    uint lidar_count;
//    deque<PointCloudXYZI::Ptr>        lidar_buffer;

    deque<PointCloudXYZI::Ptr>        surf_buffer;
    deque<PointCloudXYZI::Ptr>        edge_buffer;

    deque<double>                     time_buffer;
    deque<double>                     time_beg_buffer, time_end_buffer;
    vector<PointCloudXYZI::Ptr> rings_buffer;
    int downsampleRate;
    M3D ext_R_Lidar2Car;
    V3D ext_t_Lidar2Car;
private:
    void avia_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void oust_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void rsm1_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void rsmech_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
//    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
};
#endif
