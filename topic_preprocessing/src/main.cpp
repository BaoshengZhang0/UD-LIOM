#include "commlib.h"
mutex mtx_buffer;
condition_variable sig_buffer;

int lidar_type;
string lid_topic, imu_topic;

/**
 * Preprocess the point clouds of various LiDARs to a unified standard, and note that the "timestamp"[ms] of each point cloud is the relative time to the current scan.
 */
static deque<sensor_msgs::PointCloud2 *> msg_pcls;
void livox_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg){
    sensor_msgs::PointCloud2 *msg_pcl = new sensor_msgs::PointCloud2();
    pcl::PointCloud<PointXYZIRT>::Ptr pl_rec(new pcl::PointCloud<PointXYZIRT>());
    pcl::PointCloud<LIVOXPoint::PointXYZIRT>::Ptr pl_ori(new pcl::PointCloud<LIVOXPoint::PointXYZIRT>());
    pcl::fromROSMsg(*msg, *pl_ori);
    int pl_size = pl_ori->points.size();
    pl_rec->resize(pl_size);
    for(uint i = 0; i < pl_size; i++) {
        pl_rec->points[i].x = pl_ori->points[i].x;
        pl_rec->points[i].y = pl_ori->points[i].y;
        pl_rec->points[i].z = pl_ori->points[i].z;
        pl_rec->points[i].intensity = pl_ori->points[i].intensity;
        pl_rec->points[i].ring = pl_ori->points[i].ring;
//        pl_rec->points[i].timestamp = (pl_ori->points[i].timestamp - pl_ori->points.front().timestamp) * float(1000); // ms
        pl_rec->points[i].timestamp = pl_ori->points[i].timestamp; // ms
    }
    pcl::toROSMsg(*pl_rec, *msg_pcl);
    msg_pcl->header.frame_id = "sensor";
    msg_pcl->header.stamp = msg->header.stamp;
    mtx_buffer.lock();
    msg_pcls.push_back(msg_pcl);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void velo_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg){
    sensor_msgs::PointCloud2 *msg_pcl = new sensor_msgs::PointCloud2();
    pcl::PointCloud<PointXYZIRT>::Ptr pl_rec(new pcl::PointCloud<PointXYZIRT>());
    pcl::PointCloud<VELOPoint::PointXYZIRT>::Ptr pl_ori(new pcl::PointCloud<VELOPoint::PointXYZIRT>());
    pcl::fromROSMsg(*msg, *pl_ori);
    int pl_size = pl_ori->points.size();
    pl_rec->resize(pl_size);
    for(uint i = 0; i < pl_size; i++) {
        pl_rec->points[i].x = pl_ori->points[i].x;
        pl_rec->points[i].y = pl_ori->points[i].y;
        pl_rec->points[i].z = pl_ori->points[i].z;
        pl_rec->points[i].intensity = pl_ori->points[i].intensity;
        pl_rec->points[i].ring = pl_ori->points[i].ring;
//        pl_rec->points[i].timestamp = (pl_ori->points[i].timestamp - pl_ori->points.front().timestamp) * float(1000); // ms
        pl_rec->points[i].timestamp = pl_ori->points[i].timestamp; // ms
    }
    pcl::toROSMsg(*pl_rec, *msg_pcl);
    msg_pcl->header.frame_id = "sensor";
    msg_pcl->header.stamp = msg->header.stamp;
    mtx_buffer.lock();
    msg_pcls.push_back(msg_pcl);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void ouster_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg){
    sensor_msgs::PointCloud2 *msg_pcl = new sensor_msgs::PointCloud2();
    pcl::PointCloud<PointXYZIRT>::Ptr pl_rec(new pcl::PointCloud<PointXYZIRT>());
    pcl::PointCloud<OUSTPoint::PointXYZIRT>::Ptr pl_ori(new pcl::PointCloud<OUSTPoint::PointXYZIRT>());
    pcl::fromROSMsg(*msg, *pl_ori);
    int pl_size = pl_ori->points.size();
    pl_rec->resize(pl_size);
    for(uint i = 0; i < pl_size; i++) {
        pl_rec->points[i].x = pl_ori->points[i].x;
        pl_rec->points[i].y = pl_ori->points[i].y;
        pl_rec->points[i].z = pl_ori->points[i].z;
        pl_rec->points[i].ring = pl_ori->points[i].ring;
        pl_rec->points[i].intensity = pl_ori->points[i].intensity;
        pl_rec->points[i].timestamp = pl_ori->points[i].t / float(1000000); // ms
    }
    pcl::toROSMsg(*pl_rec, *msg_pcl);
    msg_pcl->header.frame_id = "sensor";
    msg_pcl->header.stamp = msg->header.stamp;
    mtx_buffer.lock();
    msg_pcls.push_back(msg_pcl);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void rs_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg){
    sensor_msgs::PointCloud2 *msg_pcl = new sensor_msgs::PointCloud2();
    pcl::PointCloud<PointXYZIRT>::Ptr pl_rec(new pcl::PointCloud<PointXYZIRT>());
    pcl::PointCloud<RSPoint::PointXYZIRT>::Ptr pl_ori(new pcl::PointCloud<RSPoint::PointXYZIRT>());
    pcl::fromROSMsg(*msg, *pl_ori);
    int pl_size = pl_ori->points.size();
    pl_rec->resize(pl_size);
    for(uint i = 0; i < pl_size; i++) {
        pl_rec->points[i].x = pl_ori->points[i].x;
        pl_rec->points[i].y = pl_ori->points[i].y;
        pl_rec->points[i].z = pl_ori->points[i].z;
        pl_rec->points[i].ring = pl_ori->points[i].ring;
        pl_rec->points[i].intensity = pl_ori->points[i].intensity;
        pl_rec->points[i].timestamp = (pl_ori->points[i].timestamp - pl_ori->points.front().timestamp) * float(1000); // ms
//        pl_rec->points[i].timestamp = pl_ori->points[i].timestamp; // ms
    }
    pcl::toROSMsg(*pl_rec, *msg_pcl);
    msg_pcl->header.frame_id = "sensor";
    msg_pcl->header.stamp = msg->header.stamp;
    mtx_buffer.lock();
    msg_pcls.push_back(msg_pcl);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

static deque<sensor_msgs::Imu *> msg_imus;
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg){
//    if((pub_imu_num++) % 3 != 0) return;
    sensor_msgs::Imu *msg_imu = new sensor_msgs::Imu();
    msg_imu->header.stamp = msg->header.stamp;
    msg_imu->header.frame_id = "sensor";
    //rad
    msg_imu->angular_velocity.x = msg->angular_velocity.x;
    msg_imu->angular_velocity.y = msg->angular_velocity.y;
    msg_imu->angular_velocity.z = msg->angular_velocity.z;
//    msg_imu->angular_velocity.x = -msg->angular_velocity.y;
//    msg_imu->angular_velocity.y = msg->angular_velocity.x;
//    msg_imu->angular_velocity.z = msg->angular_velocity.z;
    //Gm2
    msg_imu->linear_acceleration.x = msg->linear_acceleration.x;
    msg_imu->linear_acceleration.y = msg->linear_acceleration.y;
    msg_imu->linear_acceleration.z = msg->linear_acceleration.z;
//    msg_imu->linear_acceleration.x = -msg->linear_acceleration.y * G_m_s2;
//    msg_imu->linear_acceleration.y = msg->linear_acceleration.x * G_m_s2;
//    msg_imu->linear_acceleration.z = msg->linear_acceleration.z * G_m_s2;

    msg_imu->orientation.x = msg->orientation.x;
    msg_imu->orientation.y = msg->orientation.y;
    msg_imu->orientation.z = msg->orientation.z;
    msg_imu->orientation.w = msg->orientation.w;

    mtx_buffer.lock();
    msg_imus.push_back(msg_imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void paramsGetting(ros::NodeHandle &nh){
    nh.param<string>("common/lid_topic", lid_topic, "/lidar_points");
    nh.param<string>("common/imu_topic", imu_topic, "/imu_data");
    nh.param<int>("preprocess/lidar_type", lidar_type, RSM1);
}

int main(int argc,char **argv){
    ros::init(argc,argv,"topic_preprocessing");
    ros::NodeHandle nh;
    paramsGetting(nh);
    ros::Subscriber sub_pcl;
    switch (lidar_type) {
        case RSM1:
            sub_pcl = nh.subscribe(lid_topic, 10000, rs_pcl_cbk);
            break;
        case LIVOX:
            sub_pcl = nh.subscribe(lid_topic, 10000, livox_pcl_cbk);
            break;
        case VELO:
            sub_pcl = nh.subscribe(lid_topic, 10000, velo_pcl_cbk);
            break;
        case OUSTER:
            sub_pcl = nh.subscribe(lid_topic, 10000, ouster_pcl_cbk);
            break;
        case RSMECH:
            sub_pcl = nh.subscribe(lid_topic, 10000, rs_pcl_cbk);
            break;
    }
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 10000, imu_cbk);

    ros::Publisher pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/LIDAR_POINTS", 10000);
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/IMU", 10000);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    while(ros::ok()){
        ros::spinOnce();
        //LiDAR
        if(msg_pcls.size() > 2) {
            mtx_buffer.lock();
            pub_pcl.publish(*msg_pcls.front());
            delete msg_pcls.front();
            msg_pcls.pop_front();
            mtx_buffer.unlock();
            sig_buffer.notify_all();
        }
        //IMU
        if(!msg_imus.empty()) {
            mtx_buffer.lock();
            pub_imu.publish(*msg_imus.front());
            delete msg_imus.front();
            msg_imus.pop_front();
            mtx_buffer.unlock();
            sig_buffer.notify_all();
        }
    }
    spinner.stop();
    return 0;
}
