#include "preprocess.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "ud_liom");
    ros::NodeHandle nh;
    paramsGetting(nh);
    initSetting();
    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = nh.subscribe("/LIDAR_POINTS", 1000, pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe("/IMU", 1000, imu_cbk);
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100);
    ros::Publisher pubPoint = nh.advertise<sensor_msgs::PointCloud2>("/point_world", 100);
    tf::Transform TFtransform;
    tf::TransformBroadcaster TFbr;
    msg_body_pose.pose.position.x = 0.0;
    msg_body_pose.pose.position.y = 0.0;
    msg_body_pose.pose.position.z = 0.0;
    msg_body_pose.pose.orientation.x = 0.0;
    msg_body_pose.pose.orientation.y = 0.0;
    msg_body_pose.pose.orientation.z = 0.0;
    msg_body_pose.pose.orientation.w = 1.0;
    fusion_back->normvec = normvec;
    fusion_back->laserCloudOri = laserCloudOri;
    fusion_back->corr_normvect = corr_normvect;
    fusion_back->endpointa = endpointa;
    fusion_back->endpointb = endpointb;
    fusion_back->endpoint_a = endpoint_a;
    fusion_back->endpoint_b = endpoint_b;

    p_lidar->rings_buffer.resize(p_lidar->N_SCANS);
    for(auto &ring : p_lidar->rings_buffer){
        PointCloudXYZI::Ptr ptr_ring(new PointCloudXYZI());
        ring = ptr_ring;
    }
    thread th0(sync_packages);
    th0.detach();
    thread th1(estimating, ref(pubPoint), ref(pubPath));
    th1.detach();
    signal(SIGINT, sigHandle);
    ros::Rate rate(1000);
    while (ros::ok()){
        if (flg_exit) {
            break;
        }
        ros::spinOnce();
        if(!flg_inited){
            flg_inited = ((fusion_back->imu_enable & p_imu->flagImuInit) | !fusion_back->imu_enable);
        }
        if(!DeMeasures.empty() && preprocess_surf->empty() && preprocess_edge->empty() && flg_inited){
            mtx_buffer.lock();
            MeasuresCur = *(DeMeasures.front());
            delete DeMeasures.front();
            DeMeasures.pop_front();
            if(DeMeasures.size() > 0){
            }
            if(fusion_back->imu_enable){
                p_imu->Process(MeasuresCur, undistort_surf, undistort_edge);
            }else{
                pc_process(MeasuresCur, undistort_surf, undistort_edge);
            }
            if (undistort_surf->empty() || (undistort_surf == NULL)){
                ROS_WARN("No surf point, skip this scan!");
            }
            if (undistort_edge->empty() || (undistort_edge == NULL)){
                ROS_WARN("No edge point, skip this scan!");
            }
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();
            /*** downsample the feature points in a scan ***/
            downSizeFilterScanVox.setInputCloud(undistort_surf);
            downSizeFilterScanVox.filter(*preprocess_surf);
            downSizeFilterScanVox.setInputCloud(undistort_edge);
            downSizeFilterScanVox.filter(*preprocess_edge);
            size_pre_surf = preprocess_surf->points.size();
            size_pre_edge = preprocess_edge->points.size();
            /*** initialize the map kdtree ***/
            if(fusion_back->ikdtree_surf.Root_Node == nullptr){
                if(size_pre_surf > 5){
                    fusion_back->ikdtree_surf.set_downsample_param(filter_size_map_min);
                    down_world_surf->resize(size_pre_surf);
                    for(int i = 0; i < size_pre_surf; i++){
                        pointBodyToWorld(&(preprocess_surf->points[i]), &(down_world_surf->points[i]));
                    }
                    fusion_back->ikdtree_surf.Build(down_world_surf->points);
                }
                preprocess_surf->clear();
            }
            if(fusion_back->ikdtree_edge.Root_Node == nullptr){
                if(size_pre_surf > 5){
                    fusion_back->ikdtree_edge.set_downsample_param(filter_size_map_min);
                    down_world_edge->resize(size_pre_surf);
                    for(int i = 0; i < size_pre_surf; i++){
                        pointBodyToWorld(&(preprocess_edge->points[i]), &(down_world_edge->points[i]));
                    }
                    fusion_back->ikdtree_edge.Build(down_world_edge->points);
                }
                preprocess_edge->clear();
            }
            //follow view
            TFtransform.setOrigin( tf::Vector3(msg_body_pose.pose.position.x, msg_body_pose.pose.position.y, msg_body_pose.pose.position.z));
            TFtransform.setRotation( tf::Quaternion(msg_body_pose.pose.orientation.x,msg_body_pose.pose.orientation.y ,msg_body_pose.pose.orientation.z, msg_body_pose.pose.orientation.w) );
            TFbr.sendTransform(tf::StampedTransform(TFtransform, ros::Time::now(), "ud_liom", "view"));
            mtx_buffer.unlock();
            sig_buffer.notify_all();
        }
        rate.sleep();
    }
    spinner.stop();
    th0.join();
    th1.join();
    ros::waitForShutdown();
    return 0;
}