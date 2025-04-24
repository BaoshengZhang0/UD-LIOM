### /livox/imu
~~~C++
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg){
    sensor_msgs::Imu *msg_imu = new sensor_msgs::Imu();
    msg_imu->header.stamp = msg->header.stamp;
    msg_imu->header.frame_id = "init";
    //rad
    msg_imu->angular_velocity.x = -msg->angular_velocity.y;
    msg_imu->angular_velocity.y = msg->angular_velocity.x;
    msg_imu->angular_velocity.z = msg->angular_velocity.z;
    //Gm2
    msg_imu->linear_acceleration.x = -msg->linear_acceleration.y * G_m_s2;
    msg_imu->linear_acceleration.y = msg->linear_acceleration.x * G_m_s2;
    msg_imu->linear_acceleration.z = msg->linear_acceleration.z * G_m_s2;

    mtx_buffer.lock();
    msg_imus.push_back(msg_imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
~~~
### nclt dataset:/imu_raw
~~~C++
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg){
    sensor_msgs::Imu *msg_imu = new sensor_msgs::Imu();
    msg_imu->header.stamp = msg->header.stamp;
    msg_imu->header.frame_id = "init";
    //rad
    msg_imu->angular_velocity.x = msg->angular_velocity.x;
    msg_imu->angular_velocity.y = msg->angular_velocity.y;
    msg_imu->angular_velocity.z = msg->angular_velocity.z;
    //Gm2
    msg_imu->linear_acceleration.x = msg->linear_acceleration.x;
    msg_imu->linear_acceleration.y = msg->linear_acceleration.y;
    msg_imu->linear_acceleration.z = msg->linear_acceleration.z;

    mtx_buffer.lock();
    msg_imus.push_back(msg_imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
~~~
### TIERS dataset:/os_cloud_node/imu
~~~C++
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg){
    sensor_msgs::Imu *msg_imu = new sensor_msgs::Imu();
    msg_imu->header.stamp = msg->header.stamp;
    msg_imu->header.frame_id = "init";
    //rad
    msg_imu->angular_velocity.x = msg->angular_velocity.x;
    msg_imu->angular_velocity.y = msg->angular_velocity.y;
    msg_imu->angular_velocity.z = msg->angular_velocity.z;
    //Gm2
    msg_imu->linear_acceleration.x = msg->linear_acceleration.x;
    msg_imu->linear_acceleration.y = msg->linear_acceleration.y;
    msg_imu->linear_acceleration.z = msg->linear_acceleration.z;

    mtx_buffer.lock();
    msg_imus.push_back(msg_imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
~~~