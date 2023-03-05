#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;

// global variable saving the lidar odometry
deque<nav_msgs::Odometry> odomQueue;
odometryRegister *odomRegister;

std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;
std::mutex m_odom;

// imu相关全局变量
double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;


//***********************
    double lambda_IMU  =0;//IMU因子权值
    double lambda_camera =0;//相机因子权值
    //定义一个标志位，用于判断是否重启VIO
    bool restart_vio = false;
    double w_fecture =1;//特征跟踪性能权值1
    double w_motion=1;//线速度性能权值1
    double w_angle=1;//角速度性能权值1
    double Scvalue;//计算S值
    
    double S_norm = 0;//S值归一化
    std::vector<int> num_tracked_all;//每帧跟踪成功的特征点数
    float  average_track = 0;//平均每个特征点跟踪的次数


//**************
//创建容器保存每帧的跟踪数目
std::vector<float> tmp_track_num_sliding;;
std::ofstream Estimator_result;
std::ofstream S_result;

struct ImageStatus
{
    double time;
    Vector3d P;
    Quaterniond Q;
    Vector3d V;
    Vector3d Ba;
    Vector3d Bg;
    Vector3d acc_0;
    Vector3d gyr_0;
};

std::vector<ImageStatus> image_status_vector;





// 从当前的imu测量值和上一时刻的PQV进行递推得到当前时刻的PQV
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    // 如果是第一帧imu数据则不进行处理
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    // 计算当前imu_msg 距离上一个时刻之间的时间间隔
    double dt = t - latest_time;
    latest_time = t;
    
    // 取出imu_msg 中的数据
    // 加速度
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};
    // 角速度
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};
    
    // 中值积分计算PVQ
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt); // 四元数中值积分

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

    //从处理完的从估计器中得到滑动窗口中最后一个图像帧的imu更新项[P,Q,V,ba,bg,a,g]

// 保存状态估计信息
ImageStatus image_status;
image_status.time = latest_time;
image_status.P = tmp_P;
image_status.Q = tmp_Q;
image_status.V = tmp_V;
image_status.Ba = tmp_Ba;
image_status.Bg = tmp_Bg;
image_status.acc_0 = acc_0;
image_status.gyr_0 = gyr_0;
image_status_vector.push_back(image_status);



}

// 对imu数据和图像数据进行时间对齐
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (ros::ok())
    {
        // 如果两个buf有一个为空就之间返回
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;
        // imu 太慢，等imu数据
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            return measurements;
        }
        // 图像太老，扔掉图像
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        //这里把下一个imu_msg也放进去了,但没有pop，因此当前图像帧和下一图像帧会共用这个imu_msg
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

// imu回调函数，将imu_msg 保存到 imu_buf，并且将IMU进行状态递推并发布
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // 对imu数据进行检查（imu数据应该按时间序列的顺序被订阅）
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg); // 将imu数据放到imu_buf里
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);// 递推得到IMU的当前时刻的PQV（初值）
        std_msgs::Header header = imu_msg->header;
        // 发布最新的由IMU直接递推得到的PQV （用于RVIZ可视化）
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header, estimator.failureCount);
    }
}

/**
 * @brief 从激光惯性子系统获取的相关参数
 * 
 * @param odom_msg 
 */
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    m_odom.lock();
    odomQueue.push_back(*odom_msg);
    m_odom.unlock();
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    
    
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// thread: visual-inertial odometry
void process()
{
    // 该线程在这里不断进行循环
    while (ros::ok())
    {
        // 定义一个measurements vector，一个图像帧对应多个数据
        // 1. 获取对其时间后的measurements数据
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        //等待上面两个接收数据完成就会被唤醒
        //在提取measurements时互斥锁m_buf会锁住，此时无法接收数据
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();

        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;

            // 2. 进行 IMU 预积分
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            // 遍历当前图像帧对应的多个imu数据
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                { 
                    //2.1 对于比图像时间早的imu数据
                    if (current_time < 0) 
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    // 对imu数据进行预积分
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    //2.2 对于最后一个imu数据（时间不是严格小于图像时间），做插值处理
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    // 插值得到图像时刻的imu数据
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // 2. VINS Optimization
            // TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                double depth = img_msg->channels[5].values[i];

                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
            }

            // Get initialization info from lidar odometry
            vector<float> initialization_info;
            m_odom.lock();
            initialization_info = odomRegister->getOdometry(odomQueue, img_msg->header.stamp.toSec() + estimator.td);
            m_odom.unlock();


            estimator.processImage(image, initialization_info, img_msg->header);
            // double whole_t = t_s.toc();
            // printStatistics(estimator, whole_t);

            // 3. Visualization
            std_msgs::Header header = img_msg->header;
            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
        }
        m_estimator.unlock();

        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

void track_num_callback( const std_msgs::Float32ConstPtr& msg )
{
    Estimator_result.open("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/visual_odometry/Estimator_result.txt",ofstream::out | ofstream::app);
    S_result.open("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/visual_odometry/S_result.txt",ofstream::out | ofstream::app);
    
    if (  msg->data ==0)
            {ROS_ERROR("track_num is empty!");
                return;}
    tmp_track_num_sliding.reserve(1000);
    float track_num = msg->data;//当前帧跟踪到的特征点数
     num_tracked_all.push_back(track_num);
    if (num_tracked_all.size()>11){
        //弹出最前面的元素
        num_tracked_all.erase(num_tracked_all.begin());
    }
    else{
        ROS_INFO("num_tracked_all.size()<=10");
        return;
    }
    //输出一个窗口内最大值
    double max_track_num = *max_element(num_tracked_all.begin(),num_tracked_all.end());
    // cout<<"max_track_num: "<<max_track_num<<endl;
    //cout << "track_num: " << track_num << endl;
    average_track = track_num/150;
  
    Estimator_result<<track_num<<"    ";
    Estimator_result<<average_track<<"   ";


if (image_status_vector.size() > 1){
      // 读取当前和上一张图像的位置信息
    Vector3d current_P = image_status_vector.end()->P;
    Vector3d prev_P = (image_status_vector.end() - 1)->P;
    double delta_x = current_P.x() - prev_P.x();
    double delta_y = current_P.y() - prev_P.y();
    //机器人在水平面运动，只计算yaw角
Quaterniond current_Q = image_status_vector.end()->Q;
Quaterniond prev_Q = (image_status_vector.end() - 1)->Q;
Matrix3d current_R = current_Q.toRotationMatrix();
Matrix3d prev_R = prev_Q.toRotationMatrix();
Matrix3d R = current_R.transpose() * prev_R; // 当前机器人的旋转矩阵
Vector3d direction = R * Vector3d::UnitX(); // 当前机器人的方向向量
double yaw = atan2(direction.y(), direction.x()); // 计算yaw角

//计算Si
Scvalue = w_fecture*average_track - w_motion*sqrt(delta_x*delta_x + delta_y*delta_y) - w_angle*abs(yaw);
// cout<<"yaw: "<<yaw<<endl;
if (sqrt(delta_x*delta_x + delta_y*delta_y) >10){
    ROS_ERROR("sqrt(delta_x*delta_x + delta_y*delta_y) >10");
restart_vio = true;
}
if (abs(yaw) > 5)
{
    ROS_ERROR("abs(yaw) > 5");
    restart_vio = true;
}

Estimator_result<<delta_x<<"   ";
Estimator_result<<delta_y<<"   ";
Estimator_result<<yaw<<"   ";
Estimator_result<<sqrt(delta_x*delta_x + delta_y*delta_y)<<endl;

cout<<"average_track: "<<average_track<<endl;
cout<<"w_motion*sqrt(delta_x*delta_x + delta_y*delta_y)"<<w_motion*sqrt(delta_x*delta_x + delta_y*delta_y)<<endl;
cout<<"w_angle*abs(yaw)"<<w_angle*abs(yaw)<<endl;


}
else{
   ROS_ERROR("image_status_vector.size() <= 1");
   S_norm = 1;
   Scvalue = 0;
    S_result<<Scvalue<<"    ";
    S_result<<S_norm<<endl;
   return;
}

S_norm = Scvalue /( max_track_num/150);//归一化评分
// cout<<"max_track_num: "<<max_track_num<<endl;

cout<<"Scvalue: "<<Scvalue<<endl;
cout<<"S_norm: "<<S_norm<<endl;
S_result<<Scvalue<<"    ";
S_result<<S_norm<<endl;


S_result.close();
Estimator_result.close();
if (restart_vio)
{
    ROS_ERROR("S_norm < 0");
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
        restart_vio = false;
   return;
}
 









}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Odometry Estimator Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    readParameters(n);
    estimator.setParameter();

    registerPub(n);

    odomRegister = new odometryRegister(n);

    std::ofstream Estimator_result("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/visual_odometry/Estimator_result.txt", ofstream::trunc);
    std::ofstream S_result("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/visual_odometry/S_result.txt", ofstream::trunc);
    ros::Subscriber sub_imu     = n.subscribe(IMU_TOPIC,      5000, imu_callback,  ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_odom    = n.subscribe("odometry/imu", 5000, odom_callback);
    ros::Subscriber sub_image   = n.subscribe(PROJECT_NAME + "/vins/feature/feature", 1, feature_callback);
    ros::Subscriber sub_restart = n.subscribe(PROJECT_NAME + "/vins/feature/restart", 1, restart_callback);
    ros::Subscriber sub_track_num = n.subscribe<std_msgs::Float32>("track_topic", 1, track_num_callback);
    
    
    // TODO：对比vins-mono 多了一个 sub_odom (由LIO系统传递来)，少了一个 sub_relo_points (需要进一步学习)
    if (!USE_LIDAR)
        sub_odom.shutdown();
    
    // estimator的主线程
    std::thread measurement_process{process};

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}