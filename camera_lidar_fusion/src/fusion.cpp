#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>
#include <boost/bind.hpp>
#include <opencv2/core/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <cmath>
#include <hash_map>
#include <ctime>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include "common.h"
#include <chrono>
#include <vector>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

//定义全局点云,并初始化
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_color_cloud_DS (new pcl::PointCloud<pcl::PointXYZRGB>());
vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> global_color_cloud_vector;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn);
void odomCallback (const nav_msgs::Odometry::ConstPtr& msg);

class CameraLidarFusion{
    typedef pcl::PointXYZRGB PointT;
public:
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_;
    string camera_topic_, lidar_topic_, intrinsic_path_, extrinsic_path_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> camera_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    ros::Publisher colored_cloud_pub_;
    
    //获取机器人位姿
    // ros::Subscriber odom_sub_;
    // nav_msgs::Odometry odom_;

    //发布有色点云
    //   ros::Publisher color_cloud_global  =  node_handle_.advertise<sensor_msgs::PointCloud2>("/color_cloud_global", 10);
        //  ros::Time timeLaserInfoStamp;
        
    CameraLidarFusion();
    ~CameraLidarFusion();   
    
private:
    //  sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<pcl::PointXYZRGB> thisCloud, ros::Time thisStamp, std::string thisFrame);
   
    void callback(const sensor_msgs::CompressedImageConstPtr& input_image_msg, const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg);
    void getUV(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, float* UV);
    void getColor(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int* RGB);
    
};

//发布上色点云
 //重新发布上色点云
// ros::Publisher color_cloud_pub;
//  color_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/colored_cloud", 1);



CameraLidarFusion::CameraLidarFusion() : private_node_("~") {
    // cout << "\033[1;32m Get the parameters from the launch file \033[0m" << endl;
    private_node_.getParam("camera_topic", camera_topic_);
    private_node_.getParam("lidar_topic", lidar_topic_);
    private_node_.getParam("intrinsic_path", intrinsic_path_);
    private_node_.getParam("extrinsic_path", extrinsic_path_);

    camera_sub_.subscribe(node_handle_, camera_topic_, 15);
    lidar_sub_.subscribe(node_handle_, lidar_topic_, 10);
    colored_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("colored_cloud_toshow", 10);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2>MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera_sub_, lidar_sub_);

    sync.registerCallback(boost::bind(&CameraLidarFusion::callback, this, _1, _2));


    ros::spin();

}

CameraLidarFusion::~CameraLidarFusion() {

}

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::PointXYZRGB *pointFrom;

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = Eigen::Affine3f::Identity();
        
        for (int i = 0; i < cloudSize; ++i)
        {
            pointFrom = &cloudIn->points[i];
            if ( pointFrom->x == 0 && pointFrom->y == 0 && pointFrom->z == 0&&(  pointFrom->r == 0 && pointFrom->g == 0 && pointFrom->b == 0))
            {
                continue;
            }
            cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            cloudOut->points[i].r = pointFrom->r;
            cloudOut->points[i].g = pointFrom->g;
            cloudOut->points[i].b = pointFrom->b;

        }
        return cloudOut;
    }

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
    {
        
    }
 
void CameraLidarFusion::callback(const sensor_msgs::CompressedImageConstPtr& input_image_msg, const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) {
    //获得相机内参
    vector<float> intrinsic;
    getIntrinsic(intrinsic_path_, intrinsic);
    //获得相机畸变参数
    // vector<float> distortion;
    // getDistortion(intrinsic_path_, distortion);
    //获得相机外参
    vector<float> extrinsic;
    getExtrinsic(extrinsic_path_, extrinsic);


    double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]}, {intrinsic[3], intrinsic[4], intrinsic[5]}, {intrinsic[6], intrinsic[7], intrinsic[8]}};
    double matrix2[3][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]}, {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]}, {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}};

    cv::Mat matrix_in(3, 3, CV_64F, matrix1);
    cv::Mat matrix_out(3, 4, CV_64F, matrix2);

    cv::Mat camera_matrix = cv::Mat::eye(3, 3 ,CV_64F);
    camera_matrix.at<double>(0, 0) = intrinsic[0];
    camera_matrix.at<double>(0, 1) = intrinsic[1];
    camera_matrix.at<double>(0, 2) = intrinsic[2];
    camera_matrix.at<double>(1, 0) = intrinsic[3];
    camera_matrix.at<double>(1, 1) = intrinsic[4];
    camera_matrix.at<double>(1, 2) = intrinsic[5];
    camera_matrix.at<double>(2, 0) = intrinsic[6];
    camera_matrix.at<double>(2, 1) = intrinsic[7];
    camera_matrix.at<double>(2, 2) = intrinsic[8];

    // cv::Mat distortion_coef = cv::Mat::zeros(5, 1, CV_64F);
    // distortion_coef.at<double>(0, 0) = distortion[0];
    // distortion_coef.at<double>(1, 0) = distortion[1];
    // distortion_coef.at<double>(2, 0) = distortion[2];
    // distortion_coef.at<double>(3, 0) = distortion[3];
    // distortion_coef.at<double>(4, 0) = distortion[4];

    cv::Mat input_image;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
        return;
    }
    input_image = cv_ptr->image;

    //  cv::Mat view, rview, map1, map2;
    //  cv::Size imageSize = input_image.size();
    //  cv::Mat NewCameraMatrix =getOptimalNewCameraMatrix(camera_matrix, distortion_coef, imageSize, 1, imageSize, 0);
    //  cv::initUndistortRectifyMap(camera_matrix, distortion_coef, cv::Mat(),  NewCameraMatrix  , imageSize, CV_16SC2, map1, map2);
    //   cv::remap(input_image, input_image, map1, map2, cv::INTER_LINEAR);  // correct the distortion


    int row = input_image.rows;
    int col = input_image.cols;
    //用vector定义一个二维数组
    vector<vector<int>> color_vector;
    color_vector.resize(row*col);
    for (unsigned int i = 0; i < color_vector.size(); ++i) {
        color_vector[i].resize(3);
    }
    for (int v = 0; v < row; ++v) {
        for (int u = 0; u < col; ++u) {
           
            color_vector[v*col + u][0] = input_image.at<cv::Vec3b>(v, u)[2];
            color_vector[v*col + u][1] = input_image.at<cv::Vec3b>(v, u)[1];
            color_vector[v*col + u][2] = input_image.at<cv::Vec3b>(v, u)[0];
        }
    }
    // cv::imshow("input_image", input_image);
    // cv::waitKey(1);
    // ros::spin();

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*input_cloud_msg, *output_cloud_msg);//very important
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = input_cloud_msg->width ;
    cloud->points.resize(cloud->width);
    
    // double w_sigma = 0.07;
    
    cv::RNG rng;
    for(uint64_t i = 0; i < cloud->points.size(); ++i){
        float x = output_cloud_msg->points[i].x;
        float y = output_cloud_msg->points[i].y;
        float z = output_cloud_msg->points[i].z;
        if(x == 0 && y == 0 && z == 0) {
            continue;
        }
        cloud->points[i].x = x;
        cloud->points[i].y = y;
        cloud->points[i].z = z;
        int RGB[3] = {0, 0, 0};

        // cout << "x: " << x << " y: " << y << " z: " << z << endl;
        
        getColor(matrix_in, matrix_out, x, y, z, row, col, color_vector, RGB);
        // ROS_ERROR("RGB: %d %d %d", RGB[0], RGB[1], RGB[2]);

        if (RGB[0] == 0 && RGB[1] == 0 && RGB[2] == 0) {
            continue;
        }
        cloud->points[i].r = RGB[0];
        cloud->points[i].g = RGB[1];
        cloud->points[i].b = RGB[2];

     
     
    }


    //********************
        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*cloud, output_cloud);
        output_cloud.header = input_cloud_msg->header;
        output_cloud.header.frame_id = input_cloud_msg->header.frame_id;
        colored_cloud_pub_.publish(output_cloud);
    //********************
       
    //     sensor_msgs::PointCloud2 global_output; 
    //     //将global_color_cloud放入容器中
    //      global_color_cloud_vector.push_back(cloud);
    //     //遍历vector中的点云
    //     for (int i = 0; i < global_color_cloud_vector.size(); i++) {
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp(new pcl::PointCloud<pcl::PointXYZRGB>());
    //         //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp2(new pcl::PointCloud<pcl::PointXYZRGB>());
    //         *Temp = *transformPointCloud(global_color_cloud_vector[i]);
    //          *global_color_cloud += *Temp;  
    //         }

    //   // publishCloud(&color_cloud_global, global_color_cloud, timeLaserInfoStamp, "/color_cloud_global"); 
    // // sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<pcl::PointXYZRGB> thisCloud, ros::Time thisStamp, std::string thisFrame)
    //          pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterGlobalMapKeyFrames; // for global map visualization
    //      // globalMapVisualizationLeafSize 1m
    //      float globalMapVisualizationLeafSize = 0.4;
    //         downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
    //      downSizeFilterGlobalMapKeyFrames.setInputCloud(global_color_cloud);
    //          downSizeFilterGlobalMapKeyFrames.filter(*global_color_cloud_DS);



    //         ////将容器中的点云转换为sensor_msgs::PointCloud2类型,并发布
    //         pcl::toROSMsg(*global_color_cloud_DS, global_output);
    //          global_output.header.stamp =input_cloud_msg->header.stamp;
    //          global_output.header.frame_id = input_cloud_msg->header.frame_id;
    //          color_cloud_global.publish(global_output);
}


//输入 ：内参 ， 外参 ，x ,y , z ,uv坐标
void CameraLidarFusion::getUV(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, float* UV){
    double matrix3[4][1] = {x, y, z, 1};
    cv::Mat coordinate(4, 1, CV_64F, matrix3);
    
    cv::Mat result = matrix_in*matrix_out*coordinate;


    // cout<<"matrix_in:"<<matrix_in<<endl;
    // cout<<"matrix_out:"<<matrix_out<<endl;
    
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);
    UV[0] = u / depth;
    UV[1] = v / depth;
    UV[2] = depth;

    // cout << "u: " << UV[0] << " v: " << UV[1] << endl;

}
//输入：内参，外参 ，x, y ,z ,图像的行数，列数，图像的颜色数组，RGB数组
void CameraLidarFusion::getColor(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int* RGB) {
    float UV[3] = {0, 0, 0};
    getUV(matrix_in, matrix_out, x, y, z, UV);
    int u = int(UV[0]);
    int v = int(UV[1]);
    int depth = int(UV[2]);
    // cout << "u: " << u << " v: " << v << endl;
    int32_t index = v*col + u;
    // if (index < row*col && index >= 0) {
    //     RGB[0] = color_vector[index][0];
    //     RGB[1] = color_vector[index][1];
    //     RGB[2] = color_vector[index][2];
    // }

    if ((u<0) ||(u >col -1) ||(v < 0 )||(v > row -1)|| (depth<0)  ){
        RGB[0] = 0; 
        RGB[1] = 0;
        RGB[2] = 0;
    }
    else {
        // cout << "u: " << u << " v: " << v << endl;
        RGB[0] = color_vector[index][0];
        RGB[1] =  color_vector[index][1];
        RGB[2] =  color_vector[index][2];
    }
    //  cout << "u: " << u << " v: " << v << endl;
    //  cout <<"index"<<index<<endl;
    //   cout << "r: " << RGB[0] << " g: " << RGB[1] << " b: " << RGB[2] << endl;
    

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "fusion");
    ros::NodeHandle nh;


    CameraLidarFusion cameraLidarFusion;

    return 0;
}
