#include "ros/ros.h"
#include "std_msgs/String.h"
#include<string>
#include<iostream>
#include<fstream>

#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>

#include "geometry_msgs/Twist.h"
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>


#include<tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件


#include<eigen3/Eigen/Eigen>
#include<sophus/se3.hpp>

// #include "MeasureAlgoTime.hpp"

using namespace std;
using namespace Sophus;

nav_msgs::Path  path;
ros::Publisher  path_pub;
ofstream foutC;
ofstream foutB;
//建立一个该消息体类型的变量，用于存储订阅的信息
nav_msgs::Path current_pose;


//预先输入真实值轨迹并和接受的轨迹进行比较，计算出误差

/*

// 更高精度的轨迹 作为你真实轨迹
string groundtruth_file = "/home/projects/sophus/trajectoryError/groundtruth.txt";
// 算法计算出来的轨迹
string estimated_file = "/home/projects/sophus/trajectoryError/estimated.txt";
// Twc 的平移部分构成了机器人的轨迹
// aligned_allocator管理C++中的各种数据类型的内存方法是一样的
// 在C++11标准中,一般情况下定义容器的元素都是C++中的类型,
// 在Eigen管理内存和C++11中的方法不一样,需要单独强调元素的内存分配和管理

typedef vector<:se3d eigen::aligned_allocator>> TrajectoryType;
TrajectoryType ReadTrajectory(const string &path);
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);


int main(int argc, char **argv)
{
        TrajectoryType groundtruth = ReadTrajectory(groundtruth_file);
        TrajectoryType estimated = ReadTrajectory(estimated_file);
        assert(!groundtruth.empty() && !estimated.empty());
        assert(groundtruth.size() == estimated.size());
        // compute rmse 位姿的均方根误差
       // ATE
     double rmse = 0;
      for (size_t i = 0; i < estimated.size(); i++) {
           Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
            // 李群SE3的对数映射求李代数se3
              // 对应视觉SLAM十四讲第二版p89 公式4.44
               // .norm();代表二范数的计算过程
                 double error = (p2.inverse() * p1).log().norm();
                   rmse += error * error;  }
         rmse = rmse / double(estimated.size());
          rmse = sqrt(rmse);  cout << "RMSE = " << rmse << endl;
          DrawTrajectory(groundtruth, estimated);  return 0;}

TrajectoryType ReadTrajectory(const string &path)
{
    ifstream fin(path);
      TrajectoryType trajectory;
        if (!fin)
         {    cerr << "trajectory " << path << " not found." << endl;
           return trajectory;  }
        while (!fin.eof())
        {
            double time, tx, ty, tz, qx, qy, qz, qw;
             // tx  ty  tz 为Twc的平移部分
               // qx  qy  qz  qw 是四元数表示的 Twc的旋转部分 qw 是四元数的实部
                fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
                   // 四元数和平移向量构造李群SE3变换矩阵
                      Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
                        trajectory.push_back(p1);
         }
return trajectory;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti)
{
 // create pangolin window and plot the trajectory
 pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
glEnable(GL_DEPTH_TEST);  glEnable(GL_BLEND);
glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
pangolin::OpenGlRenderState s_cam( pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)  );
                               pangolin::View &d_cam = pangolin::CreateDisplay()      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)      .SetHandler(new pangolin::Handler3D(s_cam));
                                while (pangolin::ShouldQuit() == false)
                                 {
                                     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                                       d_cam.Activate(s_cam);
                                           glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
                                           glLineWidth(2);
                                             for (size_t i = 0; i < gt.size() - 1; i++)
                                              {
                                                   glColor3f(0.0f, 0.0f, 1.0f);
                                                    // blue for ground truth
                                                      glBegin(GL_LINES);
                                                       // 轨迹就是平移向量
                                                         auto p1 = gt[i], p2 = gt[i + 1];
                                                          glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                                                             glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                                                                glEnd();
                                                                  }
                                             for (size_t i = 0; i < esti.size() - 1; i++)
                                              {
                                                    glColor3f(1.0f, 0.0f, 0.0f);
                                                      // red for estimated
                                                        glBegin(GL_LINES);
                                                            auto p1 = esti[i], p2 = esti[i + 1];
                                                             glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                                                                 glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                                                                      glEnd();
                                                                      }
                                                                       pangolin::FinishFrame();
                                                     usleep(5000);   // sleep 5 ms
                                                       }}





*/

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    //2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"trajectory_sub_test");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;

            float x = 0;
            float y =0;
            float z = 0;
            float qx =0;
            float qy = 0;
            float qz = 0;
            float qw = 0;

            float x_last = 0;
            float y_last =0;
            float z_last = 0;
            float qx_last =0;
            float qy_last = 0;
            float qz_last = 0;
            float qw_last = 0;

          int count_end =0;

    fstream file("./src/trajectory_output/src/result/traj", ios::out);

    foutC.open("./src/trajectory_output/src/result/traj");

    // fstream time_frames("./src/trajectory_output/src/time_frames.txt", ios::out); 
    // foutB.open("./src/trajectory_output/src/time_frames.txt");

    tf::TransformListener listener;
    ros::Rate rate_10hz(20);

    while(ros::ok())
    {
         tf::StampedTransform transform;
         try
         {
             listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
             listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
             foutC << transform.stamp_ << " ";

            x = transform.getOrigin().getX();
            y = transform.getOrigin().getY();
            z = transform.getOrigin().getZ();
            qx = transform.getRotation().getX();
            qy = transform.getRotation().getY();
             qz = transform.getRotation().getZ();
             qw = transform.getRotation().getW();
            ROS_INFO("%f %f %f %f %f %f %f",x,y,z,qx,qy,qz,qw);
            foutC << x <<" " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;

          
          if ( x_last== x && y_last == y &&z_last==z&&qx_last==qx &&qy_last==qy&&qz_last==qz &&qw_last==qw)
          {
              count_end ++;
             if (count_end ==50) break;
          }
          else {
           x_last=x;
            y_last=y;
            z_last=z;
           qx_last=qx;
            qy_last=qy;
            qz_last=qz;
            qw_last = qw;
            // foutB<<count_end*0.1<<endl;
            count_end= 0;
          }



         }
         catch(const std::exception& e)
         {
             std::cerr << e.what() << '\n';
         }
         

        
        rate_10hz.sleep();
    }
    foutB.close();
    foutC.close();
    return 0;
}
 







    
