#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
// 是在使用函数模板时，添加一个宏定义#define PCL_NO_PRECOMPILE在调用函数模板之前
#include <iostream>
#include<bits/stdc++.h>
#include <ros/ros.h>
#include <deque>
#include <mutex>
#include <thread>
#include <ros/time.h>
#include <sstream> 
#include "std_msgs/String.h" 
#include <std_msgs/Float64.h>
// pcl格式头文件
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/flann.h>
#include <flann/flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/kdtree_flann.h>

//OPENCV格式头文件
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

//#include "flann/flann.cpp"
// sensor_msgs格式头文件
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
//数学函数库头文件
#include <cmath>
#include <iomanip>

//时间头文件
#include <stdio.h>
#include<chrono>
#include "MeasureAlgoTime.hpp"
//自定义消息格式
#include "trajectory_output/trend.h"




//命名空间
using namespace std;
using namespace pcl;
using std::atan2;
using std::cos;
using std::sin;
typedef pcl::PointXYZI PointType;

//激光雷达参数
int N_SCANS = 32; //线数
double MINIMUM_RANGE = 0.1;
double MAXIMUM_RANGE = 100.0;
float scan_period = 0.1;       //扫描周期
float Horizon_SCAN = 1800;
int downsampleRate = 1; // Downsample your data if too many points. i.e., 16 = 64 / 4, 16 = 16 / 1

// cv::Mat rangeMat;
//使用数组保存每条线数的角度
float ring16_angle[16] = {-15,1,-13,-3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15} ; //16线激光雷达的角度
float ring32_angle[32] = {-25 ,-1 ,-1.667,-15.639,-11.31,0 , -0.667 , -8.843 , -7.254,0.333,-0.333,-6.148,-5.333,1.333,0.667,-4 ,-4.667,1.667,1,-3.667,-3.333,3.333,2.333,-2.667,-3,7,4.667,-2.333,-2 ,15,10.333,-1.333};//32线激光雷达的角度

//面积计算参数
std::vector<double> distance_cur_vector;                  //定义一个保存当前距离的容器
std::vector<float> distance_between_point_change_vector; //定义一个保存点间距的容器
std::vector<float> area_deque;                           //存放面积单元的容器

int count_interval = 0;                                  //设置海伦公式判定计数器
float area_sum = 0;                                      //每一帧的面积和
float area_ring = 0;                                     //每一线的面积和

//距离突变计数器
std::vector<int> count_change_num_vector; //突变计数器容器

float distance_between_point = 0; //两点之间的距离
float distance_cur = 0;           //当前点到起始点的距离
float distance_xyz = 0 ;
int count_change_num = 0;                 //函数内突变计数器
// std::vector<float> idex_change_vector;       //突变索引容器
float change_num_point;//全局突变次数
float distance_between_point_change_num = 0; //突变点之间的距离

//环境复杂度及趋势计算
std::vector <double>environment_complexity_frequency_vector;

double environment_complexity_frequency = 0;//当前帧环境复杂度（全局变量）
int count_point =0;//设置x轴计数器，到达一定数值就线性统计一下
double  A_frame[2]={0}; //回归系数计数器容器
double  dt_frame[6]={0}; //回归系数器容器
double x_array[10]={0};
double y_array[10]={0};


std::ofstream area_out;  //创建文件保存面积
std::ofstream change_num;
std::ofstream environment_complexity;//保存环境复杂度
std::ofstream resolution_frame_out;//保存分辨率
std::ofstream trend_env_out;//保存趋势
//kdtree近邻密度
float resolution_frame =0;//每一帧的平均k邻点的距离


//自定义一个点云类型，用于存放点云数据
struct PointXYZIRT
{
    PCL_ADD_POINT4D;                // 表示欧几里得 xyz 坐标和强度值的点结构。
    PCL_ADD_INTENSITY;              // 激光点反射的强度，也可以存点的索引，里面是一个float 类型的变量
    uint16_t ring;                  // 总的线圈数
    float time;                     // 时间
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen的字段对齐

} EIGEN_ALIGN16;
// 注册为PCL点云格式，包括的字段为 x,y,z,intensity,ring,time
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

std::deque<sensor_msgs::PointCloud2> cloudQueue; // 激光点云数据队列 // 队列front帧，作为当前处理帧点云
sensor_msgs::PointCloud2 currentCloudMsg;
pcl::PointCloud<PointXYZIRT> laserCloudIn;
pcl::PointCloud<PointXYZ> res_cloundin;
float timeScanCur;
float timeScanEnd;
std_msgs::Header cloudHeader;

pcl::PointCloud<PointXYZIRT>::ConstPtr  res_cloundin_ptr(new pcl::PointCloud<PointXYZIRT>);//创建点云指针存储kd树使用的点云

vector< pcl::PointCloud<PointXYZIRT> >   laserCloudScans_Ring_vector; // 按照线圈数分割的点云
pcl::PointCloud<PointXYZIRT> laserCloudScans_sole; // 按照线圈数分割的点云
//对每一条线上的点数进行统计,存到数组中
std::array<int,32>   ring_point_num = {0};

//声明trend_msgs消息
trajectory_output::trend trend_msgs;
//去处范围外的点
template <typename PointT> //作用是使函数适用于不同的输入类型。
void remove_out_range_point(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres_min, float thres_max)
{
    //输入输出容器大小同步
    cloud_out.points.clear();
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }
    //遍历点云,去除超出范围的点
    //去处距离地面较近的点，离地面15cm以内的点
    size_t j = 0;
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        // cout << "cloud_in.points[i].z" << cloud_in.points[i].z << endl;
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres_min * thres_min

            || cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z > thres_max * thres_max 
            || cloud_in.points[i].z< (-0.07)
            )

            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }
}




//计算环境复杂度变换趋势
/*
功能：线性回归算法求得斜率
入口参数：x:存放自变量x的n个值的数组首地址.
*入口参数：y:存放与自变量x的n个值对应的随机变量观测值的数组首地址.
*入口参数：n:观测点数.
*出口参数：a:长度为2的数组,其中（a[0]存放回归系数b）截距没法使用，a[0]改为存放参与回归点的均值,   a[1]存放回归系数a.
*出口参数：dt:长度为6的数组,dt[0]为偏差平方和, dt[1]为平均标准偏差, dt[2]为回归平方和,   
                         dt[3]为最大偏差,   dt[4]为最小偏差,     dt[5]为偏差平均值.
*说    明：斜率越趋近于0越平稳，大于0说明斜率向高走，小于0说明斜率向低走
*/
void GetCoefficient( double   y[] , float Coeff[8]){
//计划参与线性回归的点
int i ; 
int n=10;
double a[2]={0};
double dt[6]={0};
for (int j = 0 ; j <10 ; j++){
            x_array[j]= j;
    }

double  xx, yy, e, f, q, u, p, umax, umin, s;
 xx = 0.0;   yy = 0.0;
for (i = 0; i <= n - 1; i++)
    {
        xx = xx + x_array[i] / n;
        yy = yy + y[i] / n;
    }
    e = 0.0;   f = 0.0;
    for (i = 0; i <= n - 1; i++)
    {
        q = x_array[i] - xx;   e = e + q   *   q;
        f = f + q   *   (y[i] - yy);
    }
    a[1] = f / e;   a[0] = yy - a[1] * xx;
    q = u = p = 0.0;
    umax = 0.0;   umin = 1.0e+30;
    for (i = 0; i <= n - 1; i++)
    {
        s = a[1] * x_array[i] + a[0];
        q = q + (y[i] - s)   *   (y[i] - s);
        p = p + (s - yy)   *   (s - yy);
        e = fabs(y[i] - s);
        if (e   >   umax)   umax = e;
        if (e   <   umin)   umin = e;
        u = u + e / n;
    }
    dt[1] = sqrt(q / n);
    dt[0] = q;   dt[2] = p;
    dt[3] = umax;   dt[4] = umin;   dt[5] = u;

//将a[0]的值替换成均值
a[0]=yy;
//返回数组
// cout <<"a[0]"<<a[0]<<"a1"<<a[1]<<endl;
Coeff[0]=a[0];
Coeff[1]=a[1];
Coeff[2]=dt[0];
Coeff[3]=dt[1];
Coeff[4]=dt[2];
Coeff[5]=dt[3];
Coeff[6]=dt[4];
Coeff[7]=dt[5];

for (int z=0;z<8;z++)
{       
    // cout<<"Coeff["<<z<<"]"<<Coeff[z]<<endl;
    trend_env_out<<Coeff[z]<<" ";
}

trend_env_out<<endl;

}


//计算点云范围
void get_resolution(const pcl::PointCloud<PointXYZIRT> &resolution_cloud, float &resolution)
{

    pcl::PointCloud<PointXYZIRT>::Ptr cloud(new pcl::PointCloud<PointXYZIRT>);
    cout << "->加载了 " << cloud->points.size() << " 个数据点" << endl;
    PointXYZIRT minPt, maxPt;
    pcl::getMinMax3D(resolution_cloud, minPt, maxPt);
    resolution = (maxPt.x - minPt.x) / cloud->width;
}

//计算点云k近临点
float computeCloundResolution(const pcl::PointCloud<PointXYZIRT>::ConstPtr &cloud, int k)
{

    double res = 0.0; 
    int n_points = 0; //k近临点计数器
    pcl::KdTreeFLANN<PointXYZIRT>::Ptr tree (new pcl::KdTreeFLANN<PointXYZIRT>);
    tree->setInputCloud(cloud);
    //遍历点云，看是否正常
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!pcl_isfinite((*cloud)[i].x)) // pcl_isfinite函数返回一个布尔值，检查某个值是不是正常数值
        {
            continue;
        }
        std::vector<int> indices(k); //创建一个动态数组，存储查询点近邻索引

        std::vector<float> sqr_distances(k); //存储近邻点对应平方距离

        //最近临点
        /*
        
                nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
                if (nres == 2)
            {
            res += sqrt (sqr_distances[1]);
            ++n_points;
            }
      
        */
       
        if (tree->nearestKSearch(i, k, indices, sqr_distances) == k)
        {
            for (int i = 1; i < k; i++)
            {
                res += sqrt(sqr_distances[i]);
                ++n_points;
            }
        }
   
   
   
    }
    if (n_points != 0)
    {
        res /= n_points;
    }



    return res;
}

//计算次数并返回次数
float get_change_num_point(const pcl::PointCloud<PointXYZIRT> &new_cloundin  ,vector<double>distance_cur_vector )
{
    change_num.open("./src/trajectory_output/src/result/change_num.txt", ofstream::out | ofstream::app);
 
    //计算点云的突变阈值，大于误差，小于
    float point_change = 0.1;
    count_change_num = 0;
     int a=0, b=0, c=0 ,d=0 ;
    for (int  i = 0; i < new_cloundin.points.size(); i++)
    {
        if (new_cloundin.points[i].x == 0 && new_cloundin.points[i].y == 0 && new_cloundin.points[i].z == 0)
        {
            continue;
        }
        // if (i = 0 ) continue;
        //加入纵坐标没有报错
        // distance_xyz =  sqrt(new_cloundin.points[i].x * new_cloundin.points[i].x + new_cloundin.points[i].y * new_cloundin.points[i].y+new_cloundin.points[i].z * new_cloundin.points[i].z);
        distance_between_point_change_num = distance_between_point_change_vector[i];
        
        if (distance_cur_vector[i] <= 10) point_change = 0.035 , a++  ;
        else if (10 < distance_cur_vector[i] &&  distance_cur_vector[i] <=25) point_change = 0.087  ,b++   ;
        else if (25<distance_cur_vector[i] &&  distance_cur_vector[i] <= 50) point_change = 0.175  ,c++  ;
        else  point_change = 0.349  ,d++   ;
       
        //  cout<<point_change<< " ";
        if (distance_between_point_change_num > point_change)
        {
            count_change_num = count_change_num + 1;
            // idex_change_vector.push_back(i);
        }
    }
    //  cout<<"a"<<a<<"b"<<b<<"c"<<c<< "d"<<d<<endl;
    //输出突变次数
    //  std::cout << "count_change_num: " << count_change_num << std::endl;
    change_num << count_change_num<<"\n";
    change_num.close();
    return count_change_num;
}

// void Point_time(const pcl::PointCloud<PointXYZIRT>::Ptr &new_laserCloudIn);
//计算当前帧环境复杂度= 跳变频率/面积


void get_cloud_mul ( const pcl::PointCloud<PointXYZIRT> new_laserCloudin_mul  ,  int  N_average_mul  ){
    int N_average = N_average_mul;//均分份数
    int N_points = new_laserCloudin_mul.points.size();//点云总数
    // laserCloudScans_sole.resize(N_points);
    laserCloudScans_Ring_vector.resize(N_average, new_laserCloudin_mul);
  
 //将点云按照ring分割
    for (int j = 0;j < N_points; j++){
                    int scanID = new_laserCloudin_mul.points[j].ring;
                    laserCloudScans_Ring_vector[scanID].points.push_back(new_laserCloudin_mul.points[j]);
                    ring_point_num[scanID] ++;
    }
    // ROS_INFO("N_points= %d",N_points);
   //输出ring_point_num
   for (int i = 0;i < N_average ;i ++){
    // ROS_INFO("ring_point_num[%d]= %d",i,ring_point_num[i]);
    
   }
    
   
 
}
float  get_area (pcl::PointCloud<PointXYZIRT>laserCloud_Ring ){
    //遍历每一个点云点，计算距离
    for (int i = 0; i < laserCloud_Ring.points.size(); i++) {
            //当前点距离
            distance_cur = sqrt(laserCloud_Ring.points[i].x * laserCloud_Ring.points[i].x + laserCloud_Ring.points[i].y * laserCloud_Ring.points[i].y);
            //将当前距离保存到容器中
            distance_cur_vector.push_back(distance_cur);
            //如果是第一个点，则进行下次循环
            if (i == 0) continue;
            //点间距离
            distance_between_point = sqrt(pow(laserCloud_Ring.points[i].x - laserCloud_Ring.points[i - 1].x, 2) + pow(laserCloud_Ring.points[i].y - laserCloud_Ring.points[i - 1].y, 2));
            distance_between_point_change_vector.push_back(distance_between_point);
            //计算面积单元，海伦公式
            float a, b, c, p, s;
            a = distance_between_point;
            b = distance_cur;
            c = distance_cur_vector[i - count_interval];
            if ((a + b > c) && (a + c > b) && (b + c > a))
            {
                p = (a + b + c) / 2;
                s = sqrt(p * (p - a) * (p - b) * (p - c));
                area_deque.push_back(s);
                count_interval = 1;
            }
            else
            {
                //间隔自加一
                count_interval += 1;
            }
        
        }
        //计算每一线面积和 //每一帧0.1秒，为32线激光雷达
        area_ring = (accumulate(area_deque.begin(), area_deque.end(), 0.0));
        //清空容器
        area_deque.clear();
  
        return area_ring;
}

void get_special_complexity(const pcl::PointCloud<PointXYZIRT> &cloundin)
{
area_out.open("./src/trajectory_output/src/result/area_out.txt", ofstream::out | ofstream::app);
  
     //根据线数分组
    get_cloud_mul(cloundin,N_SCANS);
    
    int a  = 0 ;
    //计算每一条线的面积
    for (int i = 0; i < N_SCANS; i++)
    {   
        if ( ring_point_num[i] == 0  ) continue;
        //计算每一条线的面积
        //  float  area_sum_ring = get_area(laserCloudScans_Ring_vector[i]);
     
        // cout <<"a"<<a<< "area_sum_ring"<<area_sum_ring<<endl;
        a++;
        //将面积保存到容器中
      
    }
        // trend_msgs.area=area_sum;
        // area_out  << area_sum<<"\n"; 
        // area_out.close();
        // std::cout << "area_sum: " << area_sum << std::endl;//打印每一帧面积和
    
    //清零
    for ( int x = 0 ;x <ring_point_num.size() ; x++  ){
         ring_point_num[x] = 0 ;
    }
    

   //原版
    for (int i = 0; i < cloundin.points.size(); i++)
    {
        //遍历每一个点云点，计算距离
        //计算当前点距离
        distance_cur = sqrt(cloundin.points[i].x * cloundin.points[i].x + cloundin.points[i].y * cloundin.points[i].y);
        //  if (distance_cur>50) cout <<"distance_cur"<<distance_cur<<endl;
        //将当前距离保存到容器中
        distance_cur_vector.push_back(distance_cur);
        //如果是第一个点，则进行下此循环
        if (i == 0) continue;
        //点间距离
        distance_between_point = sqrt(pow(cloundin.points[i].x - cloundin.points[i - 1].x, 2) + pow(cloundin.points[i].y - cloundin.points[i - 1].y, 2));
        distance_between_point_change_vector.push_back(distance_between_point);
        //计算面积单元，海伦公式
        float a, b, c, p, s;
        a = distance_between_point;
        b = distance_cur;
        c = distance_cur_vector[i - count_interval];

        if ((a + b > c) && (a + c > b) && (b + c > a))
        {
            p = (a + b + c) / 2;
            s = sqrt(p * (p - a) * (p - b) * (p - c));
            area_deque.push_back(s);
            count_interval = 1;
        }
        else
        {
            //间隔自加一
            count_interval += 1;
        }
       
    }
    //计算每一帧面积和 //每一帧0.1秒，为32线激光雷达
    area_sum = (accumulate(area_deque.begin(), area_deque.end(), 0.0))/320;
    trend_msgs.area=area_sum;
    area_out  << area_sum<<"\n"; 
    area_out.close();
    // std::cout << "area_sum: " << area_sum << std::endl;//打印每一帧面积和
    

  
    //遍历点云，计算突变次数
     change_num_point=get_change_num_point(cloundin , distance_cur_vector);
    // cout<<"change_num_point"<<change_num_point<<endl;
   trend_msgs.changeNumber = change_num_point;
    //计算突变频率,环境复杂度
    environment_complexity_frequency = change_num_point / area_sum;
    environment_complexity_frequency_vector.push_back(environment_complexity_frequency);
    trend_msgs.environmentComplexity = environment_complexity_frequency;
    // cout<<"environment_complexity_frequency"<<environment_complexity_frequency<<endl;
    environment_complexity.open("./src/trajectory_output/src/result/environment_complexity.txt", ofstream::out | ofstream::app);
    environment_complexity << environment_complexity_frequency<<"\n"; 
    environment_complexity.close();

    
 
  
  
  //每帧计算结束后清零容器
    distance_cur_vector.clear();   //保存距离的容器
    //设置计数器，到达数字就归零
    area_deque.clear(); //存放面积单元的容器
    area_sum = 0; //每一帧的面积和重置
    count_interval = 0;
    change_num_point = 0;//跳变次数
}

/*
//给点云数据增加时间，并使得点云数据有序
void Point_time(const pcl::PointCloud<PointXYZIRT>::Ptr &new_laserCloudIn)
{
    // rangeMat = cv::Mat(N_SCANS, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

    int cloundsize = (int) laserCloudIn.points.size();
    //初始化点云定义大小
    pcl::PointCloud<PointXYZIRT> temp_laserCloudIn ;
    temp_laserCloudIn.points.resize(cloundsize);
    //遍历点云
    for (size_t i = 0; i <= cloundsize; i++)
    {
        // pcl格式的点云
        PointType thisPoint;
        // laserCloudIn就是原始的点云话题中的数据
        thisPoint.x = laserCloudIn.points[i].x;
        thisPoint.y = laserCloudIn.points[i].y;
        thisPoint.z = laserCloudIn.points[i].z;
        thisPoint.intensity = laserCloudIn.points[i].intensity;

        int rowIdn = laserCloudIn.points[i].ring;//将该点的线数赋值给rowIdn
        if (rowIdn < 0 || rowIdn >= N_SCANS)
            continue;
        if (rowIdn % downsampleRate != 0)
            continue;

        int columnIdn = -1;
        // 水平扫描角度
        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        // 水平扫描角度步长 // Horizon_SCAN=1800,每格0.2度
        static float ang_res_x = 360.0 / float(Horizon_SCAN);
        // horizonAngle 为[-180,180],horizonAngle -90 为[-270,90], (horizonAngle-90.0)/ang_res_x为[450,-1350]，-round((horizonAngle-90.0)/ang_res_x)为[-450,1350],
        //-round((horizonAngle-90.0)/ang_res_x) +Horizon_SCAN/2为[450,2250]
        //  即把horizonAngle从[-180,180]映射到[450,2250]
        columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
        //大于1800，则减去1800，相当于把1801～2250映射到1～450
        //先把columnIdn从horizonAngle:(-PI,PI]转换到columnIdn:[H/4,5H/4],
        //然后判断columnIdn大小，把H到5H/4的部分切下来，补到0～H/4的部分。
        //将它的范围转换到了[0,H] (H:Horizon_SCAN)。
        //这样就把扫描开始的地方角度为0与角度为360的连在了一起，非常巧妙。
        //如果前方是x，左侧是y，那么正后方左边是180，右边是-180。这里的操作就是，把它展开成一幅图:
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;
          if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;
        float range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < 1.0)
            continue;
        // if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
        //         continue;
          if (thisPoint.z < -2.0)
             continue;


        float pointTime = timeScanCur + laserCloudIn.points[i].time;
        // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster
        // 转换成一维索引，存校正之后的激光点
        // rowIdn该点激光雷达是水平方向上第几线的。从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16
        int index = columnIdn + rowIdn * Horizon_SCAN;

       
        // newPoint.points[index].time = pointTime;

   
        //二者相加即可得到当前点的准确时刻
        // cout.setf(ios::fixed, ios::floatfield);
        // cout.precision(20);
        // std::cout << "laserCloudIn.points[i].time" << laserCloudIn.points[i].time << std::endl;
        // std::cout << "pointTime:\t" << newPoint.points[index].time << std::endl;
    }
}
*/

//点云回调函数
void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg)
{
    //遍历激光点云
    cloudQueue.push_back(*laserCloudMsg); //将激光点云添加到队列中
    if (cloudQueue.size() <= 3)  return;        //判断队列中的点云数量是否大于2，如果小于2，则不进行处理

    currentCloudMsg = std::move(cloudQueue.front()); // std::move 的内部只做了一个强制类型转换
    cloudQueue.pop_front();                          //删除第一个或最后一个元素pop_front 和pop_back 函数—list可以，queue也可以，vector不支持

    // 转换成pcl点云格式 形参: (in,out)
    pcl::moveFromROSMsg(currentCloudMsg, laserCloudIn);
    laserCloudIn.header.stamp = currentCloudMsg.header.stamp.toSec(); //将点云的时间戳设置为当前时间
    timeScanCur = currentCloudMsg.header.stamp.toSec();               //获取当前时间
   
//    //是否包含ring字段
//     for (int i = 0; i < laserCloudIn.size(); i++)
//     {//将该点的线数赋值给rowIdn
//       int VELrowIdn = laserCloudIn.points[i].ring;
//     ROS_INFO("VELrowIdn:%d", VELrowIdn);
//     }
    //转换为pcl格式
    pcl::PointCloud<PointXYZIRT> temp_laserCloudIn;
    //首先对点云滤波,去除NaN值得无效点云，以及在Lidar坐标系原点MINIMUM_RANGE距离以内的点
    std::vector<int> indices; //输出点云的索引
    pcl::removeNaNFromPointCloud(laserCloudIn, temp_laserCloudIn, indices);

    //存放范围内的点云
    pcl::PointCloud<PointXYZIRT> new_laserCloudIn;
    //去处超出范围的点
    remove_out_range_point(temp_laserCloudIn, new_laserCloudIn, MINIMUM_RANGE, MAXIMUM_RANGE);

    // pcl::PointCloud<PointXYZIRT> temp_new_laserCloudIn;
    // pcl::copyPointCloud(new_laserCloudIn, temp_new_laserCloudIn);
      //给点云点定义时间,并使点云有序
    // Point_time(new_laserCloudIn);

    int  N_clound_size = new_laserCloudIn.size();
    trend_msgs.num_clound_size = N_clound_size;

    get_special_complexity(new_laserCloudIn);

 
    
    //    cout<<"count_point"<<count_point<<endl;//当前帧
  
    int num_count = count_point%10;//每隔10帧计算一次线性回归
    if (num_count == 0 &&  count_point >0)
            { 
                trend_env_out.open("./src/trajectory_output/src/result/trend_env_out.txt", ofstream::out | ofstream::app);
                // cout<<"回归"<<endl;           
                float Coeff[8];
                //线性回归计算趋势
               GetCoefficient (&y_array[0],&Coeff[0]);
               trend_env_out.close();
               //将数组传入到自定义消息中
                for(int i=0;i<8;i++)
                {
                    trend_msgs.env_trend[i]=Coeff[i];
                }
                
                y_array[num_count] = environment_complexity_frequency;
                
            }
    else
            {    
                y_array[num_count] = environment_complexity_frequency;
            }
       
    //构建kd树计算k近邻密度
    int  kd_num_count= count_point%10;//每间隔50帧计算一次k近邻密度
    if (kd_num_count == 0 )
        {
                resolution_frame_out.open("./src/trajectory_output/src/result/resolution_frame_out.txt", ofstream::out | ofstream::app);
                //kd树
                //使用k近邻计算//极大的降低了计算速度
                //计算每一帧点云的平均密度，判断是哪一种密集情况
                res_cloundin_ptr= new_laserCloudIn.makeShared();
                resolution_frame = computeCloundResolution( res_cloundin_ptr,2 );
                cout <<"resolution_frame"<<resolution_frame<<endl;
                resolution_frame_out<<resolution_frame<<endl;
                resolution_frame_out.close();
                //resolution_frame传入自定义消息
                trend_msgs.resolution_frames=resolution_frame;

                res_cloundin_ptr.reset(new pcl::PointCloud<PointXYZIRT>);
        }
         count_point = count_point +1;

        
        

}




int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "Voxel_filter");
    ros::NodeHandle nh;

    printf("scan line number %d \n", N_SCANS);
    printf("minimum_range %f \n ", MINIMUM_RANGE);
    printf("maximum_range %f \n", MAXIMUM_RANGE);

    std::ofstream area_out("./src/trajectory_output/src/result/area_out.txt", ofstream::trunc);
    std::ofstream environment_complexity("./src/trajectory_output/src/result/environment_complexity.txt", ofstream::trunc);
    std::ofstream change_num("./src/trajectory_output/src/result/change_num.txt", ofstream::trunc);
    std::ofstream  resolution_frame_out("./src/trajectory_output/src/result/resolution_frame_out.txt", ofstream::trunc);
    std::ofstream trend_env_out("./src/trajectory_output/src/result/trend_env_out.txt", ofstream::trunc);
    //接受激光雷达原始点云数据
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 500, laserCloudHandler);
   //发布计算的结果
    ros::Publisher   pubTrend = nh.advertise<trajectory_output::trend>("Trend", 1000);

    //组织被发布数据
    ros::Rate r(1);
    while (ros::ok())
    {
        // ROS_INFO("发布成功");
        
        pubTrend.publish(trend_msgs);
     
        

        r.sleep();
        ros::spinOnce();
    }




    ros::spin();
    return 0;
}