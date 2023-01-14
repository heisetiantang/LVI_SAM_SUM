#include "ros/ros.h"
#include <bits/stdc++.h>
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>
#include "trajectory_output/trend.h"
using namespace std;

void doPerson(const trajectory_output::trend::ConstPtr& msg)
{
    ROS_INFO("I heard: [%.2f]", msg->area);
    cout<<msg->area<<endl;
     std::cout<<msg->env_trend[0]<<endl;
     cout <<"resolution_frames"<<msg->resolution_frames<<endl;
}

int main (int argc, char  *argv[])
{

setlocale(LC_ALL,"");

//2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一

   ros::init(argc,argv,"pub_trajectory_test");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

//4.实例化 发布者 对象
ros::Subscriber subA = nh.subscribe<trajectory_output::trend>("Trend",10,doPerson);

 
  //5.ros::spin();
    ros::spin();    
  
    return 0;


}