#include "utility.h"
#include "lvi_sam/cloud_info.h"
// nthnth
struct smoothness_t
{
    float value;
    size_t ind;
};

struct by_value
{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:
    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    //****************设置一个值来保存比例值
    // vector<float> ratio_value;
    double rotioofedge2surf = 0;
    double rotioEdge2SurfDC = 0;
    float edgeNum = 0;
    float surfaceNum = 0;
    float rotiofactor = 1; //比例系数
    float rotiofactorsurf = 1;

    //****************

    pcl::VoxelGrid<PointType> downSizeFilter;

    lvi_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction()
    {
        //重点关注该订阅者->调用FeatureExtraction对象的函数laserCloudInfoHandler
        subLaserCloudInfo = nh.subscribe<lvi_sam::cloud_info>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        //订阅者subLaserCloudInfo处理完点云后，将相关数据通过如下三个发布者向后续节点发布数据；
        pubLaserCloudInfo = nh.advertise<lvi_sam::cloud_info>(PROJECT_NAME + "/lidar/feature/cloud_info", 5);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/feature/cloud_corner", 5);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/feature/cloud_surface", 5);

        //初始化相关容器
        initializationValue();
    }

    void initializationValue()
    {
        // cout<<"初始化"<<endl;
        cloudSmoothness.resize(N_SCAN * Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN * Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
        cloudLabel = new int[N_SCAN * Horizon_SCAN];

        //清空容器
        // ratio_value.clear();

        //************初始化储存角点/面点/比值的容器

        // ratio_value.resize(N_SCAN*Horizon_SCAN);
    }

    void laserCloudInfoHandler(const lvi_sam::cloud_infoConstPtr &msgIn)
    {
        // 1、预处理：点云格式转换
        cloudInfo = *msgIn;                                      // new cloud info
        cloudHeader = msgIn->header;                             // new cloud header
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

        // 2、计算点云中各点曲率，为提取特征点左准备
        calculateSmoothness();

        // 3、屏蔽点云中被遮挡点或平行点
        markOccludedPoints();

        // 4、依据曲率，提取角特征和面特征
        extractFeatures();

        // 5、向后续节点发布提取到的角特征和面特征
        publishFeatureCloud();
    }

    void calculateSmoothness()
    {
        // 当前雷达帧中点云中点的数量
        int cloudSize = extractedCloud->points.size();

        // 根据曲率计算公式，所以起始5个点(序号0-4)和最后五个点，不计算曲率；
        for (int i = 5; i < cloudSize - 5; i++)
        {
            //曲率计算公式 = 当前点前5个点距离和 - 当前点距离*10 + 当前点后5个点距离和；
            float diffRange = cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4] + cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2] + cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10 + cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2] + cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4] + cloudInfo.pointRange[i + 5];

            // 曲率采用距离的平方的形式
            // diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudCurvature[i] = diffRange * diffRange;

            //  计算特征点的标记位
            // 1:表示不参与提取特征点，可能受遮挡等
            // 0:表示参与提取特征点
            // 默认为参与提取特征点
            cloudNeighborPicked[i] = 0;

            // 特征点的分类标记
            // 1：曲率比较大
            // 0:默认情况下，比较平坦点，现实世界也是以平面点为主；
            // 不满足1、-1的情况下，都是0比较平坦点
            // -1:平坦的点
            cloudLabel[i] = 0;

            // cloudSmoothness for sorting
            // 该点索引，后面需对点曲率排序会造成混乱，这个时候索引的作用就体现出来了;
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        // 当前雷达帧中点云中点的数量
        int cloudSize = extractedCloud->points.size();

        // mark occluded points and parallel beam points
        // 屏蔽受遮挡的点和平行的点
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // occluded points
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i + 1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

            // 如果相邻两个点，其水平角度接近，则距离中心更远的点及其周围5个点视为被遮挡点；
            if (columnDiff < 10)
            {
                // 10 pixel diff in range image
                if (depth1 - depth2 > 0.3)
                {
                    // 1:表示不参与提取特征点，可能受遮挡等
                    // 0:表示参与提取特征点
                    // 默认为参与提取特征点
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
                else if (depth2 - depth1 > 0.3)
                {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }

            // parallel beam
            float diff1 = std::abs(float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));
            // 如果点和相邻两个点的距离差都很大，则可能是平行线束，视为被遮挡点；
            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures()
    {
        // 存放角点特征的点云和面点特征的点云；
        // 计算特征前，容器先清空；
        cornerCloud->clear();
        surfaceCloud->clear();
        float size_surfaceCloundDS ;

        edgeThreshold =1;
       
        // downSizeFilter.setLeafSize(0.2,0.2,0.2);



        edgeNum = 0.0;
        surfaceNum = 0.0;
        edgeNumout.open("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/pointChange/featurePoint/edgeNum.txt", ofstream::out | ofstream::app);
        rotioofedge2surfout.open("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/pointChange/featurePoint/rotioofedge2surf.txt", ofstream::out | ofstream::app);
        // surfaceNumout.open("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/surfaceNum.txt", ofstream::out | ofstream::app);
        numsurfaceCloudScanDSout.open("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/pointChange/featurePoint/numsurfaceCloudScanDS.txt", ofstream::out | ofstream::app);



        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        // 保证提取的特征整体分布较为均匀
        // 遍历每个scan
        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();
            //每个scan6等分
            for (int j = 0; j < 6; j++)
            {

                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                // 对当前提取段的曲率进行排序
                // 按照曲率从小到大：by_value();
                // cloudSmoothness: 点的index和曲率，排序按照曲率排序，尽管cloudSmoothness被打乱了，可以用index与曲率的关联找到,以及该点状态；
                //
                std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

                //记录挑到的点数
                int largestPickedNum = 0;
                //选取曲率大的点，所以从后往前遍历；
                for (int k = ep; k >= sp; k--)
                {
                    //利用cloudSmoothness中index可以关联到该点是否参与提取特征点的状态
                    int ind = cloudSmoothness[k].ind;
                    // 0：参与提取特征点，曲率大于阈值，则算1个合格特征点
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        //每个scan分成6段，每段提取满足条件的曲率前20大的点作为特征点
                        if (largestPickedNum <= 20)
                        {
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);

                            edgeNum = edgeNum + 1;
                        }
                        else
                        {
                            break;
                        }

                        //避免特征点过于集中，将当前点周围5个点状态置为1，不参与特征提取
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 面点特征提取，相同道理，每段scan分成6段；
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {

                        cloudLabel[ind] = -1;

                        //避免面点过于集中，当前点周围5各点被屏蔽掉；
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++)
                        {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                    {
                        surfaceCloudScan->push_back(extractedCloud->points[k]);

                        surfaceNum = surfaceNum + 1;
                    }
                }
            }




             size_surfaceCloundDS += surfaceCloudScanDS->width * surfaceCloudScanDS->height;
            //避免特征过大，造成计算时间过长，进行降采样；
            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            // downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(*surfaceCloudScanDS);
            *surfaceCloud += *surfaceCloudScanDS;
        }
            
            
        rotioofedge2surf = edgeNum / surfaceNum;//求角点和平面点的比例，并输出
       rotioEdge2SurfDC =edgeNum / (surfaceCloud->points.size());
        
        //  cout << "edgeNum: 初始角点" << edgeNum << endl;
        
        //  cout << "PointCloud before filtering: surfaceNum: 平面点" << surfaceNum << endl;
        // surfNum_vector.push_back(surfaceNum); //输入平面点数到容器中
       // edgeNum_vector.push_back(edgeNum); //输入角点数到容器中
        // ratio_vector.push_back(rotioofedge2surf); //将比例值保存在容器中
        // cout << "rotiofactor ;//比例系数" << rotiofactor <<endl;
        // cout << "一次滤波后平面点数目：" << surfaceCloud->points.size() << endl;
        // cout << "rotioofedge2surf: //比例" << rotioofedge2surf << endl;

        edgeNumout   << edgeNum << "\n";                                      //写入角点
        // surfaceNumout << surfaceNum << "\n";                             //写入滤波前平面点
        numsurfaceCloudScanDSout  << surfaceNum<< "      " <<  surfaceCloud->points.size()<< "\n" ; //滤波前后面点数量变化
        rotioofedge2surfout << rotioofedge2surf <<" "<< rotioEdge2SurfDC<< "\n";  //角点和面点比例；角点和滤波后平面点比例
      

        edgeNumout.close();
        // surfaceNumout.close();
        rotioofedge2surfout.close();
        numsurfaceCloudScanDSout.close();
    }

    // 清空内存
    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.startRingIndex.shrink_to_fit();
        cloudInfo.endRingIndex.clear();
        cloudInfo.endRingIndex.shrink_to_fit();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointColInd.shrink_to_fit();
        cloudInfo.pointRange.clear();
        cloudInfo.pointRange.shrink_to_fit();
    }

    // 发布角点特征点云和面点特征点云
    void publishFeatureCloud()
    {
        // free cloud info memory
        // 清空点云内存
        freeCloudInfoMemory();

        // save newly extracted features
        //发布最新的角点点云和面点点云
        cloudInfo.cloud_corner = publishCloud(&pubCornerPoints, cornerCloud, cloudHeader.stamp, "base_link");
        cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, "base_link");
        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar");

    std::ofstream edgeNumout("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/pointChange/featurePoint/edgeNum.txt", ofstream::trunc);
    std::ofstream rotioofedge2surfout("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/pointChange/featurePoint/rotioofedge2surf.txt", ofstream::trunc);
    // std::ofstream surfaceNumout("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/surfaceNum.txt", ofstream::trunc);
    std::ofstream numsurfaceCloudScanDSout("/home/gjm/catkin_ws_lvi/src/LVI-SAM_detailed_comments-master/src/pointChange/featurePoint/numsurfaceCloudScanDS.txt", ofstream::trunc);

    //构建特征提取对象->构造器
    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Lidar Feature Extraction Started.\033[0m");

    ros::spin();

    return 0;
}
