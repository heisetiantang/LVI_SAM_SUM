#pragma once
#include "utility/utility.h"
#include "estimator.h"
#include "parameters.h"
#include "feature_manager.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>




class Confidence
{
  public:
    Confidence();

    void calvisondiff();

    void calinversematrix();
  
    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];
    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];
    int frame_count;

    double w_fecture;//特征跟踪性能权值5
    double w_motion;//线速度性能权值2
    double w_angle;//角速度性能权值5
    double Scvalue;//计算S值
    
    static Eigen::Matrix2d sqrt_info;//视觉残差
    static Eigen::Matrix2d sqrt_info_matix;//视觉残差矩阵的逆矩阵
    static Eigen::Matrix2d infomation_matix;//求信息矩阵
    
  

    FeatureManager          *f_manager;
    ProjectionFactor         *p_factor;
    Estimator                       *e_estimator;
    
};

