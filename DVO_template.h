#ifndef DVO
#define DVO

#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <time.h>
 #include <Eigen/Core>

class DVO_template
{
    private:
        Eigen::Matrix Intrinsic;
        Eigem::Vector Distoration;

        PointCloud Img2PC(cv::mat & RGB, cv::mat Depth);
        Eigen::Matrix Wrap2Traget(RGB, Depth, Transform);
        
    public:
        DVO_template(Some Camera Parameters);
        ~DVO_template();
        
        // given two RGB-D images, find the T from target to source that minimize the photometric error
        Eigen::Matrix4d Ailign(cv::mat& Source_RGB, cv::mat& Source_D, cv::mat& Target_RGB, cv::mat& Taget_D, Eigen::Matrix4d T_init);

        // given two RGB-D images or point cloud, calculate the cost of transformation T
        double Calc_Resigual(cv::mat& Source_RGB, cv::mat& Source_D, cv::mat& Target_RGB, cv::mat& Taget_D, Eigen::Matrix4d T);

        //
        Eigen::Matrix Calc_Jacobian(cv::mat& Source_RGB, cv::mat& Source_D, cv::mat& Target_RGB, cv::mat& Taget_D, Eigen::Matrix4d T);

        
};






#endif