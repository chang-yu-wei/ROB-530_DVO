#ifndef IMAGE_ALIGNMENT_H
#define IMAGE_ALIGNMENT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <opencv2/core/core.hpp>
#include "iostream"

/**
 * @class ImageAlignment
 * @brief Class for image alignment between 2 frames
 */
class ImageAlignment{
private:

    //Parameters for downsampling
    static const int num_pyramid = 5;
    static const int num_GNiterations = 500;

    //Image and camera matrix K
    cv::Mat img_cur;
    cv::Mat img_prev;
    cv::Mat depth_cur;
    cv::Mat depth_prev;
    Eigen::Matrix3f K;

    //Image and camera matrix pyramids
    cv::Mat img_prev_Pyramid[num_pyramid];
    cv::Mat depth_prev_Pyramid[num_pyramid];
    cv::Mat img_cur_Pyramid[num_pyramid];
    cv::Mat depth_cur_Pyramid[num_pyramid];
    Eigen::Matrix3f k_Pyramid[num_pyramid];

    //Robust weight estimation
    static constexpr float INITIAL_SIGMA = 5.0f;
    static constexpr float DEFAULT_DOF = 5.0f;

    float error;
public:

    /**
     * @brief Downsample grey image to (w/2,h/2) for pyramid
     */
    cv::Mat downsampleImg(const cv::Mat &grey);

    /**
     * @brief Downsample depth image to (w/2,h/2) for pyramid
     */
    cv::Mat downsampleDepth(const cv::Mat &depth);

    /**
     * @brief build pyramids of grey/depth and camera matrix
     */
    void createPyramid();

    /**
     * @brief Calculate image intensity gradients
     * @param[in] direction: 0 for x, 1 for y
     */
    void calcGradient(const cv::Mat &img, cv::Mat &gradient, int direction);

    /**
     * @brief Calculate photometric error between previous image and current image
     */
    Eigen::VectorXf calcRes(const Eigen::VectorXf &xi, const int level);

    /**
     * @brief Calculate Jacobian for least square 
     */
    Eigen::MatrixXf calcJacobian(const Eigen::VectorXf &xi, const int level);

    /**
     * @brief Compute robust weights from residuals
     */
    void computeWeighting(Eigen::VectorXf &residuals, Eigen::VectorXf &weights);

    /**
     * @brief Execute GaussNewton optimzation and find the optimam rotation and translation.
     * @param[out] rot, t: camera pose transformation between 2 images.
     */
    void GaussNewton(Eigen::Matrix3f& rot, Eigen::Vector3f& t);

    /**
     * @brief Execute direct image alignment between last and current image.
     * @param[out] transform: camera pose transformation between 2 images.
     */
    void alignment( Eigen::Matrix4f& transform, const cv::Mat& img_prev, const cv::Mat& depth_prev,
                      const cv::Mat& img_cur, const cv::Mat& depth_cur, const Eigen::Matrix3f& K);

    float getError();
};


#endif //IMAGE_ALIGNMENT_H