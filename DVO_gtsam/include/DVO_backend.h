#ifndef DVO_BACK
#define DVO_BACK
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <sophus/se3.hpp>

#include <opencv2/core/core.hpp>
#include "util.h"




/**
 * @class DirectImageAlignment
 * @brief Class for direct image alignment between 2 images.
 */
class DVO_backend{
private:
   
    //Image and camera matrix.
    cv::Mat img_cur;
    cv::Mat img_prev;
    cv::Mat depth_cur;
    cv::Mat depth_prev;
    Eigen::Matrix3f K;

    //Robust weight estimation
    static constexpr float INITIAL_SIGMA = 5.0f;
    static constexpr float DEFAULT_DOF = 5.0f;

public:

    /**
     * @brief Calculate image intensity gradients.
     * @param[in] direction: xdirection(0) or ydirection(1).
     */
    void calcGradient(const cv::Mat &img, cv::Mat &gradient, int direction);

    /**
     * @brief Calculate residual (photometric error between previous image and current image)
     */
    Eigen::VectorXf calcRes(const Eigen::VectorXf &xi+);

    /**
     * @brief Calculate Jacobian for minimizing least square error.
     */
    Eigen::MatrixXf calcJacobian(const Eigen::VectorXf &xi);

    /**
     * @brief Compute robust weights from residuals.
     */
    void weighting(Eigen::VectorXf &residuals, Eigen::VectorXf &weights);

    /**
     * @brief Execute GaussNewton optimzation and find the optimam rotation and translation.
     * @param[out] rot, t: camera pose transformation between 2 images.
     */
    void doGaussNewton(Eigen::Matrix3f& rot, Eigen::Vector3f& t);

    /**
     * @brief Execute direct image alignment between last and current image.
     * @param[out] transform: camera pose transformation between 2 images.
     */
    void AssignImage( Eigen::Matrix4f& transform, const cv::Mat& img_prev, const cv::Mat& depth_prev,
                  const cv::Mat& img_cur, const cv::Mat& depth_cur);
};


#endif