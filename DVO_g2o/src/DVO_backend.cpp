#include "DVO_backend.h"

void DirectImageAlignment::calcGradient(const cv::Mat &img, cv::Mat &gradient, int direction) {
    static int dx[2] = {1, 0};
    static int dy[2] = {0, 1};
    int w = img.cols;
    int h = img.rows;
    float* input_ptr = (float*)img.data;
    gradient = cv::Mat::zeros(h, w, CV_32FC1);
    float* output_ptr = (float*)gradient.data;

    int y_offest = dy[direction];
    int x_offset = dx[direction];

    if(direction == 0) {
        for (int y = y_offest; y < h-y_offest; y++) {
            for (int x = x_offset; x < w-x_offset; x++) {
                float v0 = input_ptr[y * w + (x - 1)];
                float v1 = input_ptr[y * w + (x + 1)];
                output_ptr[y * w + x] = 0.5f * (v1 - v0);
            }
        }
    }else {
        for (int y = y_offest; y < h-y_offest; y++) {
            for (int x = x_offset; x < w-x_offset; x++) {
                float v0 = input_ptr[(y - 1) * w + x];
                float v1 = input_ptr[(y + 1) * w + x];
                output_ptr[y * w + x] = 0.5f * (v1 - v0);
            }
        }
    }

    return;
}

Eigen::VectorXf DirectImageAlignment::calcRes(const Eigen::VectorXf &xi) {

    Eigen::VectorXf residuals;

    int w = img_prev.cols;
    int h = img_prev.rows;

    // camera intrinsics
    float fx = this->K(0, 0);
    float fy = this->K(1, 1);
    float cx = this->K(0, 2);
    float cy = this->K(1, 2);
    float fxi = 1.0 / fx;
    float fyi = 1.0 / fy;

    // convert SE3 to rotation matrix and translation vector
    Eigen::Matrix3f rot;
    Eigen::Vector3f t;
    SE3ToRt(xi, rot, t);

    float* ptr_img_prev = (float*)img_prev.data;
    float* ptr_depth_prev = (float*)depth_prev.data;
    float* ptr_img_cur = (float*)img_cur.data;
    float* ptr_depth_cur = (float*)depth_cur.data;

    residuals.resize(w*h);
    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < w; x++)
        {
            int off = y*w + x;
            float residual = 0.0;

            float Z_prev = ptr_depth_prev[y*w + x];
            float X_prev = ((float)(x) - cx) * fxi * Z_prev;
            float Y_prev = ((float)(y) - cy) * fyi * Z_prev;
            Eigen::Vector3f point3d_prev(X_prev, Y_prev, Z_prev);
            Eigen::Vector3f point3d_warped = rot * point3d_prev + t;

            if (point3d_warped[2] > 0.0) {
                Eigen::Vector3f point2d_warped = K * point3d_warped;
                float px = point2d_warped[0] / point2d_warped[2];
                float py = point2d_warped[1] / point2d_warped[2];

                float color_warped = interpolate(ptr_img_cur, px, py, w, h);
                if (!std::isnan(color_warped))
                {
                    float color_prev = ptr_img_prev[off];
                    residual = color_prev - color_warped;
                }
            }

            residuals[off] = residual;
        }
    }
    Eigen::VectorXf weights;

    weighting(residuals, weights);
    residuals = residuals.cwiseProduct(weights);

    return residuals;
}

Eigen::MatrixXf DirectImageAlignment::calcJacobian(const Eigen::VectorXf &xi){
    cv::Mat grad_x, grad_y;
    calcGradient(img_cur, grad_x, 0);
    calcGradient(img_cur, grad_y, 1);

    Eigen::MatrixXf J;
    float* ptr_gradx = (float*)grad_x.data;
    float* ptr_grady = (float*)grad_y.data;
    float* ptr_depth_prev = (float*)depth_prev.data;


    //Camera parameters
    float fx = this->K(0, 0);
    float fy = this->K(1, 1);
    float cx = this->K(0, 2);
    float cy = this->K(1, 2);
    float fxi = 1.0 / fx;
    float fyi = 1.0 / fy;

    //Width and Height
    int w = img_cur.cols;
    int h = img_cur.rows;

    //RotationMatrix and t
    Eigen::Matrix3f rot;
    Eigen::Vector3f t;
    SE3ToRt(xi, rot, t);

    //Jacobian
    Eigen::MatrixXf J1;
    J1 = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf Jw(2,6);
    J = Eigen::MatrixXf(w*h,6);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {

            float Z_prev = ptr_depth_prev[y*w + x];
            float X_prev = ((float)(x) - cx) * fxi * Z_prev;
            float Y_prev = ((float)(y) - cy) * fyi * Z_prev;
            Eigen::Vector3f point3d_prev(X_prev, Y_prev, Z_prev);
            Eigen::Vector3f point3d_warped = rot * point3d_prev + t;
            float X = point3d_warped[0];
            float Y = point3d_warped[1];
            float Z = point3d_warped[2];

            Jw << fx*1/Z, 0, -fx*X/(Z*Z), -fx*(X*Y)/(Z*Z), fx*(1 + (X*X)/(Z*Z)), -fx*Y/Z,
                    0, fy*1/Z, -fy*Y/(Z*Z), -fy*(1+(Y*Y)/(Z*Z)), fy*X*Y/(Z*Z), fy*X/Z;

            if (point3d_warped[2] > 0.0){
                // project 3d point to 2d
                Eigen::Vector3f point2d_warped = K * point3d_warped;
                float u = point2d_warped[0] / point2d_warped[2];
                float v = point2d_warped[1] / point2d_warped[2];
                J1(0,0) = interpolate(ptr_gradx, u, v, w, h);
                J1(0,1) = interpolate(ptr_grady, u ,v, w, h);
            }

            J.row(y*w+x) = - J1*Jw;

            if(!std::isfinite(J.row(y*w+x)[0]))
                J.row(y*w+x).setZero();

        }
    }

    return J;
}

void  DirectImageAlignment::weighting(Eigen::VectorXf &residuals, Eigen::VectorXf &weights) {
    int n = residuals.size();
    float lambda_init = 1.0f / (INITIAL_SIGMA * INITIAL_SIGMA);
    float lambda = lambda_init;
    float num = 0.0;
    float dof = DEFAULT_DOF;
    weights = Eigen::VectorXf::Ones(n);
    int itr = 0;
    do {
        itr++;
        lambda_init = lambda;
        lambda = 0.0f;
        num = 0.0f;
        for(int i = 0; i < n; ++i) {
            float data = residuals(i);

            if(std::isfinite(data)) {
                num += 1.0f;
                lambda += data * data * ( (dof + 1.0f) / (dof + lambda_init * data * data) );
            }
        }
        lambda /= num;
        lambda = 1.0f / lambda;
    } while(std::abs(lambda - lambda_init) > 1e-3);

    for(int i=0; i<n; i++){
        float data = residuals(i);
        weights(i) = ( (dof + 1.0f) / (dof + lambda * data * data) );
    }
}

void DirectImageAlignment::doGaussNewton(Eigen::Matrix3f& rot, Eigen::Vector3f& t){

    Eigen::VectorXf xi, xi_prev;
    RtToSE3(rot,t,xi);
    xi_prev = xi;

    Eigen::Matrix<float, 6, 6> H; //Hessian for GN optimization.
    Eigen::Matrix<float, 6, 1> inc; // step increments.

    for (int level = num_pyramid-1; level >= 1; --level){
        float error_prev = std::numeric_limits<float>::max();
        for (int itr = 0; itr < num_GNiterations; itr++) {
            // compute residuals and Jacobian
            Eigen::VectorXf residuals = calcRes(xi, level);
            Eigen::VectorXf weights;

            weighting(residuals, weights);
            residuals = residuals.cwiseProduct(weights);

            Eigen::MatrixXf J = calcJacobian(xi, level);
            // compute weighted Jacobian
            for (int i = 0; i < residuals.size(); ++i)
                for (int j = 0; j < J.cols(); ++j)
                    J(i, j) = J(i, j) * weights[i];
            Eigen::MatrixXf Jt = J.transpose();

            float error = residuals.transpose()*residuals;

            // compute update step.
            Eigen::VectorXf b = Jt * residuals;
            H = Jt * J;
            inc = -(H.ldlt().solve(b));

            xi_prev = xi;
            xi = Sophus::SE3f::log(Sophus::SE3f::exp(inc)*Sophus::SE3f::exp(xi) );

            //Break when convergence.
            if (error / error_prev > 0.995)
                break;

            error_prev = error;
        }
    }

    SE3ToRt(xi, rot, t);

    return;
}

void DirectImageAlignment::AssignImage( Eigen::Matrix4f& transform, const cv::Mat& img_prev, const cv::Mat& depth_prev,
                  const cv::Mat& img_cur, const cv::Mat& depth_cur) {

    this->K = K;
    this->img_prev = img_prev;
    this->img_cur = img_cur;
    this->depth_prev = depth_prev;
    this->depth_cur = depth_cur;

    makePyramid();

    Eigen::Matrix3f rot = transform.block<3,3>(0,0);
    Eigen::Vector3f t = transform.block<3,1>(0,3);

    doGaussNewton(rot, t);

    transform.block<3,3>(0,0) = rot;
    transform.block<3,1>(0,3) = t;

    return;
}


