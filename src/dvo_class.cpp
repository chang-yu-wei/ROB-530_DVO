#include "dvo_class.h"

/**
 * @brief Generate colored pointcloud from rgb/depth image.
 */
void DVO::makePointCloud( const cv::Mat& img_rgb, const cv::Mat& img_depth, pcl::PointCloud< pcl::PointXYZRGB >::Ptr& cloud) {
    cloud->header.frame_id = "/cam_origin";
    cloud->is_dense = true;
    cloud->height = img_depth.rows;
    cloud->width = img_depth.cols;
    cloud->points.resize( cloud->height*cloud->width );

    float fxi = 1.f / K(0,0), fyi = 1.f / K(1,1);
    float cx = K(0,2), cy = K(1,2);

    int idx = 0;
    float* depthdata = (float*)( &img_depth.data[0] );
    unsigned char* colordata = &img_rgb.data[0];

    for(int y = 0; y < img_depth.rows; y++ ) {
        for(int x = 0; x < img_depth.cols; x++ ) {
            pcl::PointXYZRGB& p = cloud->points[idx];
            p.z = (float)(*depthdata);
            p.x = (x - cx) * p.z * fxi;
            p.y = (y - cy) * p.z * fyi;
            depthdata++;

            int b = (*colordata++);
            int g = (*colordata++);
            int r = (*colordata++);
            int rgb = (r << 16) + (g << 8) + b;
            p.rgb = *((float*)(&rgb));

            idx++;
        }
    }

    return;
}

/**
 * @brief Subscribe images, run direct image alignment, and publish pointcloud and camera pose.
 */
void DVO::callback(const sensor_msgs::ImageConstPtr& image_rgb, const sensor_msgs::ImageConstPtr& image_depth, const sensor_msgs::CameraInfoConstPtr& info){

    //Initialization.
    K <<    info->K[0], 0.0, info->K[2],
            0.0, info->K[4], info->K[5],
            0.0, 0.0, 1.0;
    //cv_bridge is a bridge between OpenCV image and ROS image message
    cv_bridge::CvImageConstPtr img_rgb_cv_ptr = cv_bridge::toCvShare( image_rgb, "bgr8" );
    cv_bridge::CvImageConstPtr img_depth_cv_ptr = cv_bridge::toCvShare( image_depth, "32FC1" );
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    cv::Mat img_curInt, img_cur, depth_cur;
    cv::cvtColor( img_rgb_cv_ptr->image.clone(), img_curInt, CV_BGR2GRAY);
    img_curInt.convertTo(img_cur, CV_32FC1, 1.f/255.f);
    depth_cur = img_depth_cv_ptr->image.clone();

    //Run image alignment.
    ImageAlignment dia;
    if( !img_prev.empty() )
        dia.alignment( transform, img_prev, depth_prev, img_cur, depth_cur, K );

    //Update variables.
    accumulated_transform = accumulated_transform * transform.inverse();
    img_prev = img_cur.clone();
    depth_prev = depth_cur.clone();

    //Publish pointcloud.
    ros::Time timestamp = ros::Time::now();
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );
    makePointCloud( img_rgb_cv_ptr->image, img_depth_cv_ptr->image, cloud );
    pcl::transformPointCloud( *cloud, *cloud, accumulated_transform );
    pub_pointcloud = nh.advertise< pcl::PointCloud< pcl::PointXYZRGB >>( "pointcloud", 1 );
    pub_pointcloud.publish( *cloud );

    //Publish camera pose.
    tf::Transform tform;
    tform.setOrigin( tf::Vector3(accumulated_transform(0,3), accumulated_transform(1,3), accumulated_transform(2,3)));
    tf::Matrix3x3 rotation;
    rotation.setValue(
            accumulated_transform(0,0), accumulated_transform(0,1), accumulated_transform(0,2),
            accumulated_transform(1,0), accumulated_transform(1,1), accumulated_transform(1,2),
            accumulated_transform(2,0), accumulated_transform(2,1), accumulated_transform(2,2)
    );
    tform.setBasis(rotation);
    br.sendTransform(tf::StampedTransform(tform, timestamp, "cam_origin", "camera"));
    
    //Publish path
    path.header.frame_id = "/cam_origin";
    path.poses.resize(path_idx+1);
    path.poses[path_idx].pose.position.x = accumulated_transform(0,3);
    path.poses[path_idx].pose.position.y = accumulated_transform(1,3);
    path.poses[path_idx].pose.position.z = accumulated_transform(2,3);
    pub_path = nh.advertise<nav_msgs::Path>("path", 1);
    pub_path.publish(path);
    path_idx+=1;

    return;
}
