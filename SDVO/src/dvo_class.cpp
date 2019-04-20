#include "dvo_class.h"


DVO::DVO(ros::NodeHandle nh_input, string strAssociationFilename, string strDataPath){
            nh = nh_input;
            path_idx = 0;
            img_idx = 0;
            this->AssociationFilename = strAssociationFilename;
            this->DataPath = strDataPath;
            std::cout << "Data path imported: " << DataPath << std::endl;
            std::cout << "AssociationFilename: " << AssociationFilename << std::endl;
            std::cout << "Loading images..." << std::endl;
            LoadImages(AssociationFilename);
            std::cout << "Finished loading images!!" << std::endl;
            setupLog();
        }

DVO::~DVO()
{
    logfile.close();
}

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

void DVO::LoadImages(const string &strAssociationFilename)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {   
        
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
    nImages = vstrImageFilenamesRGB.size();
    std::cout<<"Successfully reading association file, there are "<<nImages<<" in data set."<<std::endl;;
}

 void DVO::setupLog()
{
    //get current time
    char buff[20];
    struct tm *sTm;
    time_t now = time (0);
    sTm = localtime (&now);
    strftime (buff, sizeof(buff), "%Y_%m_%d_%H_%M_%S", sTm);
    
    boost::filesystem::path canonicalPath = boost::filesystem::canonical(".", boost::filesystem::current_path());
    //create log root
    string root_dir = canonicalPath.string();
    
    string logname;
    logname = DataPath+string("/../log_")+string(buff)+".txt";
    
    //sprintf(logname, "%slog_%s.txt", root_dir, buff);
    logfile.open(logname);
    logfile<<"Idx RGB_file Qw Qx Qy Qz Tx Ty Tz\n";
}

void DVO::LogInfo(int frame_idx, Eigen::Matrix4f T)
{
    std::cout<<frame_idx<<" "<<vstrImageFilenamesRGB[frame_idx]<<" "<<std::endl;
    Eigen::Quaternionf qT(T.block<3,3>(0,0));
    logfile<<frame_idx<<" "<<vstrImageFilenamesRGB[frame_idx]<<" ";
    logfile<<qT.w()<<" "<<qT.x()<<" "<<qT.y()<<" "<<qT.z()<<" ";
    logfile<<T(0,3)<<" "<<T(1,3)<<" "<<T(2,3)<<"\n";  
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
    ImageAlignment img_align;
    if( !img_prev.empty() )
        img_align.alignment(transform, img_prev, depth_prev, img_cur, depth_cur, K);

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

    //setup output log
    LogInfo(img_idx, accumulated_transform);
    img_idx += 1;

    return;
}
