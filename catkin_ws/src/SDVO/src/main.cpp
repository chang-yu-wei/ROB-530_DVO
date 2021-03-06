#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "dvo_class.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SP;

using namespace std;

int main(int argc, char** argv){
    if(argc != 4){
        std::cout<<"argc: "<<argc<<std::endl;
        for(int i=0;i<argc;i++){
           std::cout<<"argc"<<i<<": "<<argv[i]<<std::endl; 
        }
        std::cout<<"Usage: rosrun dvo run_dvo --image_color --image_depth --camera_info"<<std::endl;
        // return EXIT_FAILURE;
    }
    // data path
    string data_path = "/home/justin/class/ROB530/SLAM_project/data/rgbd_dataset_freiburg1_room";
    string assoc_file = "/home/justin/class/ROB530/SLAM_project/data/associations/fr1_room.txt";
    string bag_file = "/home/justin/class/ROB530/SLAM_project/data/rgbd_dataset_freiburg2_desk.bag";
    string strCommand = "rosbag play ";

    ros::init(argc, argv, "dvo");
    ros::NodeHandle nh;
    DVO dvo(nh, assoc_file, data_path);

    std::cout<<"finshed initialzation!"<<std::endl;

    message_filters::Subscriber<sensor_msgs::Image> sub_img_RGB(nh, argv[1], 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_depth(nh, argv[2], 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera(nh, argv[3], 1);
    message_filters::Synchronizer<SP> sync(SP(10), sub_img_RGB, sub_img_depth, sub_camera);
    sync.registerCallback(boost::bind(&DVO::callback, &dvo, _1, _2, _3));
    
    ros::spin();

    return EXIT_SUCCESS;
}