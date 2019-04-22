#include "dvo_class.h"

using namespace std;

int main(int argc, char** argv){
   string kf_path = "../fr3_office_val";
   string data_path = "/home/zixu/Extra_Disk/Dataset/SLAM/tum_rgbd/rgbd_dataset_freiburg3_long_office_household_validation";
   string assoc_file = "../associations/fr3_office_val.txt";
   DVO test(assoc_file, data_path, kf_path, TUM3);
   //test.local_BA_only();
   test.PoseGraph();
   //test.odom_only(0,-1);
/*
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
   float error = test.Align_two_Frame(99,128, T);
   cout<<error<<endl;
   Eigen::Quaternionf qT(T.block<3,3>(0,0));
   std::cout<<qT.w()<<" "<<qT.x()<<" "<<qT.y()<<" "<<qT.z()<<" ";
   std::cout<<T(0,3)<<" "<<T(1,3)<<" "<<T(2,3)<<"\n";  
   */
   return 0;
}


