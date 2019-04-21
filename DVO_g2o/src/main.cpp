#include "dvo_class.h"

using namespace std;

int main(int argc, char** argv){
   string kf_path = "../fr2_desk";
   string data_path = "/home/zixu/Extra_Disk/Dataset/SLAM/tum_rgbd/rgbd_dataset_freiburg2_desk";
   string assoc_file = "../associations/fr2_desk.txt";
   DVO test(assoc_file, data_path, kf_path, TUM2);
   /*
   Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
   float error = test.Align_two_Frame(8,2599, T);
   cout<<error<<endl;
   Eigen::Quaternionf qT(T.block<3,3>(0,0));
   std::cout<<qT.w()<<" "<<qT.x()<<" "<<qT.y()<<" "<<qT.z()<<" ";
   std::cout<<T(0,3)<<" "<<T(1,3)<<" "<<T(2,3)<<"\n";  
   */
   test.local_BA_only();


   return 0;
}


