#include "dvo_class.h"

using namespace std;

int main(int argc, char** argv){
<<<<<<< HEAD
   string data_path = "/home/zixu/Extra_Disk/Dataset/SLAM/tum_rgbd/rgbd_dataset_freiburg2_desk";
   string assoc_file = "../associations/fr2_desk.txt";
   DVO test(assoc_file, data_path, TUM2);
=======
   string data_path = "/home/justin/class/ROB530/SLAM_project/data/rgbd_dataset_freiburg1_room";
   string assoc_file = "/home/justin/class/ROB530/SLAM_project/data/associations/fr1_room.txt";
   string kf_file = "/home/justin/class/ROB530/SLAM_project/data/keyframes/KeyFrameTrajectoryTUM_fr1_desk.txt";
   DVO test(assoc_file, data_path, kf_file, TUM2);
>>>>>>> cf7b9d36867f54a74279f45fc35226c70d13d57c
   test.odom_only(0,-1);
   // vector<int> KF_list;
   // vector<vector<int>> Loop_list;

   // for(int i=3; i<=30; i+=3)
   // {
   //    KF_list.push_back(i);
   // }
   // vector<int> temp;
   // temp.push_back(0);
   // temp.push_back(3);
   // Loop_list.push_back(temp);
   // temp.clear();
   // temp.push_back(3);
   // temp.push_back(30);
   // Loop_list.push_back(temp);
   // test.build_graph(KF_list, Loop_list);


   return 0;
}