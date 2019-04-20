#include "dvo_class.h"

using namespace std;

int main(int argc, char** argv){
   string data_path = "/home/zixu/Extra_Disk/Dataset/SLAM/tum_rgbd/rgbd_dataset_freiburg1_desk2";
   string assoc_file = "../associations/fr1_desk2.txt";
   DVO test(assoc_file, data_path, TUM1);
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