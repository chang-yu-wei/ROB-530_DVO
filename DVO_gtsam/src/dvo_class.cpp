#include "dvo_class.h"

DVO::DVO(string strAssociationFilename, string strDataPath, TUM type)
{
    this->AssociationFilename = strAssociationFilename;
    this->DataPath = strDataPath;
    load_intrinsic(type);
    LoadImages(strAssociationFilename);

}

void DVO::load_intrinsic(TUM type)
{
    float fx, fy, cx, cy;
    switch (type)
    {
    case TUM1:
        fx = 517.306408;
        fy = 516.469215;
        cx = 318.643040;
        cy = 255.313989;
        break;
    case TUM2:
        fx = 520.908620;
        fy = 521.007327;
        cx = 325.141442;
        cy = 249.701764;
        break;
    case TUM3:
        fx = 535.4;
        fy = 539.2;
        cx = 320.1;
        cy = 247.6;
        break;
    default:
        fx = 517.306408;
        fy = 516.469215;
        cx = 318.643040;
        cy = 255.313989;
        break;
    }
    K << fx, 0, cx,
        0, fy, cy, 
        0 ,0 ,1;
    cout<<"Successfully load intrinsic matrix"<<endl;
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
    cout<<"Successfully reading association file, there are "<<nImages<<" in data set."<<endl;;
}

Eigen::Matrix4f DVO::incr_Align_KF(int prev_KF_idx, int cur_KF_idx)
{
    Eigen::Matrix4f accumulated_transform = Eigen::Matrix4f::Identity();
    cv::Mat  img_cur,  depth_cur, Gray_read, depth_read;
    if(cur_KF_idx <= prev_KF_idx)
    {
        cout<<"Current keyframe index is smaller than previous one"<<endl;
        return accumulated_transform;
    }
    if(prev_KF_idx != prev_idx){
        Gray_read = cv::imread(DataPath+"/"+vstrImageFilenamesRGB[prev_KF_idx],CV_LOAD_IMAGE_GRAYSCALE);
        Gray_read.convertTo(img_prev, CV_32FC1, 1.f/255.f);
        depth_read= cv::imread(DataPath+"/"+vstrImageFilenamesD[prev_KF_idx],CV_LOAD_IMAGE_UNCHANGED);
        depth_read.convertTo(depth_prev, CV_32FC1, 1.f/5000.f);
    }
    for(int ni=prev_KF_idx+1; ni<=min(cur_KF_idx,nImages-1); ni++)
    {
        // Read image and depthmap from file
        Gray_read = cv::imread(DataPath+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_GRAYSCALE);
        Gray_read.convertTo(img_cur, CV_32FC1, 1.f/255.f);
        depth_read = cv::imread(DataPath+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);        
        depth_read.convertTo(depth_cur, CV_32FC1, 1.f/5000.f);
        
        Eigen::Matrix4f T_cur = Eigen::Matrix4f::Identity();
        backend.alignment(T_cur, img_prev, depth_prev,  img_cur,  depth_cur, K);

        accumulated_transform = accumulated_transform*T_cur.inverse();
        img_prev =  img_cur.clone();
        depth_prev =  depth_cur.clone();
        prev_idx = ni;
    }
    return accumulated_transform;
}

Eigen::Matrix4f DVO::Align_two_Frame(int Frame1, int Frame2)
{
    cv::Mat  img_cur,  depth_cur, Gray_read, depth_read;
    if(Frame1 != prev_idx){
        Gray_read = cv::imread(DataPath+"/"+vstrImageFilenamesRGB[Frame1],CV_LOAD_IMAGE_GRAYSCALE);
        Gray_read.convertTo(img_prev, CV_32FC1, 1.f/255.f);
        depth_read= cv::imread(DataPath+"/"+vstrImageFilenamesD[Frame1],CV_LOAD_IMAGE_UNCHANGED);
        depth_read.convertTo(depth_prev, CV_32FC1, 1.f/5000.f);
    }
    // Read image and depthmap from file
    Gray_read = cv::imread(DataPath+"/"+vstrImageFilenamesRGB[Frame2],CV_LOAD_IMAGE_GRAYSCALE);
    Gray_read.convertTo(img_cur, CV_32FC1, 1.f/255.f);
    
    depth_read = cv::imread(DataPath+"/"+vstrImageFilenamesD[Frame2],CV_LOAD_IMAGE_UNCHANGED);        
    depth_read.convertTo(depth_cur, CV_32FC1, 1.f/5000.f);
    
    Eigen::Matrix4f T_cur = Eigen::Matrix4f::Identity();
    backend.alignment(T_cur, img_prev, depth_prev,  img_cur,  depth_cur, K);

    // save for future usage
    img_prev =  img_cur.clone();
    depth_prev =  depth_cur.clone();
    prev_idx = Frame2;

    return T_cur.inverse();;
}

 void DVO::build_graph(vector<int> KF, vector<vector<int>> Loop_Closure)
 {
     KF_list = KF;
     Loop_list = Loop_Closure;
   Eigen::Matrix4f accumulated_transform = Eigen::Matrix4f::Identity();
    // add first KF as fixed
    optimizer.addVertex(KF_list[0], accumulated_transform.cast<double>());
    // first insert odometry constraint
     for(int i=1; i<KF_list.size(); i++)
     {
         Eigen::Matrix4f T = incr_Align_KF(KF_list[i-1], KF_list[i]);
         accumulated_transform = accumulated_transform*T;
         optimizer.addVertex(KF_list[i], accumulated_transform.cast<double>());
         optimizer.addOdom(KF_list[i-1], KF_list[i], T.cast<double>());
     }

    Eigen::Quaternionf qT(accumulated_transform.block<3,3>(0,0));
    cout<<qT.w()<<" "<<qT.x()<<" "<<qT.y()<<" "<<qT.z()<<endl;
    cout<<accumulated_transform.block<3,1>(0,3)<<endl;

    usleep(100);
   
   
     // insert loop constraint
     for(int i=1; i<Loop_Closure.size(); i++)
     {
         vector<int> cur_pair = Loop_Closure[i];
         Eigen::Matrix4f T = Align_two_Frame(cur_pair[0], cur_pair[1]);
         optimizer.addOdom(cur_pair[0], cur_pair[1], T.cast<double>());
     }
     
     cout<<endl<<endl<<"Start Optimization"<<endl<<endl;
     optimizer.optimize();
     optimizer.results.at(gtsam::Symbol('x',30)).print();
     
 }