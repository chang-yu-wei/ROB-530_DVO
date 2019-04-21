#include "dvo_class.h"

DVO::DVO(string strAssociationFilename, string strDataPath, TUM type)
{
    this->AssociationFilename = strAssociationFilename;
    this->DataPath = strDataPath;
    load_intrinsic(type);
    LoadImages(strAssociationFilename);
    setupLog();
}

DVO::DVO(string strAssociationFilename, string strDataPath,string strKfFoldername, TUM type)
{
    
    this->AssociationFilename = strAssociationFilename;
    this->DataPath = strDataPath;
    this->KfFoldername = strKfFoldername;
    load_intrinsic(type);
    LoadImages(strAssociationFilename);
    nKFs = 0;
    boost::filesystem::path KF_path(strKfFoldername);
    nKFs = std::count_if(
        boost::filesystem::directory_iterator(KF_path),
        boost::filesystem::directory_iterator(),
        static_cast<bool(*)(const boost::filesystem::path&)>(boost::filesystem::is_regular_file) );
    setupLog();
}
DVO::~DVO()
{
    logfile.close();
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

vector<vector<int>> DVO::LoadKF(int idx)
{
    string strKfFoldername;
    strKfFoldername = this->KfFoldername+"/Local_BA_"+std::to_string(idx)+".txt";
    vector<vector<int>> local_BA_covis;
    ifstream fAssociation;
    fAssociation.open(strKfFoldername.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            vector<int> temp;
            stringstream ss;
            ss << s;
            int kf_id, img_id;
            ss >> kf_id;
            ss >> img_id;
            temp.push_back(kf_id);
            temp.push_back(img_id);
            local_BA_covis.push_back(temp);
        }
    }
    return local_BA_covis;
}

Eigen::Matrix4f DVO::incr_Align_KF(int prev_KF_idx, int host_KF_idx)
{
    Eigen::Matrix4f accumulated_transform = Eigen::Matrix4f::Identity();

    cv::Mat  img_cur,  depth_cur, Gray_read, depth_read;
    if(host_KF_idx <= prev_KF_idx)
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
    for(int ni=prev_KF_idx+1; ni<=min(host_KF_idx,nImages-1); ni++)
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

float DVO::Align_two_Frame(int Frame1, int Frame2, Eigen::Matrix4f& T_init)
{
    cv::Mat  img_cur,  depth_cur, Gray_read, depth_read;
    //std::cout<<vstrImageFilenamesRGB[Frame1]<<" "<<vstrImageFilenamesRGB[Frame2]<<std::endl;
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
    
    Eigen::Matrix4f T_cur = T_init;
    backend.alignment(T_cur, img_prev, depth_prev,  img_cur,  depth_cur, K);
    T_init = T_cur.inverse();
    // save for future usage
    img_prev =  img_cur.clone();
    depth_prev =  depth_cur.clone();
    prev_idx = Frame2;
    //std::cout<<"Error "<<backend.getError()<<std::endl;

    return backend.getError();
}

//  void DVO::build_graph(vector<int> KF, vector<vector<int>> Loop_Closure)
//  {
//      KF_list = KF;
//      Loop_list = Loop_Closure;
//     Eigen::Matrix4f accumulated_transform = Eigen::Matrix4f::Identity();
//     // add first KF as fixed
//     optimizer.add_Vertex(accumulated_transform.cast<double>(), KF[0], true);
//     // first insert odometry constraint
//      for(uint i=1; i<KF_list.size(); i++)
//      {
//          Eigen::Matrix4f T = incr_Align_KF(KF_list[i-1], KF_list[i]);
//          accumulated_transform = accumulated_transform*T;
//          optimizer.add_Vertex(accumulated_transform.cast<double>(), KF_list[i], false);
//          optimizer.add_Edge(KF_list[i-1], KF_list[i], T.cast<double>(), 1);
//      }
    
//      // insert loop constraint
//      for(uint i=1; i<Loop_Closure.size(); i++)
//      {
//          vector<int> cur_pair = Loop_Closure[i];
//          Eigen::Matrix4f T_w1 = optimizer.get_Pose_global(cur_pair[0]).cast<float>();
//          Eigen::Matrix4f T_w2 = optimizer.get_Pose_global(cur_pair[1]).cast<float>();
//          Eigen::Matrix4f T = Align_two_Frame(cur_pair[0], cur_pair[1], T_w2.inverse()*T_w1);
//          optimizer.add_Edge(cur_pair[0], cur_pair[1], T.cast<double>(), 0.5);
//      }
     
//     optimizer.optimize_graph(100);
     
//  }

 void DVO::odom_only(int start_idx, int end_idx)
 {
    if( end_idx ==-1)
        end_idx = nImages-1;
    Eigen::Matrix4f accumulated_transform = Eigen::Matrix4f::Identity();
    LogInfo(start_idx, accumulated_transform);
    for(int idx = start_idx; idx<min(end_idx,nImages-1); idx++)
    {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        Align_two_Frame(idx, idx+1, T);
        accumulated_transform = accumulated_transform*T;
        
        //std::cout<<idx+1<<std::endl;
        //std::cout<<T<<std::endl;
        LogInfo(idx+1, accumulated_transform);
    }
 }

void DVO::local_BA_only()
{
    // kf idx start from 2 
    for(int i = 2; i<=50; i++)
    //for(int i = 2; i<=(nKFs+1); i++)
    {
        vector<vector<int>> covisible = LoadKF(i);
        if(i==2)
        { //initialize the system
            for(auto & itr : covisible)
            {
                KF_frame_id[itr[0]] = itr[1];
            }
            KF_pose[0] = Eigen::Matrix4f::Identity();
            KF_pose[1] = incr_Align_KF(KF_frame_id[0], KF_frame_id[1]);
            KF_pose[2] = KF_pose[1]*incr_Align_KF(KF_frame_id[1], KF_frame_id[2]);
        }else
        {
            //initialize new KF
            vector<int> host_frame = covisible[0];
            int host_KF_id = host_frame[0];
            int host_frame_id = host_frame[1];
            KF_frame_id[host_KF_id] = host_frame_id;
            KF_pose[host_KF_id] = KF_pose[host_KF_id-1]*incr_Align_KF(KF_frame_id[host_KF_id-1], KF_frame_id[host_KF_id]);
        }
        BundleAdjust(covisible);
    }
    for(int i=0; i<KF_frame_id.size(); i++)
    {
        LogInfo(KF_frame_id[i], KF_pose[i]);
    }    
}

void DVO::BundleAdjust(vector<vector<int>> graph)
{
    Optimizer optimizer;
    vector<int> host_frame = graph[0];
    int host_KF_id = host_frame[0];
    int host_frame_id = host_frame[1];
    int num_active_vertex = graph.size();
    map<int, int> vertex_existed;
    // first add host_vertex
    optimizer.add_Vertex(KF_pose[host_KF_id].cast<double>(), host_KF_id , false);
    vertex_existed[host_KF_id] = 1;

    for(int i=1; i<num_active_vertex; i++)
    {
        vector<int> cur_frame = graph[i];
        int cur_KF_id = cur_frame[0];
        int cur_frame_id = cur_frame[1];
        Eigen::Matrix4f T = (KF_pose[cur_KF_id].inverse()*KF_pose[host_KF_id]);
        float error = Align_two_Frame(host_frame_id, cur_frame_id, T);
        if(error<30){
            vertex_existed[cur_KF_id] = 1;
            optimizer.add_Vertex(KF_pose[cur_KF_id].cast<double>(), cur_KF_id , cur_KF_id==0);
            optimizer.add_Edge(cur_KF_id, host_KF_id, T.cast<double>(), 16/(error*error));
            //std::cout<<"Add Edge "<<host_KF_id<<" to "<<cur_KF_id<<std::endl;
        }
    }
    
    for(int i=0; i<num_active_vertex; i++){
        // add adjanct vertex
        vector<int> cur_frame = graph[i];
        int cur_KF_id = cur_frame[0];
        int cur_frame_id = cur_frame[1];
        if(vertex_existed.find(cur_KF_id)!=vertex_existed.end()){
            if((cur_KF_id>0) && vertex_existed.find(cur_KF_id-1)==vertex_existed.end())
            {
                optimizer.add_Vertex(KF_pose[cur_KF_id-1].cast<double>(), cur_KF_id-1, true);
                //std::cout<<"Add Fixed vertex "<<cur_KF_id-1<<std::endl;
                optimizer.add_Edge(cur_KF_id-1, cur_KF_id, (KF_pose[cur_KF_id].inverse()*KF_pose[cur_KF_id-1]).cast<double>(), 2);
                //std::cout<<"Add Edge "<<cur_KF_id-1<<" to "<<cur_KF_id<<std::endl;
            }
            if(KF_pose.find(cur_KF_id+1)!=KF_pose.end())
            {
                if(vertex_existed.find(cur_KF_id+1)==vertex_existed.end())
                {
                    optimizer.add_Vertex(KF_pose[cur_KF_id+1].cast<double>(), cur_KF_id+1, true);
                    //std::cout<<"Add FIxed vertex "<<cur_KF_id+1<<std::endl;
                }
                optimizer.add_Edge(cur_KF_id, cur_KF_id+1, (KF_pose[cur_KF_id+1].inverse()*KF_pose[cur_KF_id]).cast<double>(), 2);
                //std::cout<<"Add Edge "<<cur_KF_id<<" to "<<cur_KF_id+1<<std::endl;
            }
        }
    }
    optimizer.optimize_graph(100);
    for(int i=0; i<num_active_vertex; i++){
        // add adjanct vertex
        
        vector<int> cur_frame = graph[i];
        int cur_KF_id = cur_frame[0];
        if(vertex_existed.find(cur_KF_id)!=vertex_existed.end())
            KF_pose[cur_KF_id] = optimizer.get_Pose_global(cur_KF_id).cast<float>();    
    }
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
    logname = root_dir+string("/log_")+string(buff)+".txt";
    //sprintf(logname, "%slog_%s.txt", root_dir, buff);
    logfile.open(logname);
    logfile<<"Idx RGB_file Qw Qx Qy Qz Tx Ty Tz\n";
}

void DVO::LogInfo(int frame_idx, Eigen::Matrix4f T)
{
    Eigen::Quaternionf qT(T.block<3,3>(0,0));
    logfile<<frame_idx<<" "<<vstrImageFilenamesRGB[frame_idx]<<" ";
    logfile<<qT.w()<<" "<<qT.x()<<" "<<qT.y()<<" "<<qT.z()<<" ";
    logfile<<T(0,3)<<" "<<T(1,3)<<" "<<T(2,3)<<"\n";  
    logfile.flush();
}