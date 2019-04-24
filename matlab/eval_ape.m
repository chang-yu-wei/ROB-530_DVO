global data_set_path;
global GT_list;

% add data path
data_set_path = '/home/justin/class/ROB530/SLAM_project/data/rgbd_dataset_freiburg2_desk/';
addpath(data_set_path);
addpath('rgbd_benchmark_tools/')

% read GT
GT_list = import_GT(strcat(data_set_path,'/groundtruth.txt'));

to_eval = "/home/justin/class/ROB530/SLAM_project/data/log/fr2_desk/dvo_fr2_desk.txt";
result = importdata(to_eval);


