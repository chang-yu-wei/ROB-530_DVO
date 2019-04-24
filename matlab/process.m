set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'DefaultAxesFontsize',18);
set(groot,'DefaultTextFontname','Times New Roman');
set(groot,'DefaultAxesFontname','Times New Roman');
close all
clear
clc

global data_set_path;
global initial_pose;
global init_gt_idx;
global GT_list;


%% add dataset path
data_set_path = '/home/justin/class/ROB530/SLAM_project/data/rgbd_dataset_freiburg2_desk/';
addpath(data_set_path);
addpath('rgbd_benchmark_tools/')

% read GT
GT_list = import_GT(strcat(data_set_path,'/groundtruth.txt'));

to_eval = "/home/justin/class/ROB530/SLAM_project/data/traj_log/fr2_desk/dvo_fr2_desk.txt";
result = importdata(to_eval);

%% get initial pose
[init_gt_idx, initial_pose] = get_GT(result.textdata{2,2});
H_GT_prev = initial_pose;
T_prev = initial_pose;

%% process VO trajectory
num_frames = size(result.data,1);
T_vo = zeros(num_frames,3);
for i=1:num_frames
    %[idx, H_GT] = get_GT(result.textdata{i+1,2});
    T_cur = transfer2GT_frame(result.data(i,:));
    %H_12_GT = H_GT_prev\ H_GT;
    %T_12 = T_prev\T_cur;
    %dis = dist_se3(T_12 \ H_12_GT);
    %if dis>0.1
    %    i
    %end
    %display(dis);
    T_vo(i,:) = T_cur(1:3,4)';
    %H_GT_prev = H_GT;
    %T_prev = T_cur;
end

%% plot_GT_path
f1 = figure(1);
GT_path = GT_list(init_gt_idx:end,2:4);
plot3(GT_path(:,1), GT_path(:,2), GT_path(:,3),'-k', 'DisplayName', 'Ground Truth');
hold on
plot3(T_vo(:,1), T_vo(:,2), T_vo(:,3), '-r',  'DisplayName', 'VO Estimated');
hold on

legend('groundtruth','dvo');
title('Trajectory Results of DVO vs Groundtruth');
xlabel('x');
ylabel('y');
zlabel('z');
% for i=1:size(T_vo)
%     x = [T_vo(i,1),GT_path(i,1)];
%     y = [T_vo(i,2),GT_path(i,2)];
%     z = [T_vo(i,3),GT_path(i,3)];
%    plot3(x,y,z); 
% end

function [T] = transfer2GT_frame(t)
    global initial_pose;
    
    T_cur = [quat2rotm(t(1:4)),t(5:7)'; 0,0,0,1];
    T = initial_pose*T_cur;
end

function [idx_nearest_time, H_GT] = get_GT(rgbfile)
global GT_list;
time_rgb = str2double(rgbfile(5:end-4));
[~,idx_nearest_time] = min(abs(GT_list(:,1)-time_rgb));
T_gt = GT_list(idx_nearest_time,2:4)';
Quat_GT = [GT_list(idx_nearest_time,end), GT_list(idx_nearest_time,5:end-1)];
H_GT = [quat2rotm(Quat_GT),T_gt;0,0,0,1];
end


function d = dist_se3(T)
    % se(3) matrix norm

    d = norm(logm(T),'fro');
end
