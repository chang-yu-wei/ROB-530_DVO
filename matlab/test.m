%{
- The color images are stored as 640×480 8-bit RGB images in PNG format.
- The depth maps are stored as 640×480 16-bit monochrome images in PNG format.
- The color and depth images are already pre-registered using the OpenNI
driver from PrimeSense, i.e., the pixels in the color and depth images
correspond already 1:1.
- The depth images are scaled by a factor of 5000, i.e., a pixel value
of 5000 in the depth image corresponds to a distance of 1 meter from the
camera, 10000 to 2 meter distance, etc. A pixel value of 0 means missing
value/no data.


The depth images in our datasets are reprojected into the frame of the color
camera, which means that there is a 1:1 correspondence between pixels in the
depth map and the color image.

The conversion from the 2D images to 3D point clouds works as follows. Note
that the focal lengths (fx/fy), the optical center (cx/cy), the distortion
parameters (d0-d4) and the depth correction factor are different for each
camera. The Python code below illustrates how the 3D point can be computed
from the pixel coordinates and the depth value:

fx = 525.0  # focal length x
fy = 525.0  # focal length y
cx = 319.5  # optical center x
cy = 239.5  # optical center y

factor = 5000 # for the 16-bit PNG files
# OR: factor = 1 # for the 32-bit float images in the ROS bag files

for v in range(depth_image.height):
  for u in range(depth_image.width):
    Z = depth_image[v,u] / factor;
    X = (u - cx) * Z / fx;
    Y = (v - cy) * Z / fy;

Camera          fx      fy      cx      cy      d0      d1      d2      d3      d4
(ROS default)	525.0	525.0	319.5	239.5	0.0     0.0     0.0     0.0     0.0
Freiburg 1 RGB	517.3	516.5	318.6	255.3	0.2624	-0.9531	-0.0054	0.0026	1.1633
Freiburg 2 RGB	520.9	521.0	325.1	249.7	0.2312	-0.7849	-0.0033	-0.0001	0.9172
Freiburg 3 RGB	535.4	539.2	320.1	247.6	0       0       0       0       0

For more information see: https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
%}
            

set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'DefaultAxesFontsize',18);
set(groot,'DefaultTextFontname','Times New Roman');
set(groot,'DefaultAxesFontname','Times New Roman');
close all
clear
clc

%% set global variables
global data_set_path;
global RGB_List;
global Depth_List;
global num_frames;

%% add dataset path
data_set_path = '/home/justin/class/ROB530/SLAM_project/data/rgbd_dataset_freiburg1_room/';
addpath(data_set_path);
addpath('rgbd_benchmark_tools/')

% load rgb filelist
RGB_List = import_assoc_file(strcat(data_set_path,'rgb.txt'));
Depth_List =  import_assoc_file(strcat(data_set_path,'depth.txt'));
num_frames = min(size(RGB_List,1) , size(Depth_List,1));

% read GT
GT_list = import_GT(strcat(data_set_path,'/groundtruth.txt'));

%% intrinsic parameters for FR1
fx = 517.3; fy = 516.5;	cx = 318.6;	cy = 255.3; d_factor = 5000;

[img_1, H_w1] = get_PointCloud(10,  fx, fy, cx, cy, d_factor, GT_list);
[img_2, H_w2] = get_PointCloud(14, fx, fy, cx, cy, d_factor, GT_list);

H_12_GT = H_w1\ H_w2;


%% create new class
test_dvo = rgbd_dvo();%(eye(3),[0.05,0.05,0.05]');
test_dvo.set_camera_intrinsic(fx, fy, cx, cy);
test_dvo.set_ptclouds(img_1,img_2);
test_dvo.align(H_12_GT);

pcshowpair(pctransform(img_2.ptcloud,test_dvo.tform),img_1.ptcloud)


function [out, H_GT] = get_PointCloud(idx, fx, fy, cx, cy, scaling_factor, GT_list)
global data_set_path;
global RGB_List;
global Depth_List;

rgb = imread(char(strcat(data_set_path, RGB_List(idx,2))));
out.Color = rgb;
out.image = rgb2gray(rgb);
   
% load Depth image
depth = double(imread(char(strcat(data_set_path, Depth_List(idx,2)))));
depth(depth == 0) = nan;

% compute points xyz
points = double(rgb);
U = repmat(0:size(depth,2)-1, size(depth,1), 1);
V = repmat([0:size(depth,1)-1]', 1, size(depth,2));
points(:,:,3) = depth / scaling_factor;
points(:,:,1) = (U - cx) .* points(:,:,3) ./ fx;
points(:,:,2) = (V - cy) .* points(:,:,3) ./ fy;

%point_cloud = pointCloud(points, 'Color', rgb);
point_cloud = pointCloud(points, 'Intensity', double(out.image));


out.ptcloud = point_cloud;

%% get transformation in GT
time_rgb = str2double(RGB_List(idx,1));
[~,idx_nearest_time] = min(abs(GT_list(:,1)-time_rgb));
T_gt = GT_list(idx_nearest_time,2:4)';
Quat_GT = [GT_list(idx_nearest_time,end), GT_list(idx_nearest_time,5:end-1)];
H_GT = [quat2rotm(Quat_GT),T_gt;0,0,0,1];
end

