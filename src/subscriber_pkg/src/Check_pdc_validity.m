%%
%% This code takes root point cloud, a transformation between root pose and current pose
%% and a tranformed root point cloud in current pose. It manually transforms the point-
%% cloud to check whether the inbuilt transformation is currect or not! 
%% The transformation has to be manually loaded from transformations.txt file which is
%% inside the same directory as this code. For example if the output_pt_cloud is Output_100.ply,
%% you have to load transformation from the following two lines.
%% 79 Rotation q(w,x,y,z) = (0.985941, 0.00329723, 0.0839304, 0.14445).
%%  Translation t(x,y,z) = (-3.4594, 0.105345, 0.380942)

clc;
clear all;
close all;

ptcloud_input_original = pcread('/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg/Point_clouds/Input_pcd_original.ply');
ptcloud_input = pcread('/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg/Point_clouds/Input_pcd.ply');
ptcloud_transformed = pcread('/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg/Point_clouds/Output_79.ply');

quaternion_rotation = [0.985941, 0.00329723, 0.0839304, 0.14445];
quaternion_translation = [-3.4594, 0.105345, 0.380942];

rotation_matrix = quat2rotm(quaternion_rotation);
input_points = ptcloud_input.Location(:,:);
input_points_original = ptcloud_input_original.Location(:,:);
transforming_points = input_points_original;
transforming_points(:,1) = input_points_original(:,3);
transforming_points(:,2) = -input_points_original(:,1);
transforming_points(:,3) = -input_points_original(:,2);
output_points = transpose(rotation_matrix*transpose(input_points) + transpose(repmat(quaternion_translation, length(input_points), 1)));


%pcshow(ptcloud_input);
%pcshow(ptcloud_transformed);
%pcshowpair(ptcloud_input_original, ptcloud_transformed)

%% This code proves that the inverse transformation carried out is correct.

max(ptcloud_transformed.Location(:,1))
min(output_points(:,2))
max(ptcloud_transformed.Location(:,2))
-min(output_points(:,3))
max(ptcloud_transformed.Location(:,3))
max(output_points(:,1))
min(ptcloud_transformed.Location(:,1))
-max(output_points(:,2))
min(ptcloud_transformed.Location(:,2))
-max(output_points(:,3))
min(ptcloud_transformed.Location(:,3)) 
min(output_points(:,1))

%% Reconstructing new image

x = 432.0*(ptcloud_transformed.Location(:,1)./ptcloud_transformed.Location(:,3)) + 144;
y = 432.0*(ptcloud_transformed.Location(:,2)./ptcloud_transformed.Location(:,3)) + 144;

ptcloud_transformed = removeInvalidPoints(ptcloud_transformed)
x = 432.0*(ptcloud_transformed.Location(:,1)./ptcloud_transformed.Location(:,3)) + 144;
y = 432.0*(ptcloud_transformed.Location(:,2)./ptcloud_transformed.Location(:,3)) + 144;
indices = find(x < 288 & x > 1 & y > 1 & y < 288);
image=zeros(288,288,3); %initialize

for i = 1:length(indices)
    image(round(x(indices(i)), 0), round(y(indices(i)),0), :) = ptcloud_transformed.Color(indices(i),:);
end