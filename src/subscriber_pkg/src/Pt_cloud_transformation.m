%% This program takes input as point cloud and transformation at the same time. Then gets another transformation after next few seconds. Now it transforms the point cloud to the new frame of reference.

clc;
clear all;
close all;

%% rosinit;
tftree = rostf;
cam0_old_to_map = getTransform(tftree, 'cam0', 'map');
pt_sub = rossubscriber('/pcl_xyzrgb');
msg= receive(pt_sub, 10);
time = rostime('now');
time.Sec = msg.Header.Stamp.Sec;
time.Nsec = msg.Header.Stamp.Nsec;