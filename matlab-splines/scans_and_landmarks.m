% Viewing radar scans to scale
% Roberto Aldera - ORI - 31 Jan 2020
% Need to run landmark_comparison.m first, and make sure frames are synced
% close all
% clear all
% clearvars

%%
addpath('helpers')
addpath('/Users/roberto/code/MATLAB-scripts/mains')
addpath('/Users/roberto/code/MATLAB-scripts/helpers/convert_format')

%%
radar_mono_path = ['/Users/roberto/data/RadarDataLogs/2017-08-18-11-21-04-oxford-10k-with-radar-1',...
    '/logs/radar/cts350x/2017-08-18-10-21-06/cts350x_raw_scan.monolithic'];

%% Load raw Navtech data
num_time_points = 1;
frame_idx_start = 7880+10;
radar_sweeps = LoadRadarData(radar_mono_path,num_time_points,frame_idx_start);
% num_time_points = size(radar_sweeps,1);

%%
% dataset_split = strsplit(radar_mono_path,'/');
% dataset = dataset_split{6};
% fn_save = sprintf('data/%s-FILTERED-POINTCLOUDS.mat',dataset);

%%
tic

t = 1;
polar_img = radar_sweep_to_polar_img(radar_sweeps{t}); % THIS IS EFFECTIVELY A MATRIX
cart_img = polar_img_to_cart_img(polar_img);

% figure(),imshow(polar_img/256)
% p = get(0, 'MonitorPositions');
% f1 = figure('Position', [-930   -51   814   771]);
figure(2);
clf;
imshow(cart_img/256);
hold on;
offset = 1412/2;
scale = 4;
% Rotation
theta = 2.5;

ro_landmarks_x = (-ro_landmarks.xyz(:,1)*cos(theta) - ro_landmarks.xyz(:,2)*sin(theta)) * scale + offset-3; %(offset+(-ro_landmarks.xyz(:,1))*scale*cos(theta));
ro_landmarks_y = (-ro_landmarks.xyz(:,1)*sin(theta) + ro_landmarks.xyz(:,2)*cos(theta)) * scale + offset+3;
plot(ro_landmarks_x,ro_landmarks_y,'.','MarkerSize',12,'MarkerEdgeColor','r')

interpolation_landmarks_x = (-ro_landmarks_interpolation_corrected.xyz(:,1)*cos(theta) - ro_landmarks_interpolation_corrected.xyz(:,2)*sin(theta)) * scale + offset-3; %(offset+(-ro_landmarks.xyz(:,1))*scale*cos(theta));
interpolation_landmarks_y = (-ro_landmarks_interpolation_corrected.xyz(:,1)*sin(theta) + ro_landmarks_interpolation_corrected.xyz(:,2)*cos(theta)) * scale + offset+3;
plot(interpolation_landmarks_x,interpolation_landmarks_y,'.','MarkerSize',12,'MarkerEdgeColor','w')

spline_landmarks_x = (-ro_landmarks_spline_corrected.xyz(:,1)*cos(theta) - ro_landmarks_spline_corrected.xyz(:,2)*sin(theta)) * scale + offset-3; %(offset+(-ro_landmarks.xyz(:,1))*scale*cos(theta));
spline_landmarks_y = (-ro_landmarks_spline_corrected.xyz(:,1)*sin(theta) + ro_landmarks_spline_corrected.xyz(:,2)*cos(theta)) * scale + offset+3;
plot(spline_landmarks_x,spline_landmarks_y,'.','MarkerSize',12,'MarkerEdgeColor',[.3 .6 1])

% plot(50,100, 'r.', 'MarkerSize', 30, 'LineWidth', 2);

toc