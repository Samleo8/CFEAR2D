% -------------------------------------------------------------------------
% Plot 2D Trajectory
% -------------------------------------------------------------------------
clc; clearvars; close all

%% Ground Truth
gt = readmatrix('radar_odometry.csv'); % groundtruth
N = size(gt, 1); % number of frames

delta_x_vec   = -gt(:, 3); % relative pose between two frames along X-coordinate
delta_y_vec   = -gt(:, 4); % relative pose between two frames along Y-coordinate
delta_yaw_vec = -gt(:, 8); % relative rotation angle between two frames

coord = [0; 0];
yaw = 0;

figure
for i = 1 : N
    h0 = plot(coord(1), coord(2), 'k.');
    hold on
    
    delta_x   = delta_x_vec(i);
    delta_y   = delta_y_vec(i);
    delta_yaw = delta_yaw_vec(i);
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    
    coord = coord + R * [delta_x; delta_y];
    yaw = yaw + delta_yaw;
end


%% Estimations
es_1 = readmatrix('Estimate_Discrete.txt');
N = size(es_1, 1);
x_vec   = -es_1(:, 2);
y_vec   = es_1(:, 3);

for i = 1 : N
    h1 = plot(x_vec(i), y_vec(i), 'b.');
    hold on
end


es_2 = readmatrix('Estimate_Continuous.txt');
N = size(es_2, 1);
x_vec   = -es_2(:, 2);
y_vec   = es_2(:, 3);

for i = 1 : N
    h2 = plot(x_vec(i), y_vec(i), 'r.');
    hold on
end


legend([h0, h1, h2], 'Groundtruth', 'CFEAR', 'CT-CFEAR', 'FontSize', 15, 'location', 'northeast')
xlabel('x(m)')
ylabel('y(m)')
grid on