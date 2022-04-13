
% Spline fusion: Lovegrove 2013
% Trying to see if we can reproduce weird offset seen when implementing on
% real data in C++
% Roberto Aldera - 6 Jan 2020
clc;clear
%% Setting up transforms
theta = -pi/160;
R = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
a = [1; 0.1; 0]; % position of origin frame a in reference frame b
T0 = [ R a; 0 0 0 1];

T = {T0; T0; T0; T0; T0; T0; T0; T0}; % pretending the transforms are all the same for now

% Offset the points to make it more interesting
% T{1}(1,4) = T{1}(1,4) + 2;
% T{2}(1,4) = T{2}(1,4) + 1;
% T{3}(1,4) = T{3}(1,4) - 0.3;
% T{3}(2,4) = T{3}(2,4) - 0.5;
% T{4}(1,4) = T{4}(1,4) + 5; %0.3;
% T{5}(1,4) = T{5}(1,4) + 5; %0.3;
% T{4}(2,4) = T{3}(2,4) - 0.2;
T{6}(1,4) = T{6}(1,4) + 0.2;
% T{6}(2,4) = T{6}(2,4) - 1;
% T{8}(1,4) = T{8}(1,4) + 0.1;
% T{8}(2,4) = T{8}(2,4) - 3;

T_world = cell(size(T,1),1);
T_world{1} = eye(4);
% T_world{1} = T{1};
for i = 2:size(T,1)
    T_world{i,1} = T_world{i-1} * T{i};
end

Omega = cell(size(T,1)-1,1); % This holds all omegas, but each spline segment only uses 3 of them at a time (a window)
for i = 1:size(T,1)-1 
    Omega{i} = logm(T_world{i}^-1 * T_world{i+1});
end

%% Spline fusion parameters
num_intervals = size(T,1)-3;
num_spline_points = 10;
C = 1/6 * [ 6 0 0 0; 5 3 -3 1; 1 3 3 -2; 0 0 0 1 ];
T_w_s = cell(num_intervals,num_spline_points);

s_i = linspace(0,size(T,1)-1,size(T,1));

for i = 1:num_intervals
    s_t = linspace(i,i+1,num_spline_points);
    for t = 1:num_spline_points
        u = 0;%s_t(t) - s_i(i+1);
        u_vector = [1; u; u^2; u^3];
        B = C * u_vector;

%         T_w_s{i,t} = T_world{i} * expm(B(2) * Omega{i}) ...
%             * expm(B(3) * Omega{i+1}) * expm(B(4) * Omega{i+2});
          T_w_s{i,t} = expm(B(2) * Omega{i}) ...
              * expm(B(3) * Omega{i+1}) * expm(B(4) * Omega{i+2});
    end
end

%% Plotting
figure(1);
clf;
hold on;
grid on;
axis equal;
title("Spline fusion using cumulative cubic B-Splines and off-spline predictions");

for i = 1:size(T,1)
    scatter(T_world{i}(1,4),T_world{i}(2,4),1500,'r.');
end

for i = 1:num_intervals
    for j = 1:size(T_w_s,2)
        scatter(T_w_s{i,j}(1,4),T_w_s{i,j}(2,4),100,'b.');
    end
end

%% Spline extentions
% num_pred_points = 10;
% for i = 1:num_intervals
%     t_s_x = [T_w_s{i,num_spline_points-2}(1,4),T_w_s{i,num_spline_points-1}(1,4),T_w_s{i,num_spline_points}(1,4)];
%     t_s_y = [T_w_s{i,num_spline_points-2}(2,4),T_w_s{i,num_spline_points-1}(2,4),T_w_s{i,num_spline_points}(2,4)];
%     order_1_pred = polyfit([T_w_s{i,num_spline_points-1}(1,4),T_w_s{i,num_spline_points}(1,4)], ... 
%         [T_w_s{i,num_spline_points-1}(2,4),T_w_s{i,num_spline_points}(2,4)],1);
%     order_2_pred = polyfit(t_s_x,t_s_y,2);
%     t = linspace(T_world{i+2}(1,4),T_world{i+3}(1,4),num_pred_points);
%     plot(t,polyval(order_1_pred,t),'--o','Color',[1 0 1],'Linewidth',2,'MarkerIndices',[1,num_pred_points]);
%     plot(t,polyval(order_2_pred,t),'--o','Color',[0 1 1],'Linewidth',2,'MarkerIndices',[1,num_pred_points]);
%     
% 
% end

%% Polyfit comparisons
% degree = 3;
% p = polyfit([T_world{3}(1,4),T_world{4}(1,4),T_world{5}(1,4),T_world{6}(1,4)], ... 
%     [T_world{3}(2,4),T_world{4}(2,4),T_world{5}(2,4),T_world{6}(2,4)],degree);
% interpolations = polyval(p,linspace(T_world{4}(1,4),T_world{5}(1,4),num_spline_points));
% plot(linspace(T_world{4}(1,4),T_world{5}(1,4),num_spline_points),interpolations);
% 
% diffs = [];
% for i = 1:size(T_w_s,1)
%     diffs(i) = T_w_s{i}(2,4) - interpolations(i);
% end

%% Plot connections directly between control points
T_world_coords = zeros(size(T_world,1),2);
for i = 1:size(T_world)
    T_world_coords(i,:) = [T_world{i}(1,4) T_world{i}(2,4)];
end
plot(T_world_coords(:,1),T_world_coords(:,2),'k');


%% Plotting velocities
velocity_T_world = zeros(size(T_world,1)-1,1);
control_pose_delta_time = 0.25;
control_pose_times = linspace(control_pose_delta_time,(size(T,1)-1)*control_pose_delta_time, ... 
    size(T,1)-1); % mimic 4Hz data
for i = 1:size(T_world)-1
    velocity_T_world(i) = (T_world{i+1}(1,4) - T_world{i}(1,4))/control_pose_delta_time;
end

velocity_T_spline = zeros(size(T_w_s,1)*(size(T_w_s,2)-1),1);
spline_times = linspace(control_pose_delta_time,(size(T,1)-2)*control_pose_delta_time, ... 
    size(T_w_s,1)*(num_spline_points-1)); % mimic 4Hz data
spline_delta_time = control_pose_delta_time/num_spline_points;
for i = 1:size(T_w_s,1)
    for j = 1:(size(T_w_s,2)-1)
        velocity_T_spline(j+(i-1)*(num_spline_points-1)) = (T_w_s{i,j+1}(1,4) - T_w_s{i,j}(1,4))/spline_delta_time;
%           velocity_T_spline(j+(i-1)*(num_spline_points-1)) = (T_w_s{1,j+1}(1,4) - T_w_s{1,j}(1,4))/spline_delta_time;
    end
end

%%
% Doing proper velocity estimation using spline fusion

num_intervals = size(T,1)-3;
num_spline_points = 20;
C = 1/6 * [ 6 0 0 0; 5 3 -3 1; 1 3 3 -2; 0 0 0 1 ];
T_dot_w_s = cell(num_intervals,num_spline_points);

% s_i = linspace(0,size(T,1)-1,size(T,1));

for i = 1:num_intervals
    s_t = linspace(i,i+1,num_spline_points);
    for t = 1:num_spline_points
        u = s_t(t) - s_i(i+1);
        u_vector = [1; u; u^2; u^3];
        B = C * u_vector;
        B_dot = (1/control_pose_delta_time) * C * [0; 1; 2*u; 3*u^2];

        A0 = expm(Omega{i} * B(2));
        A1 = expm(Omega{i+1} * B(3));
        A2 = expm(Omega{i+2} * B(4));

        A0_dot = A0 * Omega{i} * B_dot(2);
        A1_dot = A1 * Omega{i+1} * B_dot(3);
        A2_dot = A2 * Omega{i+2} * B_dot(4);
        
        T_dot_w_s{i,t} = T_world{i} * (A0_dot*A1*A2 + A0*A1_dot*A2 + A0*A1*A2_dot);
    end
end

% figure(3);
% clf;
% hold on;
% grid on;
figure(2);
clf;
hold on;
grid on;
% axis equal;
title('Velocities in x');
plot(control_pose_times,velocity_T_world,'r*--')
plot(spline_times,velocity_T_spline,'b*')
% legend('Control pose estimates','Spline estimates','Location','southwest');

spline_velocity_times = linspace(control_pose_delta_time,(size(T,1)-2)*control_pose_delta_time, ... 
    size(T_w_s,1)*(num_spline_points-0)); % mimic 4Hz data
for i = 1:num_intervals
    for j = 1:size(T_dot_w_s,2)
%         plot((i-1)*num_spline_points+ j,T_dot_w_s{i,j}(1,4),'k.')
        plot(spline_velocity_times((i-1)*num_spline_points + j),T_dot_w_s{i,j}(1,4),'k.')
    end
end
