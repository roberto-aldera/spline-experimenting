
% Spline fusion: Lovegrove 2013
% Attempting second and third order splines
% Roberto Aldera - 19 November 2019
clc;clear
%% Setting up transforms
theta = -pi/16;
R = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
a = [1; 0.1; 0]; % position of origin frame a in reference frame b
T0 = [ R a; 0 0 0 1];

T = {eye(4);eye(4);T0; T0; T0; T0; T0; T0; T0; eye(4);eye(4)}; % pretending the transforms are all the same for now

% Offset the points to make it more interesting
T{3}(1,4) = T{3}(1,4) - 0.3;
T{3}(2,4) = T{3}(2,4) - 0.5;
T{4}(1,4) = T{3}(1,4) + 0.3;
T{4}(2,4) = T{3}(2,4) - 0.2;
T{6}(1,4) = T{6}(1,4) + 0.2;
T{6}(2,4) = T{6}(2,4) - 1;
% T{8}(1,4) = T{8}(1,4) + 0.1;
% T{8}(2,4) = T{8}(2,4) - 3;

T_world = cell(size(T,1),1);
T_world{1} = eye(4);
for i = 2:size(T,1)
    T_world{i,1} = T_world{i-1} * T{i};
end

Omega = cell(size(T,1),1);
for i = 1:size(T,1)-1 
    Omega{i} = logm(T_world{i}^-1 * T_world{i+1});
end

%% Spline fusion parameters
num_intervals = size(T,1)-3;
num_spline_points = 20;
C = 1/6 * [ 6 0 0 0; 5 3 -3 1; 1 3 3 -2; 0 0 0 1 ];
T_w_s = cell(num_intervals,num_spline_points);

s_i = linspace(0,size(T,1)-1,size(T,1));

for i = 1:num_intervals
    s_t = linspace(i,i+1,num_spline_points);
    for t = 1:num_spline_points
        u = s_t(t) - s_i(i+1);
        u_vector = [1; u; u^2; u^3];
        B = C * u_vector;

        T_w_s{i,t} = T_world{i} * expm(B(2) * Omega{i}) ...
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
num_pred_points = 10;
for i = 1:num_intervals
    t_s_x = [T_w_s{i,num_spline_points-2}(1,4),T_w_s{i,num_spline_points-1}(1,4),T_w_s{i,num_spline_points}(1,4)];
    t_s_y = [T_w_s{i,num_spline_points-2}(2,4),T_w_s{i,num_spline_points-1}(2,4),T_w_s{i,num_spline_points}(2,4)];
    order_1_pred = polyfit([T_w_s{i,num_spline_points-1}(1,4),T_w_s{i,num_spline_points}(1,4)], ... 
        [T_w_s{i,num_spline_points-1}(2,4),T_w_s{i,num_spline_points}(2,4)],1);
    order_2_pred = polyfit(t_s_x,t_s_y,2);
    t = linspace(T_world{i+2}(1,4),T_world{i+3}(1,4),num_pred_points);
    plot(t,polyval(order_1_pred,t),'--o','Color',[1 0 1],'Linewidth',2,'MarkerIndices',[1,num_pred_points]);
    plot(t,polyval(order_2_pred,t),'--o','Color',[0 1 1],'Linewidth',2,'MarkerIndices',[1,num_pred_points]);
    

end

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

%% Playground
% clf;
% figure(2);
% p = [1 1 1];
% t = linspace(0,5,10);
% plot(t, polyval(p,t));
% hold on;
% 
% p_x = [0 1 2];
% p_y = [0 1 4];
% q = polyfit(p_x,p_y,2);
% plot(t,polyval(q,t));


