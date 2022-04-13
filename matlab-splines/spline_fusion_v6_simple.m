
% Spline fusion: Lovegrove 2013
% Back to simple splines (using this for debugging while trying to write up
% python implementation
% Roberto Aldera - 24 Sept 2020
clc;clear
%% Setting up transforms
theta = -pi/32;
R = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
a = [1; 0.0; 0]; % position of origin frame a in reference frame b
T0 = [ R a; 0 0 0 1];

T = {T0; T0; T0; T0; T0; T0; T0; T0}; % pretending the transforms are all the same for now

% Offset the points to make it more interesting
T{3}(1,4) = T{3}(1,4) + 0.3;
T{3}(2,4) = T{3}(2,4) - 0.3;
T{6}(1,4) = T{6}(1,4) + 0.2;
T{6}(2,4) = T{6}(2,4) - 0.4;
T{8}(1,4) = T{8}(1,4) + 0.1;
T{8}(2,4) = T{8}(2,4) - 0.1;

T_world = cell(size(T,1),1);
T_world{1} = eye(4);
for i = 2:size(T,1)
    T_world{i,1} = T_world{i-1} * T{i};
end

Omega = cell(size(T,1),1);
for i = 1:size(T,1)-1 
    Omega{i} = logm(T_world{i}^-1 * T_world{i+1});
end
disp(Omega{1})
%% Spline fusion parameters
num_intervals = size(T,1)-3;
num_spline_points = 10;
C = 1/6 * [ 6 0 0 0; 5 3 -3 1; 1 3 3 -2; 0 0 0 1 ];
T_w_s = cell(num_spline_points,num_intervals);

s_i = linspace(0,size(T,1)-1,size(T,1));

for i = 1:num_intervals
    s_t = linspace(i,i+1,num_spline_points);
    for t = 1:num_spline_points
        u = s_t(t) - s_i(i+1);
        u_vector = [1; u; u^2; u^3];
        B = C * u_vector;
        
        p1 = expm(B(2) * Omega{i});
        p2 = expm(B(3) * Omega{i+1});
        p3 = expm(B(4) * Omega{i+2});
        T_w_s{t,i} = T_world{i} * p1 * p2 * p3;
%         This line below works, I just wanted to break it into parts (above)
%         for debugging purposes
%         T_w_s{t,i} = T_world{1} * expm(B(2) * Omega{i}) ...
%             * expm(B(3) * Omega{i+1}) * expm(B(4) * Omega{i+2});
        disp(T_w_s{t,i}(1,4))
    end
end

%% Plotting
figure(1);
clf;
hold on;
grid on;
axis equal;
title("Spline fusion using cumulative cubic B-Splines");

for i = 1:size(T,1)
    scatter(T_world{i}(1,4),T_world{i}(2,4),1000,'r.');
end

for i = 1:size(T_w_s,1)
    for j = 1:num_intervals
        scatter(T_w_s{i,j}(1,4),T_w_s{i,j}(2,4),100,'b.');
    end
end

%% Plot connections directly between control points
T_world_coords = zeros(size(T_world,1),2);
for i = 1:size(T_world)
    T_world_coords(i,:) = [T_world{i}(1,4) T_world{i}(2,4)];
end
plot(T_world_coords(:,1),T_world_coords(:,2),'k');