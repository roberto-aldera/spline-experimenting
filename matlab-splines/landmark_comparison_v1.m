% Reading point clouds from file
clc;clear;clf;

default_landmarks_mono_path = '/private/tmp/ro_landmarks.monolithic';
ro_landmarks_mono = Monolithic(default_landmarks_mono_path);

interpolation_corrected_landmarks_mono_path = '/private/tmp/ro_landmarks_interpolation_corrected.monolithic';
ro_landmarks_interpolation_corrected_mono = Monolithic(interpolation_corrected_landmarks_mono_path);

spline_corrected_landmarks_mono_path = '/private/tmp/ro_landmarks_spline_corrected.monolithic';
ro_landmarks_spline_corrected_mono = Monolithic(spline_corrected_landmarks_mono_path);

range = 50;
bin_size = 0.25;
num_bins = range/bin_size;
centres = zeros(num_bins,2);
radii = linspace(bin_size,range,num_bins);    
frame = 10;
for i = frame:frame %5:17
% for i = 5:17
    clf;
    index_ro = i;
    ro_landmarks = JavaPbSerialisedPointCloudToMatlab(ro_landmarks_mono(index_ro));
    
    index_interpolation_corrected_ro = index_ro - 2; %spline is running behind RO
    ro_landmarks_interpolation_corrected = JavaPbSerialisedPointCloudToMatlab(ro_landmarks_interpolation_corrected_mono(index_interpolation_corrected_ro));
    
    index_spline_corrected_ro = index_ro-2; %spline is running behind RO
    ro_landmarks_spline_corrected = JavaPbSerialisedPointCloudToMatlab(ro_landmarks_spline_corrected_mono(index_spline_corrected_ro));

    f1 = figure(1);
    hold on;
    axis equal;
    scope = 20;
    xlim([-scope scope]);
    ylim([-scope scope]);
    
    % Make grid
%     viscircles(centres,radii,'Color',[0.9 0.9 0.9]);
%     for k = 1:400
%         plot([0 400 * cos(k*2*pi/400);],[0 400 * sin(k*2*pi/400)],'Color',[0.9 0.9 0.9]);
%     end

    scatter(0,0,'o','LineWidth',2,...
    'MarkerEdgeColor','black',...
    'MarkerFaceColor',[0 1 0]);
    
     scatter(ro_landmarks.xyz(:,1),ro_landmarks.xyz(:,2),'ro','LineWidth',1)
     scatter(ro_landmarks_interpolation_corrected.xyz(:,1),ro_landmarks_interpolation_corrected.xyz(:,2),'k+','LineWidth',1)
     scatter(ro_landmarks_spline_corrected.xyz(:,1),ro_landmarks_spline_corrected.xyz(:,2),'^','LineWidth',1,'MarkerEdgeColor',[.3 .6 1])

%     start_idx = ro_landmarks.num_valid_points-10;
%     for n = start_idx:ro_landmarks.num_valid_points
%         colour = [.3 (n-start_idx)/(ro_landmarks.num_valid_points-start_idx) 1];
%         plot(ro_landmarks.xyz(n,1),ro_landmarks.xyz(n,2),'o','LineWidth',1,'MarkerEdgeColor',colour)
%         plot(ro_landmarks_interpolation_corrected.xyz(n,1),ro_landmarks_interpolation_corrected.xyz(n,2),'+','LineWidth',1,'MarkerEdgeColor',colour)
%         plot(ro_landmarks_spline_corrected.xyz(n,1),ro_landmarks_spline_corrected.xyz(n,2),'^','LineWidth',1,'MarkerEdgeColor',colour)
% 
% %         scatter(ro_landmarks_interpolation_corrected.xyz(:,1),ro_landmarks_interpolation_corrected.xyz(:,2),'k+','LineWidth',1)
% %         scatter(ro_landmarks_spline_corrected.xyz(:,1),ro_landmarks_spline_corrected.xyz(:,2),'o','LineWidth',1,'MarkerEdgeColor',[.3 .6 1])
%     end

%     scatter(ro_landmarks.xyz(:,1),ro_landmarks.xyz(:,2),'ro','LineWidth',1)
%     scatter(ro_landmarks_interpolation_corrected.xyz(:,1),ro_landmarks_interpolation_corrected.xyz(:,2),'k+','LineWidth',1)
%     scatter(ro_landmarks_spline_corrected.xyz(:,1),ro_landmarks_spline_corrected.xyz(:,2),'o','LineWidth',1,'MarkerEdgeColor',[.3 .6 1])
    
    legend('Radar','Uncorrected landmarks','Interpolation-corrected landmarks','Spline-corrected landmarks');
    saveas(f1,['/Users/roberto/data/spline-experiments/matlab-figs/idx_',num2str(i),'.png']);
end
