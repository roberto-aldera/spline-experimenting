% Comparing different point clouds of landmarks
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
frame = 11;
% for i = frame:frame %5:17
for i = 5:17
    index_ro = i;
    ro_landmarks = JavaPbSerialisedPointCloudToMatlab(ro_landmarks_mono(index_ro));
    
    index_interpolation_corrected_ro = index_ro - 2; %interpolation is running behind RO
    ro_landmarks_interpolation_corrected = JavaPbSerialisedPointCloudToMatlab(ro_landmarks_interpolation_corrected_mono(index_interpolation_corrected_ro));
    
    index_spline_corrected_ro = index_ro - 3; %spline is running behind RO
    ro_landmarks_spline_corrected = JavaPbSerialisedPointCloudToMatlab(ro_landmarks_spline_corrected_mono(index_spline_corrected_ro));

    f1 = figure(1);
    clf;
    hold on;
    axis equal;
    scope = 20;
    xlim([-scope scope]);
    ylim([-scope scope]);

    scatter(0,0,'o','LineWidth',2,...
    'MarkerEdgeColor','black',...
    'MarkerFaceColor',[0 1 0]);
    
    scatter(ro_landmarks.xyz(:,1),ro_landmarks.xyz(:,2),'ro','LineWidth',1)
    scatter(ro_landmarks_interpolation_corrected.xyz(:,1),ro_landmarks_interpolation_corrected.xyz(:,2),'k+','LineWidth',1)
    scatter(ro_landmarks_spline_corrected.xyz(:,1),ro_landmarks_spline_corrected.xyz(:,2),'^','LineWidth',1,'MarkerEdgeColor',[.3 .6 1])
    legend('Radar','Uncorrected landmarks','Interpolation-corrected landmarks','Spline-corrected landmarks');
    saveas(f1,['/Users/roberto/data/spline-experiments/matlab-figs/idx_',num2str(i),'.png']);
    
    ro_vs_spline = sqrt((ro_landmarks.xyz(:,1)-ro_landmarks_spline_corrected.xyz(:,1)).^2 + ...
         (ro_landmarks.xyz(:,2)-ro_landmarks_spline_corrected.xyz(:,2)).^2);
    ro_vs_interpolation = sqrt((ro_landmarks.xyz(:,1)-ro_landmarks_interpolation_corrected.xyz(:,1)).^2 + ...
         (ro_landmarks.xyz(:,2)-ro_landmarks_interpolation_corrected.xyz(:,2)).^2);
    interpolation_vs_spline = sqrt((ro_landmarks_interpolation_corrected.xyz(:,1)-ro_landmarks_spline_corrected.xyz(:,1)).^2 + ...
         (ro_landmarks_interpolation_corrected.xyz(:,2)-ro_landmarks_spline_corrected.xyz(:,2)).^2);
     
    figure(2);
%     clf;
    hold on;
    plot(ro_vs_spline,'b');
    plot(ro_vs_interpolation,'k');
%     plot(interpolation_vs_spline);
    xlabel('Landmark index');
    ylabel('Distance moved');
    title('Magnitude of landmark corrections');
    legend('Spline corrections','Interpolation corrections');
    
    ro_spline_mag = sqrt(sum(ro_vs_spline.^2));
    ro_interpolation_mag = sqrt(sum(ro_vs_interpolation.^2));
    interpolation_spline_mag = sqrt(sum(interpolation_vs_spline.^2));

end
