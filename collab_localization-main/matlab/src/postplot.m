
nBins = 40;
ea = 0.3;
fa = 0.8;

p_max = max([EKF_rmse(1,:), DCL_rmse(1,:)]);
p_min = min([EKF_rmse(1,:), DCL_rmse(1,:)]);
p_step = min([  range(EKF_rmse(1,:)), ...
                range(EKF_rmse(1,:)), ...
                range(EKF_rmse(1,:))] / nBins);
p_edges = p_min:p_step:p_max;
      
t_max = max([EKF_rmse(2,:),  DCL_rmse(2,:)]);
t_min = min([EKF_rmse(2,:),  DCL_rmse(2,:)]);
t_step = min([  range(EKF_rmse(2,:)), ...
                range(EKF_rmse(2,:)), ...
                range(EKF_rmse(2,:))] / nBins);
t_edges = t_min:t_step:t_max;

figure(1); clf; hold on; 
histogram(EKF_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'EKF');
histogram(DCL_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'DCL');
hold off;


figure(1); legend; xlabel('Position Error'); ylabel('Relative Frequency')
% figure(2); legend; xlabel('Heading Error');

clear nBins ea fa;
clear p_min p_max;
clear t_min t_max;
clear p_edges t_edges;

%Vehicle 1 trajectory

figure(2)
 hold on;
plot(squeeze(x_truth(1, 1, :)), squeeze(x_truth(2, 1, :)), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
plot(squeeze(EKF_x(1, 1, 1, :)), squeeze(EKF_x(2, 1, 1, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2, 'LineStyle', ':');
plot(squeeze(DCL_x(1, 1, 1, :)), squeeze(DCL_x(2, 1, 1, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2, 'LineStyle', '--');
grid on;