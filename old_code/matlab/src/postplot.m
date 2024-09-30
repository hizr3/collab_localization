



nBins = 40;
ea = 0.3;
fa = 0.8;

p_max = max([EKF_rmse_with_gps(1,:), EKF_rmse_no_gps(1,:) , DCL_rmse(1,:)]);
p_min = min([EKF_rmse_with_gps(1,:), EKF_rmse_no_gps(1,:) , DCL_rmse(1,:)]);
p_step = min([  range(EKF_rmse_with_gps(1,:)), ...
                range(EKF_rmse_no_gps(1,:)),...
                range(DCL_rmse(1,:))] / nBins);
p_edges = p_min:p_step:p_max;
      
t_max = max([EKF_rmse_with_gps(2,:), EKF_rmse_no_gps(2,:), DCL_rmse(2,:)]);
t_min = min([EKF_rmse_with_gps(2,:), EKF_rmse_no_gps(2,:), DCL_rmse(2,:)]);
t_step = min([  range(EKF_rmse_with_gps(2,:)), ...
                range(EKF_rmse_no_gps(2,:)),...
                range(DCL_rmse(2,:))] / nBins);
t_edges = t_min:t_step:t_max;

figure(1); clf; hold on; 
histogram(EKF_rmse_with_gps(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'EKF - GNSS Cluster', 'FaceColor', [128, 128, 0] / 255);
histogram(EKF_rmse_no_gps(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'EKF - No GNSS Cluster', 'FaceColor', [75, 0, 130] / 255);
histogram(DCL_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'DCL', 'FaceColor', 'blue');

legend; xlabel('Position Error');


hold off;
 
clear nBins ea fa;
clear p_min p_max;
clear t_min t_max;
clear p_edges t_edges;


