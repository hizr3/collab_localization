



nBins = 40;
ea = 0.3;
fa = 0.8;

p_max = max([GSF_MAP_rmse(1,:),EKF_rmse(1,:) DCL_rmse(1,:)]);
p_min = min([GSF_MAP_rmse(1,:),EKF_rmse(1,:) DCL_rmse(1,:)]);
p_step = min([  range(GSF_MAP_rmse(1,:)), ...
                range(DCL_rmse(1,:)), ...
                range(EKF_rmse(1,:))] / nBins);
p_edges = p_min:p_step:p_max;
      
t_max = max([GSF_MAP_rmse(2,:) , EKF_rmse(2,:), DCL_rmse(2,:)]);
t_min = min([GSF_MAP_rmse(2,:) , EKF_rmse(2,:), DCL_rmse(2,:)]);
t_step = min([  range(GSF_MAP_rmse(2,:)),...
                range(EKF_rmse(2,:)), ...
                range(DCL_rmse(2,:))] / nBins);
t_edges = t_min:t_step:t_max;

figure(1); clf; hold on; 
histogram(EKF_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'EKF', 'FaceColor', [128, 128, 0] / 255);
histogram(DCL_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'DCL', 'FaceColor', [75, 0, 130] / 255);
histogram(CKF_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'CKF', 'FaceColor', 'blue');
% histogram(GSF_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'GSF', 'FaceColor', 'green');
histogram(GSF_MAP_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'GSF-MAP', 'FaceColor', [5, 125, 128] / 255);
 legend; xlabel('Position Error');


hold off;
% 
clear nBins ea fa;
clear p_min p_max;
clear t_min t_max;
clear p_edges t_edges;

% 
%Vehicle 1 trajectory
figure; hold on;
plot(squeeze(x_truth(1, 1, :)), squeeze(x_truth(2, 1, :)), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
% plot(squeeze(GSF_x(1, 1, 1, :)), squeeze(GSF_x(2, 1, 1, :)), 'Color', 'green', 'LineWidth', 2, 'LineStyle', '-.');
plot(squeeze(GSF_MAP_x(1, 1, 1, :)), squeeze(GSF_MAP_x(2, 1, 1, :)), 'Color', [0, 128, 128] / 255, 'LineWidth', 2, 'LineStyle', '-.');

plot(squeeze(EKF_x(1, 1, 1, :)), squeeze(EKF_x(2, 1, 1, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2, 'LineStyle', ':');
plot(squeeze(DCL_x(1, 1, 1, :)), squeeze(DCL_x(2, 1, 1, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2, 'LineStyle', '--');

%%% Dont touch the CKF !!
plot(squeeze(CKF_x(1,  1, :)), squeeze(CKF_x(2, 1, :)), 'Color', 'blue', 'LineWidth', 2, 'LineStyle', ':');
legend("Truth" , "GSF-MAP", "EKF" , "DCL" , "CKF")
title("Vehicle #1")
grid on


% %%%%%%%%%%%%%%%%%

%Vehicle 6 trajectory
figure; hold on;
plot(squeeze(x_truth(1, 6, :)), squeeze(x_truth(2, 6, :)), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
% plot(squeeze(GSF_x(1, 6, 1, :)), squeeze(GSF_x(2, 6, 1, :)), 'Color', 'green', 'LineWidth', 2, 'LineStyle', '-.');
plot(squeeze(GSF_MAP_x(1, 6, 1, :)), squeeze(GSF_MAP_x(2, 6, 1, :)), 'Color', [0, 128, 128] / 255, 'LineWidth', 2, 'LineStyle', '-.');

plot(squeeze(EKF_x(1, 6, 1, :)), squeeze(EKF_x(2, 6, 1, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2, 'LineStyle', ':');
plot(squeeze(DCL_x(1, 6, 1, :)), squeeze(DCL_x(2, 6, 1, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2, 'LineStyle', '--');
plot(squeeze(CKF_x(1, 6,  :)), squeeze(CKF_x(2, 6, :)), 'Color', 'blue', 'LineWidth', 2, 'LineStyle', ':');
legend("Truth" ,  "GSF-MAP", "EKF" , "DCL" , "CKF")
title("Vehicle #6")
grid on


