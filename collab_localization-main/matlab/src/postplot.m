close all;

% Hist plot
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

figure(1)
histogram(EKF_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'EKF');
hold on
histogram(DCL_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'DCL');
histogram(GSF_MAP_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'GSF-MAP', 'FaceColor', [5, 125, 128] / 255);
grid on
hold off;
legend; xlabel('Position Error'); ylabel('Relative Frequency')
clear nBins ea fa;
clear p_min p_max;
clear t_min t_max;
clear p_edges t_edges;

% Choose any MC run. I chose 42 
trstar = 42;

% Vehicle 1 trajectory
figure(2)
title("Vehicle 1 trajectory")
plot(squeeze(x_truth(1, 1, :)), squeeze(x_truth(2, 1, :)), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
hold on
plot(squeeze(EKF_x(1, 1, trstar, :)), squeeze(EKF_x(2, 1, trstar, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2, 'LineStyle', ':');
plot(squeeze(DCL_x(1, 1, trstar, :)), squeeze(DCL_x(2, 1, trstar, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2, 'LineStyle', '--');
plot(squeeze(GSF_MAP_x(1, 1, trstar, :)), squeeze(GSF_MAP_x(2, 1, trstar, :)), 'Color', [0, 128, 128] / 255, 'LineWidth', 2, 'LineStyle', '-.');
grid on
legend("Truth" , "EKF" , "DCL","GSF")
grid on;



% Vehicle 6 trajectory
figure(3)
title("Vehicle 6 trajectory")
plot(squeeze(x_truth(1, 6, :)), squeeze(x_truth(2, 6, :)), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
hold on
plot(squeeze(EKF_x(1, 6, trstar, :)), squeeze(EKF_x(2, 6, trstar, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2, 'LineStyle', ':');
plot(squeeze(DCL_x(1, 6, trstar, :)), squeeze(DCL_x(2, 6, trstar, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2, 'LineStyle', '--');
plot(squeeze(GSF_MAP_x(1, 6,trstar, :)), squeeze(GSF_MAP_x(2, 6, trstar, :)), 'Color', [0, 128, 128] / 255, 'LineWidth', 2, 'LineStyle', '-.');
grid on
legend("Truth" , "EKF" , "DCL","GSF")
grid on;


% % Orientation plot
% figure(4)
% plot(1:nTicks, squeeze(x_truth(3, 1, :)), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
% hold on
% plot(1:nTicks, squeeze(EKF_x(3, 1,trstar, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2);
% plot(1:nTicks, squeeze(DCL_x(3, 1 , trstar, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2);
% plot(1:nTicks, squeeze(GSF_MAP_x(3, 1 , trstar, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2);
% legend("Theta #1 ", "EKF theta" , "DCL theta" , "GSF theta")
% 
% Steering angle plot
figure(5)
plot(1:nTicks, del_ref(:,1), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
hold on
plot(1:nTicks, squeeze(GSF_MAP_x(7, 1, trstar, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2);
plot(1:nTicks, del(:,1), 'Color', [75, 0, 130] / 255, 'LineWidth', 2);
legend("Ref. steering angle" , "GSF steering angle" , "Actual steering input")