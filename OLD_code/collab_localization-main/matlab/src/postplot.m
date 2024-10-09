% close all;
% 
% 
% EKF_diff = EKF_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
% EKF_err  = [sqrt(EKF_diff(1,:,:,:).^2 + EKF_diff(2,:,:,:).^2); EKF_diff(3,:,:,:,:)];
% EKF_rmse = reshape(RMSE(EKF_err), [2, nSims]); 
% 
% uDCL_diff = uDCL_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
% uDCL_err  = [sqrt(uDCL_diff(1,:,:,:).^2 + uDCL_diff(2,:,:,:).^2); uDCL_diff(3,:,:,:,:)];
% uDCL_rmse = reshape(RMSE(uDCL_err), [2, nSims]); 
% 
% GSF_MAP_diff = GSF_MAP_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
% GSF_MAP_err  = [sqrt(GSF_MAP_diff(1,:,:,:).^2 + GSF_MAP_diff(2,:,:,:).^2); GSF_MAP_diff(3,:,:,:,:)];
% GSF_MAP_rmse = reshape(RMSE(GSF_MAP_err), [2, nSims]); 
% 
% 
% clear EKF_diff EKF_err;
% clear uDCL_diff uDCL_err;
% clear GSF_MAP_diff GSF_MAP_err;
% 
% 
% 
% % Hist plot
% nBins = 40;
% ea = 0.3;
% fa = 0.8;
% 
% p_max = max([GSF_MAP_rmse(1,:),EKF_rmse(1,:) uDCL_rmse(1,:)]);
% p_min = min([GSF_MAP_rmse(1,:),EKF_rmse(1,:) uDCL_rmse(1,:)]);
% p_step = min([  range(GSF_MAP_rmse(1,:)), ...
%                 range(uDCL_rmse(1,:)), ...
%                 range(EKF_rmse(1,:))] / nBins);
% p_edges = p_min:p_step:p_max;
% 
% t_max = max([GSF_MAP_rmse(2,:) , EKF_rmse(2,:), uDCL_rmse(2,:)]);
% t_min = min([GSF_MAP_rmse(2,:) , EKF_rmse(2,:), uDCL_rmse(2,:)]);
% t_step = min([  range(GSF_MAP_rmse(2,:)),...
%                 range(EKF_rmse(2,:)), ...
%                 range(uDCL_rmse(2,:))] / nBins);
% t_edges = t_min:t_step:t_max;
% 
% figure(1)
% histogram(EKF_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'EKF');
% hold on
% histogram(uDCL_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'uDCL');
% histogram(GSF_MAP_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'GSF-MAP', 'FaceColor', [5, 125, 128] / 255);
% grid on
% hold off;
% legend; xlabel('Position Error'); ylabel('Relative Frequency')
% clear nBins ea fa;
% clear p_min p_max;
% clear t_min t_max;
% clear p_edges t_edges;

% Choose any MC run. I chose 42 
trstar = 42;

% Vehicle 1 trajectory
figure(2)
title("Vehicle 1 trajectory")
plot(squeeze(x_truth(1, 1, :)), squeeze(x_truth(2, 1, :)), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
hold on
plot(squeeze(EKF_x(1, 1, trstar, :)), squeeze(EKF_x(2, 1, trstar, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2, 'LineStyle', ':');
plot(squeeze(uDCL_x(1, 1, trstar, :)), squeeze(uDCL_x(2, 1, trstar, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2, 'LineStyle', '--');
plot(squeeze(GSF_MAP_x(1, 1, trstar, :)), squeeze(GSF_MAP_x(2, 1, trstar, :)), 'Color', [0, 128, 128] / 255, 'LineWidth', 2, 'LineStyle', '-.');
grid on
legend("Truth" , "EKF" , "uDCL","GSF-MAP")
grid on;



% Vehicle 6 trajectory
figure(3)
title("Vehicle 6 trajectory")
plot(squeeze(x_truth(1, 6, :)), squeeze(x_truth(2, 6, :)), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
hold on
plot(squeeze(EKF_x(1, 6, trstar, :)), squeeze(EKF_x(2, 6, trstar, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2, 'LineStyle', ':');
plot(squeeze(uDCL_x(1, 6, trstar, :)), squeeze(uDCL_x(2, 6, trstar, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2, 'LineStyle', '--');
plot(squeeze(GSF_MAP_x(1, 6,trstar, :)), squeeze(GSF_MAP_x(2, 6, trstar, :)), 'Color', [0, 128, 128] / 255, 'LineWidth', 2, 'LineStyle', '-.');
grid on
legend("Truth" , "EKF" , "uDCL","GSF-MAP")
grid on;



% Steering angle plot
figure(5)
plot(1:nTicks, del_ref(:,1), 'Color', [128, 0, 0] / 255, 'LineWidth', 2);
hold on
plot(1:nTicks, squeeze(EKF_x(7, 1, trstar, :)), 'Color', [128, 128, 0] / 255, 'LineWidth', 2, 'LineStyle', ':');
plot(1:nTicks, squeeze(uDCL_x(7, 1, trstar, :)), 'Color', [75, 0, 130] / 255, 'LineWidth', 2, 'LineStyle', '--');
plot(1:nTicks, squeeze(GSF_MAP_x(7, 1, trstar, :)), 'Color', [0, 128, 128] / 255, 'LineWidth', 2, 'LineStyle', '-.');
plot(1:nTicks, del(:,1), 'Color', [75, 0, 130] / 255, 'LineWidth', 2);
legend("Ref. steering angle" , "EKF steering angle" ,"uDCL steering angle", "GSF-MAP steering angle" , "Actual steering input")



function err = RMSE(x)
    m = size(x,1);
    n = size(x,3);
    
    N = size(x,2) * size(x,4);
    err = zeros(m, n);
    
    for i = 1:m
        for j = 1:n
            err(i,j) = sqrt( sum(x(i,:,j,:).^2,[2,4]) / N);
        end
    end
end
