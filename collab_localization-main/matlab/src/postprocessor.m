
EKF_diff = EKF_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
EKF_err  = [sqrt(EKF_diff(1,:,:,:).^2 + EKF_diff(2,:,:,:).^2); EKF_diff(3,:,:,:,:)];
EKF_rmse = reshape(RMSE(EKF_err), [2, nSims]); 

DCL_diff = DCL_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
DCL_err  = [sqrt(DCL_diff(1,:,:,:).^2 + DCL_diff(2,:,:,:).^2); DCL_diff(3,:,:,:,:)];
DCL_rmse = reshape(RMSE(DCL_err), [2, nSims]); 

GSF_MAP_diff = GSF_MAP_x(1:3,:,:,:) - repmat(reshape(x_truth(1:3,:,:), [3, nCars, 1, nTicks]), [1, 1, nSims, 1]);
GSF_MAP_err  = [sqrt(GSF_MAP_diff(1,:,:,:).^2 + GSF_MAP_diff(2,:,:,:).^2); GSF_MAP_diff(3,:,:,:,:)];
GSF_MAP_rmse = reshape(RMSE(GSF_MAP_err), [2, nSims]); 


clear EKF_diff EKF_err;
clear DCL_diff DCL_err;
clear GSF_MAP_diff GSF_MAP_err;


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
