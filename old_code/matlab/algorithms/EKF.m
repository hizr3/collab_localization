if t == 1
    

  % We modify the EKF to handle cars with GPS and without at the same time
    gps_idxs = [1:nCars_with_gps]
    no_gps_idx = [(nCars_with_gps+1):(nCars_no_gps+nCars_with_gps)]

    EKF_x_with_gps = zeros(6, nCars_with_gps, nSims, nTicks);
    EKF_x_with_gps(1:3,:,:,t) = repmat(x_truth(1:3,gps_idxs,t), [1,1,nSims]);
    EKF_x_with_gps(4,:,:,t)   = repmat(-sin(x_truth(3,gps_idxs,t)) .* x_truth(4,gps_idxs,t), [1,1,nSims]);
    EKF_x_with_gps(5,:,:,t)   = repmat( cos(x_truth(3,gps_idxs,t)) .* x_truth(4,gps_idxs,t), [1,1,nSims]);
    
    EKF_P_with_gps = repmat(... 
            diag([  imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_gyr_err]), [1,1,nCars_with_gps,nSims]);

    % no gps cluster
    EKF_x_no_gps = zeros(6, nCars_no_gps, nSims, nTicks);
    EKF_x_no_gps(1:3,:,:,t) = repmat(x_truth(1:3,no_gps_idx,t), [1,1,nSims]);
    EKF_x_no_gps(4,:,:,t)   = repmat(-sin(x_truth(3,no_gps_idx,t)) .* x_truth(4,no_gps_idx,t), [1,1,nSims]);
    EKF_x_no_gps(5,:,:,t)   = repmat( cos(x_truth(3,no_gps_idx,t)) .* x_truth(4,no_gps_idx,t), [1,1,nSims]);
    
    EKF_P_no_gps = repmat(... 
            diag([  imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_gyr_err]), [1,1,nCars_no_gps,nSims]);
                

else
    
    % Predict Step at 400 Hz for GPS capable cars
    if mod(t, rate/rate_imu) == 0
        accel = [acc(t,gps_idxs); x_truth(4,gps_idxs,t).^2 / wb.* tan(del(t,gps_idxs))] ...
            + normrnd(0, imu_acc_err, [2, nCars_with_gps, nSims]);
        
        gyro = x_truth(4,gps_idxs,t) .* tan(del(t,gps_idxs)) / wb ...
            + normrnd(0, imu_gyr_err, [1, nCars_with_gps, nSims]);
        
        mag = x_truth(3,gps_idxs,t) ...
            + normrnd(0, imu_mag_err, [1, nCars_with_gps, nSims]);
        
        theta = EKF_x_with_gps(3,:,:,t);

        accel_r = [...
            -sin(theta) .* accel(1,:,:) - cos(theta).*accel(2,:,:) ;...
             cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ];

        EKF_x_with_gps(:,:,:,t) = [...
            EKF_x_with_gps(1,:,:,t-1) + EKF_x_with_gps(4,:,:,t-1) * dt + accel_r(1,:,:) / 2 * dt^2; ...
            EKF_x_with_gps(2,:,:,t-1) + EKF_x_with_gps(5,:,:,t-1) * dt + accel_r(2,:,:) / 2 * dt^2; ...
           (EKF_x_with_gps(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;...
            EKF_x_with_gps(4,:,:,t-1) + accel_r(1,:,:) * dt;...
            EKF_x_with_gps(5,:,:,t-1) + accel_r(2,:,:) * dt;...
            gyro];

        
        Q = diag([  imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_gyr_err]);

        F = [   1,  0,  0, dt,  0,  0  ;...
                0,  1,  0,  0, dt,  0  ;...
                0,  0,  1,  0,  0, dt  ;...
                0,  0,  0,  1,  0,  0  ;...
                0,  0,  0,  0,  1,  0  ;...
                0,  0,  0,  0,  0,  1 ];
            
        EKF_P_with_gps = pagemtimes(...
                    pagemtimes(...
                        F, ...
                        EKF_P_with_gps),'none',...
                    F, 'transpose') + Q;
        
        clear accel accel_r gyro mag theta Q F
    end

     
    % Predict Step at 400 Hz for GPS-less cars
    if mod(t, rate/rate_imu) == 0
        
        accel = [acc(t,no_gps_idx); x_truth(4,no_gps_idx,t).^2 / wb.* tan(del(t,no_gps_idx))] ...
            + normrnd(0, imu_acc_err, [2, nCars_no_gps, nSims]);
        
        gyro = x_truth(4,no_gps_idx,t) .* tan(del(t,no_gps_idx)) / wb ...
            + normrnd(0, imu_gyr_err, [1, nCars_no_gps, nSims]);
        
        mag = x_truth(3,no_gps_idx,t) ...
            + normrnd(0, imu_mag_err, [1, nCars_no_gps, nSims]);
        
        theta = EKF_x_no_gps(3,:,:,t);

        accel_r = [...
            -sin(theta) .* accel(1,:,:) - cos(theta).*accel(2,:,:) ;...
             cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ];

        EKF_x_no_gps(:,:,:,t) = [...
            EKF_x_no_gps(1,:,:,t-1) + EKF_x_no_gps(4,:,:,t-1) * dt + accel_r(1,:,:) / 2 * dt^2; ...
            EKF_x_no_gps(2,:,:,t-1) + EKF_x_no_gps(5,:,:,t-1) * dt + accel_r(2,:,:) / 2 * dt^2; ...
           (EKF_x_no_gps(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;...
            EKF_x_no_gps(4,:,:,t-1) + accel_r(1,:,:) * dt;...
            EKF_x_no_gps(5,:,:,t-1) + accel_r(2,:,:) * dt;...
            gyro];

        
        Q = diag([  imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_acc_err / rate_imu          ;...
                    imu_gyr_err]);

        F = [   1,  0,  0, dt,  0,  0  ;...
                0,  1,  0,  0, dt,  0  ;...
                0,  0,  1,  0,  0, dt  ;...
                0,  0,  0,  1,  0,  0  ;...
                0,  0,  0,  0,  1,  0  ;...
                0,  0,  0,  0,  0,  1 ];
            
        EKF_P_no_gps = pagemtimes(...
                    pagemtimes(...
                        F, ...
                        EKF_P_no_gps),'none',...
                    F, 'transpose') + Q;
        
        clear accel accel_r gyro mag theta Q F
    end

    % Pacmod Step at 30 Hz for GPS_capable cars
    if mod(t, rate/rate_mdl) == 0 && sensors(1)

        z_vel = x_truth(4,gps_idxs,t) + normrnd(0, enc_err, [1, nCars_with_gps, nSims]);
        z_del = del(t,gps_idxs) + normrnd(0, imu_mag_err, [1, nCars_with_gps, nSims]);
        
        z = [   z_vel;...
                z_vel.*tan(z_del) / wb ];

        kf_vel = sqrt( EKF_x_with_gps(4,:,:,t).^2 + EKF_x_with_gps(5,:,:,t).^2 );
        
        H = zeros(2,6,nCars_with_gps,nSims);
        H(1,4,:,:) = EKF_x_with_gps(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = EKF_x_with_gps(5,:,:,t) ./ kf_vel;
        H(2,6,:,:) = 1;
        
        h = [kf_vel; EKF_x_with_gps(6,:,:,t)];

        R = zeros(2,2,nCars_with_gps,nSims);
        R(1,1,:,:) = enc_err;
        R(2,2,:,:) = str_err * z_vel / wb;

        K = pageDiv( pagemtimes(EKF_P_with_gps,'none',H,'transpose'), (pagemtimes( pagemtimes(H,EKF_P_with_gps), 'none', H, 'transpose') + R));
        EKF_x_with_gps(:,:,:,t) = EKF_x_with_gps(:,:,:,t) + reshape( pagemtimes( K, reshape((z-h), [2, 1, nCars_with_gps, nSims])), [6, nCars_with_gps, nSims]);
        EKF_P_with_gps = pagemtimes( (repmat(eye(6), [1,1,nCars_with_gps,nSims]) - pagemtimes(K,H)), EKF_P_with_gps);

        clear z_vel z_del z kf_vel H h R K
    end


    % Pacmod Step at 30 Hz for gps_less cars
    if mod(t, rate/rate_mdl) == 0 && sensors(1)

        z_vel = x_truth(4,no_gps_idx,t) + normrnd(0, enc_err, [1, nCars_no_gps, nSims]);
        z_del = del(t,no_gps_idx) + normrnd(0, imu_mag_err, [1, nCars_no_gps, nSims]);
        
        z = [   z_vel;...
                z_vel.*tan(z_del) / wb ];

        kf_vel = sqrt( EKF_x_no_gps(4,:,:,t).^2 + EKF_x_no_gps(5,:,:,t).^2 );
        
        H = zeros(2,6,nCars_no_gps,nSims);
        H(1,4,:,:) = EKF_x_no_gps(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = EKF_x_no_gps(5,:,:,t) ./ kf_vel;
        H(2,6,:,:) = 1;
        
        h = [kf_vel; EKF_x_no_gps(6,:,:,t)];

        R = zeros(2,2,nCars_no_gps,nSims);
        R(1,1,:,:) = enc_err;
        R(2,2,:,:) = str_err * z_vel / wb;

        K = pageDiv( pagemtimes(EKF_P_no_gps,'none',H,'transpose'), (pagemtimes( pagemtimes(H,EKF_P_no_gps), 'none', H, 'transpose') + R));
        EKF_x_no_gps(:,:,:,t) = EKF_x_no_gps(:,:,:,t) + reshape( pagemtimes( K, reshape((z-h), [2, 1, nCars_no_gps, nSims])), [6, nCars_no_gps, nSims]);
        EKF_P_no_gps = pagemtimes( (repmat(eye(6), [1,1,nCars_no_gps,nSims]) - pagemtimes(K,H)), EKF_P_no_gps);

        clear z_vel z_del z kf_vel H h R K
    end





    % GPS step at 10 Hz
    if mod(t, rate/rate_gps) == 0 && sensors(2)

        z_x = x_truth(1,gps_idxs,t) + normrnd(0, gps_per, [1, nCars_with_gps, nSims]);
        z_y = x_truth(2,gps_idxs,t) + normrnd(0, gps_per, [1, nCars_with_gps, nSims]);
        z_t = x_truth(3,gps_idxs,t) + normrnd(0, gps_her, [1, nCars_with_gps, nSims]);
        z_v = x_truth(4,gps_idxs,t) + normrnd(0, gps_ver, [1, nCars_with_gps, nSims]);
        
        z = [ z_x; z_y; z_t; z_v ];

        kf_vel = sqrt( EKF_x_with_gps(4,:,:,t).^2 + EKF_x_with_gps(5,:,:,t).^2 );
        
        H = zeros(4,6,nCars_with_gps,nSims);
        H(1,1,:,:) = 1;
        H(2,2,:,:) = 1;
        H(3,3,:,:) = 1;
        H(4,4,:,:) = EKF_x_with_gps(4,:,:,t) ./ kf_vel;
        H(4,5,:,:) = EKF_x_with_gps(5,:,:,t) ./ kf_vel;
        
        h = [   EKF_x_with_gps(1:3,:,:,t);...
                kf_vel];
            
        R = repmat(diag([  ...
            gps_per;...
            gps_per;...
            gps_her;...
            gps_ver]), [1, 1, nCars_with_gps , nSims]);

        K = pageDiv( pagemtimes(EKF_P_with_gps,'none',H,'transpose'), ( pagemtimes( pagemtimes( H, EKF_P_with_gps), 'none', H, 'transpose') + R ) );

        EKF_x_with_gps(:,:,:,t) = EKF_x_with_gps(:,:,:,t) + reshape( pagemtimes( K, reshape(z - h, [4, 1, nCars_with_gps, nSims])), [6, nCars_with_gps, nSims] );
        EKF_P_with_gps = pagemtimes( ( repmat( eye(6), [1,1,nCars_with_gps,nSims] ) - pagemtimes(K,H) ), EKF_P_with_gps );

        clear z_x z_y z_t z_v z kf_vel H h R K
    end
end


if t == length(ticks)

    EKF_x = cat(2,EKF_x_with_gps,EKF_x_no_gps);
    EKF_P = cat(3,EKF_P_with_gps,EKF_P_no_gps);
    

end