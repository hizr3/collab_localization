if t == 1
    
    EKF_x = zeros(6, nCars, nSims, nTicks);
    EKF_x(1:3,:,:,t) = repmat(x_truth(1:3,:,t), [1,1,nSims]);
    EKF_x(4,:,:,t)   = repmat(cos(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    EKF_x(5,:,:,t)   = repmat(sin(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);

    EKF_P = repmat(... 
            diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err/ 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err/ rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err ]), [1,1,nCars,nSims]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = EKF_P(:,:,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    EKF_P(:,:,j1,j2) = temp0;           
                end
             end
   %%%%%%%%%%%%%%%%%%%%%%%%%       
else

    % Predict Step at 400 Hz
    if mod(t, rate/rate_imu) ==  0 


       accel = [acc(t,:); x_truth(4,:,t).^2 / wb.* tan(del(t,:))] ...
            + normrnd(0, imu_acc_err, [2, nCars, nSims]);

        kf_vel = sqrt( EKF_x(4,:,:,t-1).^2 + EKF_x(5,:,:,t-1).^2 );
        theta = EKF_x(3,:,:,t-1); 
        % this was the error. In hartzer's original code, he takes the t value of the array, but it's just zero since it hasnt been loaded yet
        % It doesnt end up mattering since he doesnt have the vehicle do
        % any turns........
            accel_r = [...
            cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ;...
            sin(theta) .* accel(1,:,:) + cos(theta).*accel(2,:,:) ];

        
        gyro = x_truth(4,:,t) .* tan(del(t,:)) / wb ...
            + normrnd(0, imu_gyr_err, [1, nCars, nSims]); 
        
        mag = x_truth(3,:,t) ...  
            + normrnd(0, imu_mag_err, [1, nCars, nSims]);
        



        EKF_x(:,:,:,t) = [...
            EKF_x(1,:,:,t-1) + EKF_x(4,:,:,t-1) * dt + accel_r(1,:,:) / 2 * dt^2; ...
            EKF_x(2,:,:,t-1) + EKF_x(5,:,:,t-1) * dt + accel_r(2,:,:) / 2 * dt^2; ...
           (EKF_x(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;...
            EKF_x(4,:,:,t-1) + accel_r(1,:,:) * dt;...
            EKF_x(5,:,:,t-1) + accel_r(2,:,:) * dt;...
            gyro];

        
        Q = diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err]);


        F = [   1,  0,  0, dt,  0,  0 ;...
                0,  1,  0,  0, dt,  0 ;...
                0,  0,  1,  0,  0, dt ;...
                0,  0,  0,  1,  0,  0 ;...
                0,  0,  0,  0,  1,  0 ;...
                0,  0,  0,  0,  0,  1 ];
            
        EKF_P = pagemtimes(...
                    pagemtimes(...
                        F, ...
                        EKF_P),'none',...
                    F, 'transpose') + Q;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              for j2 = 1 : nSims
%                 for j1 = 1 : nCars
%                     %Resymmetrize
%                     temp0 = EKF_P(:,:,j1,j2);
%                     temp0 = 0.5*(temp0 + temp0');
%                     EKF_P(:,:,j1,j2) = temp0;           
%                 end
%              end
%    %%%%%%%%%%%%%%%%%%%%%%%%%         
        clear accel accel_r gyro mag theta Q F
    end

    % Pacmod Step at 30 Hz
    if mod(t, rate/rate_mdl) == 0 && sensors(1)
        z_vel = x_truth(4,:,t) + normrnd(0, enc_err, [1, nCars, nSims]);

        z_del = x_truth(4,:,t).*tan( del(t,:) ) /wb + normrnd(0, imu_mag_err, [1, nCars, nSims]);
        
        z = [   z_vel;...
                z_del ];

        kf_vel = sqrt( EKF_x(4,:,:,t).^2 + EKF_x(5,:,:,t).^2 );
        
        H = zeros(2,6,nCars,nSims);
        H(1,4,:,:) = EKF_x(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = EKF_x(5,:,:,t) ./ kf_vel;
        H(2,6,:,:) = 1;
        
        h = [kf_vel; EKF_x(6,:,:,t)];

        R = zeros(2,2,nCars,nSims);
        R(1,1,:,:) = enc_err*enc_err;
        R(2,2,:,:) = imu_mag_err*imu_mag_err ;

        K = pageDiv( pagemtimes(EKF_P,'none',H,'transpose'), (pagemtimes( pagemtimes(H,EKF_P), 'none', H, 'transpose') + ...
            R ));

        EKF_x(:,:,:,t) = EKF_x(:,:,:,t) + reshape( pagemtimes( K, reshape((z-h), [2, 1, nCars, nSims])), [6, nCars, nSims]);
        EKF_P = pagemtimes( (repmat(eye(6), [1,1,nCars,nSims]) - pagemtimes(K,H)), EKF_P);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              for j2 = 1 : nSims
%                 for j1 = 1 : nCars
%                     %Resymmetrize
%                     temp0 = EKF_P(:,:,j1,j2);
%                     temp0 = 0.5*(temp0 + temp0');
%                     EKF_P(:,:,j1,j2) = temp0;           
%                 end
%              end
%    %%%%%%%%%%%%%%%%%%%%%%%%% 
        clear z_vel z_del z kf_vel H h R K
    end

    % GPS step at 10 Hz
    if mod(t, rate/rate_gps) == 0 && sensors(2)

        z_x = x_truth(1,:,t) + normrnd(0, gps_per, [1, nCars, nSims]);
        z_y = x_truth(2,:,t) + normrnd(0, gps_per, [1, nCars, nSims]);
        z_t = x_truth(3,:,t) + normrnd(0, gps_her, [1, nCars, nSims]);
        z_v = x_truth(4,:,t) + normrnd(0, gps_ver, [1, nCars, nSims]);
        
        z = [ z_x; z_y; z_t; z_v ];

        kf_vel = sqrt( EKF_x(4,:,:,t).^2 + EKF_x(5,:,:,t).^2 );
        
        H = zeros(4,6,nCars,nSims);
        H(1,1,:,:) = 1;
        H(2,2,:,:) = 1;
        H(3,3,:,:) = 1;
        H(4,4,:,:) = EKF_x(4,:,:,t) ./ kf_vel;
        H(4,5,:,:) = EKF_x(5,:,:,t) ./ kf_vel;
        
        h = [   EKF_x(1:3,:,:,t);...
                kf_vel];
            
        R = repmat(diag([  ...
            gps_per*gps_per;...
            gps_per*gps_per;...
            gps_her*gps_her;...
            gps_ver*gps_ver]), [1, 1, nCars, nSims]);


        K = pageDiv( pagemtimes(EKF_P,'none',H,'transpose'), ( pagemtimes( pagemtimes( H, EKF_P), 'none', H, 'transpose') + R ) );

        EKF_x(:,:,:,t) = EKF_x(:,:,:,t) + reshape( pagemtimes( K, reshape(z - h, [4, 1, nCars, nSims])), [6, nCars, nSims] );
        EKF_P = pagemtimes( ( repmat( eye(6), [1,1,nCars,nSims] ) - pagemtimes(K,H) ), EKF_P );
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              for j2 = 1 : nSims
%                 for j1 = 1 : nCars
%                     %Resymmetrize
%                     temp0 = EKF_P(:,:,j1,j2);
%                     temp0 = 0.5*(temp0 + temp0');
%                     EKF_P(:,:,j1,j2) = temp0;           
%                 end
%              end
%    %%%%%%%%%%%%%%%%%%%%%%%%% 
        clear z_x z_y z_t z_v z kf_vel H h R K
    end
end
