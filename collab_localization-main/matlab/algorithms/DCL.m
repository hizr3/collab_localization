if t == 1
    cck998 = true;
    cck999 = true;
    cck1000 = true;
    DCL_x = zeros(7, nCars, nSims, nTicks);
    nx = size(DCL_x, 1);
    DCL_x(1:3,:,:,t) = repmat(x_truth(1:3,:,t), [1,1,nSims]);
    DCL_x(4,:,:,t)   = repmat(cos(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    DCL_x(5,:,:,t)   = repmat(sin(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
                   
    DCL_s = repmat(... 
            diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err/ 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err/ rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err;...
                    sa_err*sa_err]), [1,1,nCars,nCars,nSims]);


   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
            for j3 = 1 : nCars
             for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = DCL_s(:,:,j3,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    DCL_s(:,:,j3,j1,j2) = temp0;           
                end
             end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


else

    % Predict Step at 400 Hz
    if mod(t, rate/rate_imu) == 0
        
        accel = [acc(t,:); x_truth(4,:,t).^2 / wb.* tan(del(t,:))] ...
            + normrnd(0, imu_acc_err, [2, nCars, nSims]);
        
        gyro = x_truth(4,:,t) .* tan(del(t,:)) / wb ...
            + normrnd(0, imu_gyr_err, [1, nCars, nSims]);
        
        mag = x_truth(3,:,t) ...
            + normrnd(0, imu_mag_err, [1, nCars, nSims]);
        
        theta = DCL_x(3,:,:,t-1);

        accel_r = [...
            cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ;...
             sin(theta) .* accel(1,:,:) + cos(theta).*accel(2,:,:) ];

        DCL_x(:,:,:,t) = [...
            DCL_x(1,:,:,t-1) + DCL_x(4,:,:,t-1) * dt + accel_r(1,:,:) / 2 * dt^2; ...
            DCL_x(2,:,:,t-1) + DCL_x(5,:,:,t-1) * dt + accel_r(2,:,:) / 2 * dt^2; ...
           (DCL_x(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;...
            DCL_x(4,:,:,t-1) + accel_r(1,:,:) * dt;...
            DCL_x(5,:,:,t-1) + accel_r(2,:,:) * dt;...
            gyro;...
            DCL_x(7,:,:,t-1)];

        
        Q = diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err/ 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err/ rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err;...
                    sa_err*sa_err]);


        F = [   1,  0,  0, dt,  0,  0  ,0;...
                0,  1,  0,  0, dt,  0  ,0;...
                0,  0,  1,  0,  0, dt  ,0;...
                0,  0,  0,  1,  0,  0  ,0;...
                0,  0,  0,  0,  1,  0  ,0;...
                0,  0,  0,  0,  0,  1 , 0;...
                0 , 0,  0,  0,  0,  0,  1;];

        for i = 1:nCars
            DCL_s(:,:,i,i,:) = pagemtimes( pagemtimes( F, DCL_s(:,:,i,i,:)),'none', F, 'transpose') + Q;
            DCL_s(:,:,i,setdiff(1:nCars,i),:) = pagemtimes( F, DCL_s(:,:,i,setdiff(1:nCars,i),:));
        end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
            for j3 = 1 : nCars
             for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = DCL_s(:,:,j3,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    DCL_s(:,:,j3,j1,j2) = temp0;           
                end
             end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        clear accel accel_r gyro mag theta Q F
    end

    % Pacmod Step at 30 Hz
    if mod(t, rate/rate_mdl) == 0 && sensors(1) 


        z_vel = x_truth(4,:,t) + normrnd(0, enc_err, [1, nCars, nSims]);
        z_del = del(t,:) + normrnd(0, sa_err, [1, nCars, nSims]);
        
        z = [   z_vel;...
                z_del];


        kf_vel = sqrt( DCL_x(4,:,:,t).^2 + DCL_x(5,:,:,t).^2 );
        H = zeros(2,7,nCars,nSims);
        H(1,4,:,:) = DCL_x(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = DCL_x(5,:,:,t) ./ kf_vel;
        H(2,7,:,:) = 1;      
        h = [kf_vel; EKF_x(7,:,:,t)];
        R = zeros(2,2,nCars,nSims);
        R(1,1,:,:) = enc_err*enc_err;
        R(2,2,:,:) = sa_err*sa_err;

        for i = 1:nCars
            sigma = reshape(DCL_s(:,:,i,i,:), [7,7,nSims]);
            Hr = reshape(H(:,:,i,:), [2,7,nSims]);
            Rr = reshape(R(:,:,i,:), [2,2,nSims]);
            
            K   = pageDiv( pagemtimes(sigma,'none', Hr,'transpose'),...
                  (pagemtimes( pagemtimes( Hr, sigma), 'none', Hr, 'transpose') + Rr));

            DCL_x(:,i,:,t) = DCL_x(:,i,:,t) + pagemtimes( K, (z(:,i,:)-h(:,i,:)));
            DCL_s(:,:,i,:,:) = pagemtimes( reshape(repmat(eye(7), [1,1,nSims]) - pagemtimes(K, Hr), [7,7,1,1,nSims]), DCL_s(:,:,i,:,:));
        end

        clear i z_vel z_del z kf_vel H h R K sigma Hr Rr
    end

    % GPS step at 10 Hz
    if mod(t, rate/rate_gps) == 0 && sensors(2)

        z_x = x_truth(1,:,t) + normrnd(0, gps_per, [1, nCars, nSims]);
        z_y = x_truth(2,:,t) + normrnd(0, gps_per, [1, nCars, nSims]);
        z_t = x_truth(3,:,t) + normrnd(0, gps_her, [1, nCars, nSims]);
        z_v = x_truth(4,:,t) + normrnd(0, gps_ver, [1, nCars, nSims]);
        
        z = [ z_x; z_y; z_t; z_v ];

        kf_vel = sqrt( DCL_x(4,:,:,t).^2 + DCL_x(5,:,:,t).^2 );
        
        H = zeros(4,7,nCars,nSims);
        H(1,1,:,:) = 1;
        H(2,2,:,:) = 1;
        H(3,3,:,:) = 1;
        H(4,4,:,:) = DCL_x(4,:,:,t) ./ kf_vel;
        H(4,5,:,:) = DCL_x(5,:,:,t) ./ kf_vel;
        
        h = [   DCL_x(1:3,:,:,t);...
                kf_vel];
            
        R = repmat(diag([  ...
            gps_per*gps_per;...
            gps_per*gps_per;...
            gps_her*gps_her;...
            gps_ver*gps_ver]), [1, 1, nCars, nSims]);

        for i = 1:nCars
            sigma = reshape(DCL_s(:,:,i,i,:), [7,7,nSims]);
            Hr = reshape(H(:,:,i,:), [4,7,nSims]);
            Rr = reshape(R(:,:,i,:), [4,4,nSims]);
            
            K   = pageDiv( pagemtimes(sigma,'none', Hr,'transpose'), (pagemtimes( pagemtimes( Hr, sigma), 'none', Hr, 'transpose') + Rr));

            DCL_x(:,i,:,t) = DCL_x(:,i,:,t) + pagemtimes( K, (z(:,i,:)-h(:,i,:)));
            DCL_s(:,:,i,:,:) = pagemtimes( reshape(repmat(eye(7), [1,1,nSims]) - pagemtimes(K, Hr), [7,7,1,1,nSims]), DCL_s(:,:,i,:,:));
        end

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
            for j3 = 1 : nCars
             for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = DCL_s(:,:,j3,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    DCL_s(:,:,j3,j1,j2) = temp0;           
                end
             end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
        clear z_x z_y z_t z_v z kf_vel H h R K sigma Hr Rr i 
    end
    

    % UWB Update Step
    if mod(t, rate/rate_uwb) == 0 && nCars > 1 && sensors(3)
        B = nchoosek(1:nCars,2);
        B_h = [B(:,2) , B(:,1) ];
        B = [ B ; B_h ];
        
        for i = 1:size(B,1)
            z_x = x_truth(1,B(i,1),t) - x_truth(1,B(i,2),t);
            z_y = x_truth(2,B(i,1),t) - x_truth(2,B(i,2),t);
            z = reshape(sqrt(z_x.^2 + z_y.^2) + normrnd(0, uwb_err, [1,nSims]), [1,1,nSims]);

            h_x = DCL_x(1, B(i,1), :, t) - DCL_x(1, B(i,2), :, t);
            h_y = DCL_x(2, B(i,1), :, t) - DCL_x(2, B(i,2), :, t);
            h = sqrt(h_x.^2 + h_y.^2);
            
            Sigma_ii = DCL_s(:, :, B(i,1), B(i,1), :);
            Sigma_jj = DCL_s(:, :, B(i,2), B(i,2), :);
            Sigma_ij = pagemtimes(DCL_s(:,:,B(i,1),B(i,2),:), 'none', DCL_s(:,:,B(i,2),B(i,1),:), 'transpose');
            Sigma_aa = reshape([Sigma_ii, Sigma_ij; pagetranspose(Sigma_ij), Sigma_jj], [14,14,nSims]);

            H = zeros(1,14, nSims);
            H(1,1,:) =   h_x ./ (h);
            H(1,8,:) = - h_x ./ (h);
            H(1,2,:) =   h_y ./ (h);
            H(1,9,:) = - h_y ./ (h);
            R(1,1,nSims) = uwb_err*uwb_err; 
            Ka = pageDiv( pagemtimes(Sigma_aa, 'none', H, 'transpose'), pagemtimes( pagemtimes(H,Sigma_aa), 'none', H, 'transpose') + R );
            update = pagemtimes(Ka, z - h);
            DCL_x(:, B(i,1), :, t) = DCL_x(:, B(i,1), :, t) + update(1:7, 1,:);
            DCL_x(:, B(i,2), :, t) = DCL_x(:, B(i,2), :, t) + update(8:14,1,:);
            
            Sigma_t = pagemtimes(repmat(eye(14),      [1,1,nSims]) - pagemtimes(Ka, H), Sigma_aa);
            
            DCL_s(:,:,B(i,1),setdiff(1:nCars, B(i,:)), :) = pagemtimes(pageDiv(Sigma_t(1:7,  1:7,  :), Sigma_ii), DCL_s(:,:,B(i,1),setdiff(1:nCars, B(i,:)), :));
            DCL_s(:,:,B(i,2),setdiff(1:nCars, B(i,:)), :) = pagemtimes(pageDiv(Sigma_t(8:14, 8:14, :), Sigma_jj), DCL_s(:,:,B(i,2),setdiff(1:nCars, B(i,:)), :));

            DCL_s(:,:,B(i,1),B(i,1),:) = Sigma_t(1:7,  1:7,  :);
            DCL_s(:,:,B(i,2),B(i,2),:) = Sigma_t(8:14, 8:14, :);
            DCL_s(:,:,B(i,1),B(i,2),:) = Sigma_t(1:7,  8:14, :);
            DCL_s(:,:,B(i,2),B(i,1),:) = repmat(eye(7), [1,1,nSims]); % set the sqrt of sigma to identity

            if t == 998 && cck998 == true
               t
               cck998 = false;
               Sigma_aa(:,:,1)
               eigSigma = eig(Sigma_aa(:,:,i))
               H(:,:,1)
               R(:,:,1)
               Ka(:,:,1)
               Sigma_t(:,:,1)
            end
            if t == 999 && cck999 == true
                t
               cck999 = false;
               Sigma_aa(:,:,1)
               eigSigma = eig(Sigma_aa(:,:,i))
               H(:,:,1)
               R(:,:,1)
               Ka(:,:,1)
               Sigma_t(:,:,1)
            end
            if t == 1000 && cck1000 == true
                t
               cck1000 = false;
               Sigma_aa(:,:,1)
               eigSigma = eig(Sigma_aa(:,:,i))
               H(:,:,1)
               R(:,:,1)
               Ka(:,:,1)
               Sigma_t(:,:,1)
            end

        end
        clear z_x z_y z H h R K i B h_x h_y

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
            for j3 = 1 : nCars
             for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = DCL_s(:,:,j3,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    DCL_s(:,:,j3,j1,j2) = temp0;           
                end
             end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    end


DEBUG_DCL_P(:,:,t) = DCL_s(:,:,1,1,1);





end
