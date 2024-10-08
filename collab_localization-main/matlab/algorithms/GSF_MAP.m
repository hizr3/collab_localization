if t == 1 %t scorre fino a nTicks
    GSF_MAP_x = zeros(6, nCars, nSims, nTicks);
    GSF_MAP_x(1:3,:,:,t) = repmat(x_truth(1:3,:,t), [1,1,nSims]);
    GSF_MAP_x(4,:,:,t)   = repmat(cos(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    GSF_MAP_x(5,:,:,t)   = repmat(sin(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    
    GSF_MAP_P = repmat(... 
            diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err/ 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err/ rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err]), [1,1,nCars,nSims]);

    nx = size(GSF_MAP_P(:,:,1,1) , 1);
    % I implement the GSF as a bank 3 EKFs. I need a data structure
    % that can store each individual filter, as well as weights.
    % The EKFs are using the CT , CV and CA models, respectively
    nModes = 3;
    weights = ones(nModes , nCars , nSims )/nModes;
    weights = log(weights);

    EKF_x_gsf = zeros(6, nCars, nSims, nTicks);
    EKF_x_gsf(1:3,:,:,t) = repmat(x_truth(1:3,:,t), [1,1,nSims]);
    EKF_x_gsf(4,:,:,t)   = repmat(cos(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    EKF_x_gsf(5,:,:,t)   = repmat( sin(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    
    EKF_P_gsf = repmat(... 
            diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err/ 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err/ rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err ]), [1,1,nCars,nSims]);
    
              for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = EKF_P_gsf(:,:,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    EKF_P_gsf(:,:,j1,j2) = temp0;           
                end
              end
              clear temp0

    GSF_MAP_struct = struct( "w" , weights, ... 
                         "CA_x" , EKF_x_gsf , "CA_P" , EKF_P_gsf , ...
                         "CT_x" , EKF_x_gsf , "CT_P" , EKF_P_gsf , ...
                         "CV_x" , EKF_x_gsf , "CV_P" , EKF_P_gsf );
clear EKF_x_gsf EKF_P_gsf


                
else

    % Predict Step at 400 Hz
    if mod(t, rate/rate_imu) == 0
    
    % Filter inputs
        accel = [acc(t,:); x_truth(4,:,t).^2 / wb.* tan(del(t,:))] ...
            + normrnd(0, imu_acc_err, [2, nCars, nSims]);
        
        gyro = x_truth(4,:,t) .* tan(del(t,:)) / wb ...
            + normrnd(0, imu_gyr_err, [1, nCars, nSims]);
        
        mag = x_truth(3,:,t) ...
            + normrnd(0, imu_mag_err, [1, nCars, nSims]);
        

        Q = diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err]);   

    % Begin CA model

        theta = GSF_MAP_struct.CA_x(3,:,:,t-1);
        % 
         accel_r = [...
             cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ;...
             sin(theta) .* accel(1,:,:) + cos(theta).*accel(2,:,:) ];



        GSF_MAP_struct.CA_x(:,:,:,t) = [...
                GSF_MAP_struct.CA_x(1,:,:,t-1) + GSF_MAP_struct.CA_x(4,:,:,t-1) * dt + accel_r(1,:,:) / 2 * dt^2; ...
                GSF_MAP_struct.CA_x(2,:,:,t-1) + GSF_MAP_struct.CA_x(5,:,:,t-1) * dt + accel_r(2,:,:) / 2 * dt^2; ...
               (GSF_MAP_struct.CA_x(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;...
                GSF_MAP_struct.CA_x(4,:,:,t-1) + accel_r(1,:,:) * dt;...
                GSF_MAP_struct.CA_x(5,:,:,t-1) + accel_r(2,:,:) * dt;...
                gyro];

        F_CA = [1,  0,  0, dt,  0,  0;...
                0,  1,  0,  0, dt,  0 ;...
                0,  0,  1,  0,  0, dt ;...
                0,  0,  0,  1,  0,  0 ;...
                0,  0,  0,  0,  1,  0 ;...
                0,  0,  0,  0,  0,  1 ];


        % % Pagemtimes does matmult along the third dimension ( batch dimension )
        % Since its           
        GSF_MAP_struct.CA_P = pagemtimes(...
                    pagemtimes(...
                        F_CA, ...
                        GSF_MAP_struct.CA_P),'none',...
                    F_CA, 'transpose') + Q;  
    % End CA model

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Begin CT model

        theta = GSF_MAP_struct.CT_x(3,:,:,t-1);

        accel_r = [...
             cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ;...
             sin(theta) .* accel(1,:,:) + cos(theta).*accel(2,:,:) ];


        ct_zeroturn = zeros(size(GSF_MAP_struct.CT_x(6,:,:,t-1)));



        GSF_MAP_struct.CT_x(:,:,:,t) = [...
            GSF_MAP_struct.CT_x(1,:,:,t-1) + GSF_MAP_struct.CT_x(4,:,:,t-1) * dt + accel_r(1,:,:) / 2 * dt^2; ...
            GSF_MAP_struct.CT_x(2,:,:,t-1) + GSF_MAP_struct.CT_x(5,:,:,t-1) * dt + accel_r(2,:,:) / 2 * dt^2; ...
           (GSF_MAP_struct.CT_x(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;...
            GSF_MAP_struct.CT_x(4,:,:,t-1) + accel_r(1,:,:) * dt;...
            GSF_MAP_struct.CT_x(5,:,:,t-1) + accel_r(2,:,:) * dt;...
            ct_zeroturn];
        
        F_CT = [ 1,  0,  0, dt,  0,  0  ;...
                0,  1,  0,  0, dt,  0  ;...
                0,  0,  1,  0,  0, dt  ;...
                0,  0,  0,  1,  0,  0  ;...
                0,  0,  0,  0,  1,  0  ;...
                0,  0,  0,  0,  0,  0 ];

        GSF_MAP_struct.CT_P = pagemtimes(...
                    pagemtimes(...
                        F_CT, ...
                        GSF_MAP_struct.CT_P),'none',...
                    F_CT, 'transpose') + Q;

    % End CT Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Begin CV model
        Q_CV = diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err]);


        theta = GSF_MAP_struct.CV_x(3,:,:,t-1);
        accel_r = [...
             cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ;...
             sin(theta) .* accel(1,:,:) + cos(theta).*accel(2,:,:) ];

        GSF_MAP_struct.CV_x(:,:,:,t) = ...
        [ GSF_MAP_struct.CV_x(1,:,:,t-1) + GSF_MAP_struct.CV_x(4,:,:,t-1) * dt ; ...
          GSF_MAP_struct.CV_x(2,:,:,t-1) + GSF_MAP_struct.CV_x(5,:,:,t-1) * dt ; ...
         (GSF_MAP_struct.CV_x(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;... % comp filter ?
          GSF_MAP_struct.CV_x(4,:,:,t-1) ;...
          GSF_MAP_struct.CV_x(5,:,:,t-1) ;...
          gyro];

        F_CV = [ 1,  0,  0, dt,  0,  0;...
                0,  1,  0,  0, dt,  0  ;...
                0,  0,  1,  0,  0, dt  ;...
                0,  0,  0,  1,  0,  0  ;...
                0,  0,  0,  0,  1,  0  ;...
                0,  0,  0,  0,  0,  1  ];

        GSF_MAP_struct.CV_P = pagemtimes(...
                    pagemtimes(...
                        F_CV, ...
                        GSF_MAP_struct.CV_P),'none',...
                    F_CV, 'transpose') + Q_CV;

    % End CV model


    clear accel accel_r gyro mag theta Q F_CA F_CT F_CV
    end

    % Pacmod Step at 30 Hz
    if mod(t, rate/rate_mdl) == 0 && sensors(1)


    %%%%%%NEW MEAS%%%%%%%%%
    
        z_vel = x_truth(4,:,t) + normrnd(0, enc_err, [1, nCars, nSims]); % Measures steering angle
        z_del =  x_truth(4,:,t).*tan ( del(t,:) ) /wb + normrnd(0, imu_mag_err, [1, nCars, nSims]); 
        z = [   z_vel;...
                z_del];
%%%%%%%%%%%%%%%%%%%
%Begin CA model
        kf_vel = sqrt( GSF_MAP_struct.CA_x(4,:,:,t).^2 + GSF_MAP_struct.CA_x(5,:,:,t).^2 );
        H = zeros(2,nx,nCars,nSims);
        H(1,4,:,:) = GSF_MAP_struct.CA_x(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = GSF_MAP_struct.CA_x(5,:,:,t) ./ kf_vel;
        H(2,6,:,:) = 1;

        h = [kf_vel; GSF_MAP_struct.CA_x(6,:,:,t)];

        R = zeros(2,2,nCars,nSims);
        R(1,1,:,:) = enc_err*enc_err;
        R(2,2,:,:) = imu_mag_err*imu_mag_err;
        R_single = R(:,:,1,1);


        z_res = z - h;
        K = pageDiv( pagemtimes(GSF_MAP_struct.CA_P,'none',H,'transpose'), (pagemtimes( pagemtimes(H,GSF_MAP_struct.CA_P), 'none', H, 'transpose') + R));
        GSF_MAP_struct.CA_x(:,:,:,t) = GSF_MAP_struct.CA_x(:,:,:,t) + reshape( pagemtimes( K, reshape((z_res), [2, 1, nCars, nSims])), [nx, nCars, nSims]);
        GSF_MAP_struct.CA_P = pagemtimes( (repmat(eye(nx), [1,1,nCars,nSims]) - pagemtimes(K,H)), GSF_MAP_struct.CA_P);

       
          % End CA model
        % Begin pacmod CA weight update
            for j2 = 1 : nSims
                for j1 = 1 : nCars
                    temp0 = GSF_MAP_struct.CA_P(:,:,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    GSF_MAP_struct.CA_P(:,:,j1,j2) = temp0;           
                    S_CA = R_single + H(:,:,j1,j2)*temp0*H(:,:,j1,j2)';
                    quadform = z_res(:,j1,j2)' /S_CA *  z_res(:,j1,j2);
                     likCA =  log(det(S_CA)) + quadform +  nx * log(2 * pi);
                     likCA = -0.5 * likCA;
                     GSF_MAP_struct.w(1 , j1 ,  j2 )= GSF_MAP_struct.w(1 , j1 ,  j2 ) + likCA ; 
                end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Begin CT model
       
        kf_vel = sqrt( GSF_MAP_struct.CT_x(4,:,:,t).^2 + GSF_MAP_struct.CT_x(5,:,:,t).^2 );
        H = zeros(2,6,nCars,nSims);
        H(1,4,:,:) = GSF_MAP_struct.CT_x(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = GSF_MAP_struct.CT_x(5,:,:,t) ./ kf_vel;
        H(2,6,:,:) = 1;
        h = [kf_vel; GSF_MAP_struct.CT_x(6,:,:,t)];

        z_res = z - h;
        K = pageDiv( pagemtimes(GSF_MAP_struct.CT_P,'none',H,'transpose'), (pagemtimes( pagemtimes(H,GSF_MAP_struct.CT_P), 'none', H, 'transpose') + R));
        GSF_MAP_struct.CT_x(:,:,:,t) = GSF_MAP_struct.CT_x(:,:,:,t) + reshape( pagemtimes( K, reshape((z_res), [2, 1, nCars, nSims])), [nx, nCars, nSims]);
        GSF_MAP_struct.CT_P = pagemtimes( (repmat(eye(nx), [1,1,nCars,nSims]) - pagemtimes(K,H)), GSF_MAP_struct.CT_P);
        % End CT model
        % Begin pacmod CT weight update
            for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = GSF_MAP_struct.CT_P(:,:,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    GSF_MAP_struct.CT_P(:,:,j1,j2) = temp0;           
                    S_CT = R_single + H(:,:,j1,j2)*temp0*H(:,:,j1,j2)';
                    quadform = z_res(:,j1,j2)' /S_CT *  z_res(:,j1,j2);
                     likCT =  log(det(S_CT)) + quadform +  nx * log(2 * pi);
                     likCT = -0.5 * likCT;
                     GSF_MAP_struct.w(2 , j1 ,  j2 )= GSF_MAP_struct.w(2 , j1 ,  j2 ) +  likCT ; 
                end
 end
        % End CT weight update
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Begin CV model
        kf_vel = sqrt( GSF_MAP_struct.CV_x(4,:,:,t).^2 + GSF_MAP_struct.CV_x(5,:,:,t).^2 );
        H = zeros(2,6,nCars,nSims);
        H(1,4,:,:) = GSF_MAP_struct.CV_x(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = GSF_MAP_struct.CV_x(5,:,:,t) ./ kf_vel;
        H(2,6,:,:) = 1;
        h = [kf_vel; GSF_MAP_struct.CV_x(6,:,:,t)];

        
        z_res = z - h;
        K = pageDiv( pagemtimes(GSF_MAP_struct.CV_P,'none',H,'transpose'), (pagemtimes( pagemtimes(H,GSF_MAP_struct.CV_P), 'none', H, 'transpose') + R));
        GSF_MAP_struct.CV_x(:,:,:,t) = GSF_MAP_struct.CV_x(:,:,:,t) + reshape( pagemtimes( K, reshape((z_res), [2, 1, nCars, nSims])), [nx, nCars, nSims]);
        GSF_MAP_struct.CV_P = pagemtimes( (repmat(eye(nx), [1,1,nCars,nSims]) - pagemtimes(K,H)), GSF_MAP_struct.CV_P);
        % End CV Model
   
        %Begin pacmod CV weighht update
            for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = GSF_MAP_struct.CV_P(:,:,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    GSF_MAP_struct.CV_P(:,:,j1,j2) = temp0;           
                    S_CV = R_single + H(:,:,j1,j2)*temp0*H(:,:,j1,j2)';
                    quadform = z_res(:,j1,j2)' /S_CV *  z_res(:,j1,j2);
                     likCV =  log(det(S_CV)) + quadform +  nx * log(2 * pi);
                     likCV = -0.5 * likCV;
                     GSF_MAP_struct.w(3 , j1 ,  j2 )= GSF_MAP_struct.w(3 , j1 ,  j2 ) + likCV ; 
                end
 end
        % End cv weight updatee
        
        clear z_vel z_del z kf_vel H h  K likCA likCT likCV quadform


        %Normalize log weights
 for j2 = 1 : nSims
    for j1 = 1 : nCars
        GSF_MAP_struct.w(:,j1,j2) = normalizeLogWeights(GSF_MAP_struct.w(:,j1,j2));
    end
 end


    end

    % GNSS step at 10 Hz
    if mod(t, rate/rate_gps) == 0 && sensors(2)

        z_x = x_truth(1,:,t) + normrnd(0, gps_per, [1, nCars, nSims]);
        z_y = x_truth(2,:,t) + normrnd(0, gps_per, [1, nCars, nSims]);
        z_t = x_truth(3,:,t) + normrnd(0, gps_her, [1, nCars, nSims]);
        z_v = x_truth(4,:,t) + normrnd(0, gps_ver, [1, nCars, nSims]); 
        z = [ z_x; z_y; z_t; z_v ];


        % Begin CA model
        kf_vel = sqrt( GSF_MAP_struct.CA_x(4,:,:,t).^2 + GSF_MAP_struct.CA_x(5,:,:,t).^2 );
        H = zeros(4,6,nCars,nSims);
        H(1,1,:,:) = 1;
        H(2,2,:,:) = 1;
        H(3,3,:,:) = 1;
        H(4,4,:,:) = GSF_MAP_struct.CA_x(4,:,:,t) ./ kf_vel;
        H(4,5,:,:) = GSF_MAP_struct.CA_x(5,:,:,t) ./ kf_vel;

        h = [   GSF_MAP_struct.CA_x(1:3,:,:,t);...
                kf_vel];   


        R = repmat(diag([  ...
            gps_per*gps_per;...
            gps_per*gps_per;...
            gps_her*gps_her;...
            gps_ver*gps_ver]), [1, 1, nCars, nSims]);

        R_single = R(:,:,1,1);

        z_res = z - h ;
        K = pageDiv( pagemtimes(GSF_MAP_struct.CA_P,'none',H,'transpose'), ( pagemtimes( pagemtimes( H, GSF_MAP_struct.CA_P), 'none', H, 'transpose') + R ) );

        GSF_MAP_struct.CA_x(:,:,:,t) = GSF_MAP_struct.CA_x(:,:,:,t) + reshape( pagemtimes( K, reshape(z_res, [4, 1, nCars, nSims])), [nx, nCars, nSims] );
        GSF_MAP_struct.CA_P = pagemtimes( ( repmat( eye(nx), [1,1,nCars,nSims] ) - pagemtimes(K,H) ), GSF_MAP_struct.CA_P );
        % End CA model


        % Begin GNSS CA weight update
            for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = GSF_MAP_struct.CA_P(:,:,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    GSF_MAP_struct.CA_P(:,:,j1,j2) = temp0;           
                    S_CA = R_single + H(:,:,j1,j2)*temp0*H(:,:,j1,j2)';
                    % chol( S_CA )
                    % det(S_CA)
                    quadform = z_res(:,j1,j2)' /S_CA *  z_res(:,j1,j2);
                     likCA =  log(det(S_CA)) + quadform +  nx * log(2 * pi);
                     likCA = -0.5 * likCA;
                     GSF_MAP_struct.w(1 , j1 ,  j2 )= GSF_MAP_struct.w(1 , j1 ,  j2 ) + likCA ; 
                end
end


        % Begin CT model
        kf_vel = sqrt( GSF_MAP_struct.CT_x(4,:,:,t).^2 + GSF_MAP_struct.CT_x(5,:,:,t).^2 );
        H = zeros(4,6,nCars,nSims);
        H(1,1,:,:) = 1;
        H(2,2,:,:) = 1;
        H(3,3,:,:) = 1;
        H(4,4,:,:) = GSF_MAP_struct.CT_x(4,:,:,t) ./ kf_vel;
        H(4,5,:,:) = GSF_MAP_struct.CT_x(5,:,:,t) ./ kf_vel;
        h = [   GSF_MAP_struct.CT_x(1:3,:,:,t);...
                kf_vel];   
        
        z_res = z - h;
        K = pageDiv( pagemtimes(GSF_MAP_struct.CT_P,'none',H,'transpose'), ( pagemtimes( pagemtimes( H, GSF_MAP_struct.CT_P), 'none', H, 'transpose') + R ) );
        GSF_MAP_struct.CT_x(:,:,:,t) = GSF_MAP_struct.CT_x(:,:,:,t) + reshape( pagemtimes( K, reshape(z_res, [4, 1, nCars, nSims])), [nx, nCars, nSims] );
        GSF_MAP_struct.CT_P = pagemtimes( ( repmat( eye(nx), [1,1,nCars,nSims] ) - pagemtimes(K,H) ), GSF_MAP_struct.CT_P );

        % End CT model

        % Begin GNSS CT weight update
            for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = GSF_MAP_struct.CT_P(:,:,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    GSF_MAP_struct.CT_P(:,:,j1,j2) = temp0;           
                    S_CT = R_single + H(:,:,j1,j2)*temp0*H(:,:,j1,j2)';
                    quadform = z_res(:,j1,j2)' /S_CT *  z_res(:,j1,j2);
                     likCT =  log(det(S_CT)) + quadform +  nx * log(2 * pi);
                     likCT = -0.5 * likCT;
                     GSF_MAP_struct.w(2 , j1 ,  j2 )= GSF_MAP_struct.w(2 , j1 ,  j2 ) + likCT ; 
                end
            
end
        % End CT weight update 




        % Begin CV model
        kf_vel = sqrt( GSF_MAP_struct.CV_x(4,:,:,t).^2 + GSF_MAP_struct.CV_x(5,:,:,t).^2 );
        H = zeros(4,6,nCars,nSims);
        H(1,1,:,:) = 1;
        H(2,2,:,:) = 1;
        H(3,3,:,:) = 1;
        H(4,4,:,:) = GSF_MAP_struct.CV_x(4,:,:,t) ./ kf_vel;
        H(4,5,:,:) = GSF_MAP_struct.CV_x(5,:,:,t) ./ kf_vel;
        h = [   GSF_MAP_struct.CV_x(1:3,:,:,t);...
                kf_vel];   

        z_res = z - h;
        K = pageDiv( pagemtimes(GSF_MAP_struct.CV_P,'none',H,'transpose'), ( pagemtimes( pagemtimes( H, GSF_MAP_struct.CV_P), 'none', H, 'transpose') + R ) );
        GSF_MAP_struct.CV_x(:,:,:,t) = GSF_MAP_struct.CV_x(:,:,:,t) + reshape( pagemtimes( K, reshape(z_res, [4, 1, nCars, nSims])), [nx, nCars, nSims] );
        GSF_MAP_struct.CV_P = pagemtimes( ( repmat( eye(nx), [1,1,nCars,nSims] ) - pagemtimes(K,H) ), GSF_MAP_struct.CV_P );
        % End CV model

        % Begin GNSS CV weight update
            for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = GSF_MAP_struct.CV_P(:,:,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    GSF_MAP_struct.CV_P(:,:,j1,j2) = temp0;           
                    S_CV= R_single + H(:,:,j1,j2)*temp0*H(:,:,j1,j2)';
                    % chol( S_CV )
                    % det(S_CV)
                    quadform = z_res(:,j1,j2)' /S_CV *  z_res(:,j1,j2);
                     likCV =  log(det(S_CV)) + quadform +  nx * log(2 * pi);
                     likCV = -0.5 * likCV;
                     GSF_MAP_struct.w(3 , j1 ,  j2 )= GSF_MAP_struct.w(3 , j1 ,  j2 ) + likCV ; 
                end
            
end
        % End CV  weight update
  
        
        
        clear z_vel z_del z kf_vel H h  K likCA likCT likCV quadform
        %Normalize log weights
 for j2 = 1 : nSims
    for j1 = 1 : nCars
        GSF_MAP_struct.w(:,j1,j2) = normalizeLogWeights(GSF_MAP_struct.w(:,j1,j2));
    end
 end

 end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%Begin Fusion of hypotheses

GSF_MAP_struct.w = exp(GSF_MAP_struct.w);
% Begin MAP
for j2 = 1 : nSims
    for j1 = 1 : nCars

        [~, maxIdx] = max(GSF_MAP_struct.w(:,j1,j2));
        if maxIdx == 1
                GSF_MAP_x(:,j1,j2,t) = GSF_MAP_struct.CA_x(:,j1,j2,t);
         elseif maxIdx == 2
                GSF_MAP_x(:,j1,j2,t) = GSF_MAP_struct.CT_x(:,j1,j2,t);
          else
                GSF_MAP_x(:,j1,j2,t) = GSF_MAP_struct.CV_x(:,j1,j2,t);
           end

    end
end

for j2 = 1 : nSims
    for j1 = 1 : nCars

        if maxIdx == 1
            GSF_MAP_P(:,:,j1,j2) = GSF_MAP_struct.CA_P(:,:,j1,j2);
        elseif maxIdx == 2

            GSF_MAP_P(:,:,j1,j2) = GSF_MAP_struct.CT_P(:,:,j1,j2);
        else

            GSF_MAP_P(:,:,j1,j2) = GSF_MAP_struct.CV_P(:,:,j1,j2);
        end

    end
end

% %End of MAP

GSF_MAP_struct.w = log( GSF_MAP_struct.w);


end

































function [log_w] = normalizeLogWeights(log_w)

%
% Input:
% log_w - log weights, e.g., log likelihoods
%
% Output:
% log_w - log of the normalized weights
% log_sum_w - log of the sum of the non-normalized weights
%

if length(log_w)<=1
    % Log of sum of weights times prior probabilities
    log_sum_w = log_w;
    % Normalize
    log_w = log_w-log_sum_w;
elseif length(log_w)>1
    % Log of sum of weights times prior probabilities
    [log_w_aux,I] = sort(log_w,'descend');
    log_sum_w = max(log_w_aux)+log(1+sum(exp(log_w(I(2:end))-max(log_w_aux))));
    % Normalize
    log_w = log_w-log_sum_w;
end
end