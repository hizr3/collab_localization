if t == 1
    disp_upd = false;
    uDCL_x = zeros(6, nCars, nSims, nTicks);
    nx = size(uDCL_x, 1);
    uDCL_x(1:3,:,:,t) = repmat(x_truth(1:3,:,t), [1,1,nSims]);
    uDCL_x(4,:,:,t)   = repmat(cos(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
    uDCL_x(5,:,:,t)   = repmat(sin(x_truth(3,:,t)) .* x_truth(4,:,t), [1,1,nSims]);
                   
    uDCL_s = repmat(... 
            diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err/ 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err/ rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err]), [1,1,nCars,nCars,nSims]);


   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
            for j3 = 1 : nCars
             for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = uDCL_s(:,:,j3,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    uDCL_s(:,:,j3,j1,j2) = temp0;           
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
        
        theta = uDCL_x(3,:,:,t-1);

        accel_r = [...
            cos(theta) .* accel(1,:,:) - sin(theta).*accel(2,:,:) ;...
             sin(theta) .* accel(1,:,:) + cos(theta).*accel(2,:,:) ];

        uDCL_x(:,:,:,t) = [...
            uDCL_x(1,:,:,t-1) + uDCL_x(4,:,:,t-1) * dt + accel_r(1,:,:) / 2 * dt^2; ...
            uDCL_x(2,:,:,t-1) + uDCL_x(5,:,:,t-1) * dt + accel_r(2,:,:) / 2 * dt^2; ...
           (uDCL_x(3,:,:,t-1) + gyro * dt) * 0.98 + 0.02 * mag;...
            uDCL_x(4,:,:,t-1) + accel_r(1,:,:) * dt;...
            uDCL_x(5,:,:,t-1) + accel_r(2,:,:) * dt;...
            gyro];

        
        Q = diag([  imu_acc_err*imu_acc_err / 2 / rate_imu^2    ;...
                    imu_acc_err*imu_acc_err/ 2 / rate_imu^2    ;...
                    imu_gyr_err*imu_gyr_err/ rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_acc_err*imu_acc_err / rate_imu          ;...
                    imu_gyr_err*imu_gyr_err]);


             F = [   1,  0,  0, dt,  0,  0 ;...
                0,  1,  0,  0, dt,  0 ;...
                0,  0,  1,  0,  0, dt ;...
                0,  0,  0,  1,  0,  0 ;...
                0,  0,  0,  0,  1,  0 ;...
                0,  0,  0,  0,  0,  1 ];

        for i = 1:nCars
            uDCL_s(:,:,i,i,:) = pagemtimes( pagemtimes( F, uDCL_s(:,:,i,i,:)),'none', F, 'transpose') + Q;
            uDCL_s(:,:,i,setdiff(1:nCars,i),:) = pagemtimes( F, uDCL_s(:,:,i,setdiff(1:nCars,i),:)); % updates cross-deviation
        end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
            for j3 = 1 : nCars
             for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = uDCL_s(:,:,j3,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    uDCL_s(:,:,j3,j1,j2) = temp0;           
                end
             end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        clear accel accel_r gyro mag theta Q F
    end

    % Pacmod Step at 30 Hz
    if mod(t, rate/rate_mdl) == 0 && sensors(1) 


        z_vel = x_truth(4,:,t) + normrnd(0, enc_err, [1, nCars, nSims]);
        z_del = x_truth(4,:,t).*tan( del(t,:) ) /wb + normrnd(0, imu_mag_err, [1, nCars, nSims]);
        
        z = [   z_vel;...
                z_del ];


        kf_vel = sqrt( uDCL_x(4,:,:,t).^2 + uDCL_x(5,:,:,t).^2 );
        H = zeros(2,6,nCars,nSims);
        H(1,4,:,:) = uDCL_x(4,:,:,t) ./ kf_vel;
        H(1,5,:,:) = uDCL_x(5,:,:,t) ./ kf_vel;
        H(2,6,:,:) = 1;      
        h = [kf_vel; uDCL_x(6,:,:,t)];
        R = zeros(2,2,nCars,nSims);
        R(1,1,:,:) = enc_err*enc_err;
        R(2,2,:,:) = imu_mag_err*imu_mag_err;

        for i = 1:nCars
            sigma = reshape(uDCL_s(:,:,i,i,:), [6,6,nSims]);
            Hr = reshape(H(:,:,i,:), [2,6,nSims]);
            Rr = reshape(R(:,:,i,:), [2,2,nSims]);
            
            K   = pageDiv( pagemtimes(sigma,'none', Hr,'transpose'),...
                  (pagemtimes( pagemtimes( Hr, sigma), 'none', Hr, 'transpose') + Rr));

            uDCL_x(:,i,:,t) = uDCL_x(:,i,:,t) + pagemtimes( K, (z(:,i,:)-h(:,i,:)));
            uDCL_s(:,:,i,:,:) = pagemtimes( reshape(repmat(eye(6), [1,1,nSims]) - pagemtimes(K, Hr), [6,6,1,1,nSims]), uDCL_s(:,:,i,:,:));
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

        kf_vel = sqrt( uDCL_x(4,:,:,t).^2 + uDCL_x(5,:,:,t).^2 );
        
        H = zeros(4,6,nCars,nSims);
        H(1,1,:,:) = 1;
        H(2,2,:,:) = 1;
        H(3,3,:,:) = 1;
        H(4,4,:,:) = uDCL_x(4,:,:,t) ./ kf_vel;
        H(4,5,:,:) = uDCL_x(5,:,:,t) ./ kf_vel;
        
        h = [   uDCL_x(1:3,:,:,t);...
                kf_vel];
            
        R = repmat(diag([  ...
            gps_per*gps_per;...
            gps_per*gps_per;...
            gps_her*gps_her;...
            gps_ver*gps_ver]), [1, 1, nCars, nSims]);

        for i = 1:nCars
            sigma = reshape(uDCL_s(:,:,i,i,:), [6,6,nSims]);
            Hr = reshape(H(:,:,i,:), [4,6,nSims]);
            Rr = reshape(R(:,:,i,:), [4,4,nSims]);
            
            K   = pageDiv( pagemtimes(sigma,'none', Hr,'transpose'), (pagemtimes( pagemtimes( Hr, sigma), 'none', Hr, 'transpose') + Rr));

            uDCL_x(:,i,:,t) = uDCL_x(:,i,:,t) + pagemtimes( K, (z(:,i,:)-h(:,i,:)));
            uDCL_s(:,:,i,:,:) = pagemtimes( reshape(repmat(eye(6), [1,1,nSims]) - pagemtimes(K, Hr), [6,6,1,1,nSims]), uDCL_s(:,:,i,:,:));
        end

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
            for j3 = 1 : nCars
             for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = uDCL_s(:,:,j3,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    uDCL_s(:,:,j3,j1,j2) = temp0;           
                end
             end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
        clear z_x z_y z_t z_v z kf_vel H h R K sigma Hr Rr i 
    end
    

    % uKF UWB Update Step
    if mod(t, rate/rate_uwb) == 0 && nCars > 1 && sensors(3)

      B = nchoosek(1:nCars,2);
      R_uwb = uwb_err*uwb_err;
      kappa = 0.5;
      idx1 = [1:nx];
      idx2 = idx1 + nx;

       for k = 1 : nSims
        for i = 1:size(B,1)
            %Measurement
            z_x = x_truth(1,B(i,1),t) - x_truth(1,B(i,2),t);
            z_y = x_truth(2,B(i,1),t) - x_truth(2,B(i,2),t);
            z = sqrt(z_x*z_x + z_y*z_y) + normrnd(0, uwb_err);
            
            % Process 
            Sigma_ii = uDCL_s(:, :, B(i,1), B(i,1), k); 
            Sigma_jj = uDCL_s(:, :, B(i,2), B(i,2), k);
            Sigma_ij = uDCL_s(:,:,B(i,1),B(i,2),k)*uDCL_s(:,:,B(i,2),B(i,1),k)';

            % Extended state covariance matrix
            Sigma_aa = [Sigma_ii, Sigma_ij; Sigma_ij', Sigma_jj];
            Sigma_aa = 0.5*(Sigma_aa + Sigma_aa');
            uDCL_x_aa = [  uDCL_x(:, B(i,1), k, t) ; ...
                           uDCL_x(:, B(i,2), k, t) ] ;

            % UKF update step
            [x_sp, w_sp] = ut(uDCL_x_aa,Sigma_aa,kappa);
            z_sp = splitapply(@measfunc,x_sp, [1:length(w_sp)]);
            z_hat = sum(w_sp.*z_sp , 2);
            z_res = z - z_hat;
            Py = mycov(w_sp,z_sp,z_hat);
            S_uwb = R_uwb + Py;
            K_uwb = myxcov(w_sp,x_sp, uDCL_x_aa , z_sp,z_hat)/S_uwb;
            uDCL_x_aa = uDCL_x_aa + K_uwb*z_res;
            Sigma_aa = Sigma_aa - K_uwb*Py*(K_uwb');
            Sigma_aa = 0.5*(Sigma_aa + Sigma_aa');

             % Last step of uDCL Update. i and j share info with the platoon
              upd_idx = setdiff(1:nCars, B(i,:));
              if disp_upd
                  upd_idx
              end
               if not(isempty(upd_idx))
                for p = 1 : size(upd_idx,1)
                    uDCL_s(:,:,B(i,1), upd_idx(p) , k) = ...
                       ( Sigma_aa(idx1,idx1)/ uDCL_s(:,:,B(i,1),B(i,1),k) )*uDCL_s(:,:,B(i,1),p, k) ;
    
                    uDCL_s(:,:,B(i,2), upd_idx(p) , k) = ...
                       ( Sigma_aa(idx2,idx2)/ uDCL_s(:,:,B(i,2),B(i,2),k) )*uDCL_s(:,:,B(i,2),p, k) ;
                end
               end
            % if nCars = 3, for example, the above for becomes just 2
            % assignments

            % update cov ingfo
            uDCL_s(:,:,B(i,1),B(i,1),k) = Sigma_aa(idx1,idx1);
            uDCL_s(:,:,B(i,2),B(i,2),k) = Sigma_aa(idx2,idx2);
            uDCL_s(:,:,B(i,1),B(i,2),k) = Sigma_aa(idx1,idx2);
            uDCL_s(:,:,B(i,2),B(i,1),k) = eye(nx);


            uDCL_x(:, B(i,1), k, t) = uDCL_x_aa(idx1);
            uDCL_x(:, B(i,2), k, t) = uDCL_x_aa(idx2);
            

         end
       end
       
       clear z_x z_y z H z_hat R_uwb K_uwb i B h_x h_y z_res z_sp w_sp Py uDCL_x_aa Sigma_aa idx2 upd_idx kappa

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
            for j3 = 1 : nCars
             for j2 = 1 : nSims
                for j1 = 1 : nCars
                    %Resymmetrize
                    temp0 = uDCL_s(:,:,j3,j1,j2);
                    temp0 = 0.5*(temp0 + temp0');
                    uDCL_s(:,:,j3,j1,j2) = temp0;           
                end
             end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end



end



function [sigma_points, weights] = ut(x,P,kappa)
% Usage: pass column vector x and matrix P. Returns timeseries of sigma points along with weights

nx = size(x,1);
%sqrtScaledCov = chol((kappa+nx)*(P + P' )*0.5  ); 
[U, S, V]  =  svd((kappa + nx)*0.5*(P + P'));
S = 0.5*(S + S');
sqrtScaledCov = U* sqrt(S) * V';
sigma_points = zeros(nx,2*nx); 
weights = zeros(1,2*nx);
    for i = 1 : nx
        delta_x = sqrtScaledCov(:,i);
        sigma_points(:,i)   = x + delta_x;
        sigma_points(:,i+nx) = x - delta_x;
        weights(i) = 1/(2*kappa + 2*nx);
        weights(i+nx) = 1/(2*kappa + 2*nx);
    end
sigma_points = [x , sigma_points];
weights = [ kappa/(kappa +nx) , weights ];

end


function [y] = measfunc(x)
            h_x = x(1) - x(8);
            h_y = x(2) - x(9);
            y = sqrt(h_x.^2 + h_y.^2);
end


function [P_out] = mycov( weights, sigma_points , sigma_mean )
% Usage: feed weights, datapoints and datamean to get covariance. If
% uniform weights you have unweighted covariance.

sizes_sp = size(sigma_points);

P_out = weights(1)*(sigma_points(:,1) - sigma_mean)*((sigma_points(:,1) - sigma_mean)');
    for j = 2 : sizes_sp(2)
        P_out = P_out + weights(j)*(sigma_points(:,j) - sigma_mean)*((sigma_points(:,j) - sigma_mean)');
    end
end

function [P_out] = myxcov(weights,sigma_points1, sigma_mean1, sigma_points2, sigma_mean2 )
% Usage: feed weights, datapoints and datamean to get xcovariance.

sizes_sp = size(sigma_points1);

P_out = weights(1)*(sigma_points1(:,1) - sigma_mean1)*( (sigma_points2(:,1) - sigma_mean2)' );
    for j = 2 : sizes_sp(2)
        P_out = P_out + weights(j)*(sigma_points1(:,j) - sigma_mean1)*((sigma_points2(:,j) - sigma_mean2)');
    end
end


function [value_mean] = circularMean(w,values)

Y = sum(w.*sin(values), 2);
X = sum(w.*cos(values), 2);
value_mean = atan2(Y_sum,X_sum);


end


function [c,C,omega]=CI(a,A,b,B,H)
%
% function [c,C,omega]=CI(a,A,b,B,H)
%
% This function implements the CI algorithm and fuses two estimates
% (a,A) and (b,B) together to give a new estimate (c,C) and the value
% of omega which minimizes the determinant of C. The observation
% matrix is H.
Ai=inv(A);
Bi=inv(B);
% Work out omega using the matlab constrained minimiser function
% fminbnd().
f=inline('1/det(Ai*omega+H''*Bi*H*(1-omega))', ...
'omega', 'Ai', 'Bi', 'H');
omega=fminbnd(f,0,1,optimset('Display','off'),Ai,Bi,H);
% The unconstrained version of this optimisation is:
% omega = fminsearch(f,0.5,optimset('Display','off'),Ai,Bi,H);
% omega = min(max(omega,0),1);
% New covariance
C=inv(Ai*omega+H'*Bi*H*(1-omega));
% New mean
nu=b-H*a;
W=(1-omega)*C*H'*Bi;
c=a+W*nu;

end