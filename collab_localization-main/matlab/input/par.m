nSims = 10^3;
runtime =30 ;
SF = -0;
sve = false;

sensors = [true, true, true];






nCars = 6;
in_name = 'par';

x0 = randn(2,nCars) ; % random xy position
x0 = [x0 ; 2*rand( 1,nCars) - 1] ; % orientation
x0 = [x0 ; randi(10, 1,nCars)]; % random velocity





%%%%%%%%%%%%%%%%%%

rate = 150;
dt = 1/rate;
ticks = 0:runtime*rate-1;

rate_imu = 150;
rate_mdl =  30;
rate_gps =  10;
rate_uwb =   3;

wb = 4;
tw = 2;

% Standard deviations

% 
% gps_per = 0       * 10^SF;
% gps_her = 0     * 10^SF;
% gps_ver = 0      * 10^SF;
% enc_err = 0      * 10^SF;
% str_err = 0      * 10^SF;
% uwb_err = 0     * 10^SF;
% imu_acc_err = 0   * 10^SF;
% imu_gyr_err = 0   * 10^SF;
% imu_mag_err = 0   * 10^SF;
% sa_err = 0  * 10^SF;

gps_per = 1.2      * 10^SF;
gps_her = 0.035     * 10^SF;
gps_ver = 0.05      * 10^SF;
enc_err = 0.05      * 10^SF;
str_err = 0.05      * 10^SF;
uwb_err = 0.9      * 10^SF;
imu_acc_err = 0.1   * 10^SF;
imu_gyr_err = 0.1   * 10^SF;
imu_mag_err = 0.1   * 10^SF;
sa_err = 0.290866   * 10^SF;

% gps_per = 1       * 10^SF;
% gps_her = 1     * 10^SF;
% gps_ver = 1     * 10^SF;
% enc_err = 1      * 10^SF;
% str_err = 1      * 10^SF;
% uwb_err = 1      * 10^SF;
% imu_acc_err = 1  * 10^SF;
% imu_gyr_err = 1   * 10^SF;
% imu_mag_err = 1   * 10^SF;
% sa_err = 1        * 10^SF;

nTicks = length(ticks)


  
    % Very hard trajectory
    % Acc input
     acc = 0.0000001*rand(nTicks , nCars) ;

    phi_dot = @(y,u) (-tan(y)*tan(y)*u)/(4+4*tan(y)*tan(y)) ;
    % Steering angle reference
     del_ref = zeros(nTicks, nCars);
     del = zeros(nTicks, nCars);
     del_err = 0;
     del_err_int = 0;
     for i = 2 : nTicks
      % Compute reference input
       del_ref(i,:) = sat( cos(i*dt*2*pi*3), 0.5) ; % 0.5 rad is approx 28 deg
      % Compute driving input for vehicle 1
       del_err_old = del_err;
       del_err = del_ref(i,1) - del(i,1);
       del_err_int =  del_err_int + dt*del_err;
       del_err_dev = (del_err - del_err_old)/dt;

       if abs(del_err_int) > 256
           del_err_int = 0;
       end

           Ki = 1;
           Kp = 1;
           Kd = 0.05;

       u_del = Ki*del_err_int + Kp * del_err + Kd * del_err_dev;
       del(i,1) = del(i-1,1) + dt*phi_dot(del(i-1,1) , u_del);
       % Replicate input
       del(i,:) = del(i,1);
     end  
% Stop gap until i figure it out 
del = del_ref;






function [x_out] = sat (x_in , satvalue)
    if satvalue < 0 
        disp("Satvalue should be positive")
    end

    if x_in > satvalue
        x_out = satvalue;
    elseif x_in < -satvalue
        x_out = -satvalue;
    else
        x_out = x_in;
    end
end