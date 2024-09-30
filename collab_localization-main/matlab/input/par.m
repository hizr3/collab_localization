nCars = 2;
in_name = 'par';

if easy
    x0 = zeros(3,nCars);

else
    x0 = randn(2,nCars) ; % random xy position
    x0 = [x0 ; 2*rand( 1,nCars) - 1] ; % random orientation
    x0 = [x0 ; rand( 1,nCars)]; % random velocity

end





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

gps_per = 1.2       * 10^SF;
gps_her = 0.035     * 10^SF;
gps_ver = 0.05      * 10^SF;
enc_err = 0.05      * 10^SF;
str_err = 0.05      * 10^SF;
uwb_err = 0.30      * 10^SF;
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


if easy
        % Straight line default
        acc = ones(nTicks, nCars);
        del = zeros(nTicks, nCars);
        size(acc)
        size(del)
else
    % Very hard trajectory
    acc = randi([0 , 70] ,nTicks,nCars)+normrnd(0,1,nTicks,nCars);
    
            for i = 1 : nTicks
                if i <= nTicks
                    del(i,:) = cos(i*dt*2*pi*0.3);
                else
                    del(i,:) = cos(i*dt*2*pi*0.3);
                end
            end
    size(acc)
    size(del)

end