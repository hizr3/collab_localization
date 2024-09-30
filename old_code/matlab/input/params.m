nCars_with_gps =  2;
nCars_no_gps = 6 - nCars_with_gps;
nCars = nCars_no_gps + nCars_with_gps;
in_name = 'params';


    x0 = [  -0.5382   -0.9961    0.1766    1.8169    0.0142   -0.4091;
            0.8672   -0.5232    0.7552   -0.1238   -0.0596   -1.2819;
            0.9760   -1.2974   -0.5915   -1.1106   -0.6610   -0.2849;
            0.3374    0.9174    1.8444   -0.6809    0.3060   -0.0648]




% acc_cmd = [ 0,  12,  13; 4,  0,  0];
acc_cmd =0.1*randi(5,2,nCars_with_gps+nCars_no_gps);

del_cmd = [ ...
    0,      0,      0;...
    4,      0,      0;...
    8,      0,      0;...
    12,     0,      0];

del_cmd = 0*ones(1,nCars_with_gps+nCars_no_gps);