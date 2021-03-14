clear; clc; 

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');

nSims = 10^4;
nThreads = 10;
EKF_rmse = zeros(3, nSims/nThreads, nThreads);
DCL_rmse = zeros(3, nSims/nThreads, nThreads);
CKF_rmse = zeros(3, nSims/nThreads, nThreads);

sensors = [true, true, true];

tic
parfor i = 1:nThreads
    out = fmain(nSims, nThreads, sensors);
    
    EKF_rmse(:,:,i) = out.EKF;
    DCL_rmse(:,:,i) = out.DCL;
    CKF_rmse(:,:,i) = out.CKF;
    
end
toc

plt = true; 
postplot;

function [out] = fmain(nS, nT, sensors)

    addpath('algorithms');
    addpath('input');
    addpath('output');
    addpath('src');

    nSims = nS / nT;
    runtime = 5;
    SF = -0;
    plt = false;
    sve = false;

    par;
    preprocessor;

    for t=1:length(ticks)
        Truth;
        EKF;
        DCL;
        CKF;
    end

    postprocessor;

    out.EKF = EKF_rmse;
    out.DCL = DCL_rmse;
    out.CKF = CKF_rmse;

end

