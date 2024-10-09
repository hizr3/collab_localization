clear; clc; 
warning('all','off');
addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');
rng(42)
nSims = 10^3;
runtime = 10;
SF = -0;
sve = false;

sensors = [true, true, true];

par;
preprocessor;
nTicks
tic 
for t=1:nTicks
    t
    Truth;
    EKF;
    GSF_MAP;
    DCL;
end
toc

postprocessor;
postplot;
