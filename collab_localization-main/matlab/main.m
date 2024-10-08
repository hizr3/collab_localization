clear; clc; close all
rng(42)

warning('off', 'all');


addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');
%%

par;   
%%% Elapsed time is 1429.983475 seconds. ca 23 minuti

DEBUG_DCL_P = zeros(7,7,nTicks);
%%
tic
for t=1:length(ticks)
    t
    Truth;
    EKF;
    uDCL;
    GSF_MAP;
end
toc
%%
close all
postplot;


