clear; clc; close all
rng(42)

warning('off', 'all');


addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');
%%

par;   
%%%%% Elapsed time is 1429.983475 seconds, 23 minuti

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
postplot;


