clear; clc; close all
rng(42)

warning('off', 'all');


addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');
%%

par;   
%%
tic
for t=1:length(ticks)
    t;
    Truth;
    EKF;
    DCL;
    GSF_MAP;
end
toc
%%
close all
postprocessor;
postplot;


