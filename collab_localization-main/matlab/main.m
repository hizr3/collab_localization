clear; clc; close all
rng(42)

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');
%%
nSims = 10^3;
runtime = 1;
SF = -0;
sve = false;

sensors = [true, true, true];
easy = 0;

par;   

tic
for t=1:length(ticks)
    t
    Truth;
    EKF;
    DCL;
end
toc
%%
close all
postprocessor;
postplot;


