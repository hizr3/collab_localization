clc; close all ; clear;  

rng(42)
addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');
%%

nSims = 10^3;
runtime = 5;
SF = -0;
sve = false;

sensors = [true, true, true];

params;
preprocessor;

%%
for i = 1 : length(ticks)
    if i <= floor(length(ticks)*0.5)
    del(i,:) = 5*cos(i*dt*2*pi*0.3);
    else
    del(i,:) = -5*cos(i*dt*2*pi*0.3);
    end
end
%%

length(ticks)
tic 
for t=1:length(ticks)
    t
    Truth;
    EKF;
    DCL;
end
toc
%%

postprocessor;
postplot;
