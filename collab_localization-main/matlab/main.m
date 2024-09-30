clear; clc; close all
rng(42)

addpath('algorithms');
addpath('input');
addpath('output');
addpath('src');
%%
nSims = 10^3;
runtime = 10;
SF = -0;
sve = false;

sensors = [true, true, true];

par;
preprocessor;



 %%%% Comment below for straight line trajectory 


for i = 1 : length(ticks)
    if i <= floor(length(ticks)*0.5)
        del(i,:) = 5*cos(i*dt*2*pi*0.3);
    else
        del(i,:) = -5*cos(i*dt*2*pi*0.3);
    end
end


tic 
for t=1:length(ticks)
    Truth;
    EKF;
    DCL;
end
toc
%%
postprocessor;
postplot;
