

nCars = 10;
in_name = 'par';

x0 = 100*randn(2,nCars) ; % random xy position
x0 = [x0 ; zeros(1,nCars)] ; % orientation
x0 = [x0 ; randi(10, 1,nCars)]; % random velocity

