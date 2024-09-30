% Truth Model
%
% x(1) : x position
% x(2) : y position
% x(3) : orientation
% x(4) : velocity

%%% x(5) : steering angle rate
%



if t == 1
    x_truth = zeros(4,nCars,nTicks);
    x_truth(:,:,1) = x0;
else
    
    x_truth(1:4,:,t) = x_truth(1:4,:,t-1) ...
        + [-dt * x_truth(4,:,t-1) .* sin(x_truth(3,:,t-1));...
            dt * x_truth(4,:,t-1) .* cos(x_truth(3,:,t-1));...
            dt * x_truth(4,:,t-1) .* tan(x_truth(5,:,t-1))/wb;...
            dt * acc(t,:)];
end
