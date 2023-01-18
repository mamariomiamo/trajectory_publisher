% equation 8 and 9 of 
% https://mathworld.wolfram.com/Lemniscate.html
d = 1; % each globe has diameter of d
x = [];
y=[];
for theta=0:0.01:2*pi
    x = [x, d*cos(theta) / (1+(sin(theta)^2))];
    y = [y, d*cos(theta)*sin(theta) / (1+(sin(theta)^2))];
end

vx = -(a*omega*cos(omega*x)*(sin(omega*x)^2-1))/(sin(omega*x)^2+1)^2;
vy = -(a*omega*(sin(omega*x)^4+(cos(omega*x)^2+1)*sin(omega*x)^2-cos(omega*x)^2))/(sin(omega*x)^2+1)^2;
ax = (a*omega^2*sin(omega*x)*(sin(omega*x)^4+2*cos(omega*x)^2*sin(omega*x)^2-6*cos(omega*x)^2-1))/(sin(omega*x)^2+1)^3;
ay = (2*a*omega^2*cos(omega*x)*sin(omega*x)*(sin(omega*x)^4+(cos(omega*x)^2-1)*sin(omega*x)^2-3*cos(omega*x)^2-2))/(sin(omega*x)^2+1)^3;
plot(x,y);