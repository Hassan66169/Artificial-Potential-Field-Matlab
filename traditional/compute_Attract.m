function [Yatx,Yaty]=compute_Attract(X,Xsum,k,angle)%Input parameters are current coordinates, target coordinates, gain constant, component and force angle
%Take the temporary point on the path as the Xgoal at each moment
R=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%Squared distance between waypoint and target
r=sqrt(R);%Distance from waypoint to target
Yatx=k*r*cos(angle);
Yaty=k*r*sin(angle);
end