%Repulsion Calculation
function [Yrerxx,Yreryy]=compute_repulsion(X,Xsum,m,angle_re,n,Po)%The input parameter is the current coordinates, Xsum is the coordinate vector of the target and the obstacle, the gain constant, the obstacle, the angle of the target direction
  for i=1:n
    Rrei(i)=(X(1)-Xsum(i+1,1))^2+(X(2)-Xsum(i+1,2))^2;%Squared distance between waypoints and obstacles
    rre(i)=sqrt(Rrei(i));%The distance between the waypoint and the obstacle is stored in the array rrei
    if rre(i)>Po%If the distance between each obstacle and the path is greater than the obstacle influence distance, the repulsion order is 0
          Yrerx(i)=0
          Yrery(i)=0
    else
       Yrer(i)=m*(1/rre(i)-1/Po)^2*1/(rre(i)^2)%Decomposed Fre1 vector
       Yrerx(i)=Yrer(i)*cos(angle_re(i))%angle_re(i)=Y(i+1)
       Yrery(i)=Yrer(i)*sin(angle_re(i))
    end%Determine whether the distance is within the influence range of the obstacle
end
   Yrerxx=sum(Yrerx)%Components of superposition repulsion
   Yreryy=sum(Yrery)