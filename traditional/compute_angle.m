function Y=compute_angle(X,Xsum,n)%Y is the angle vector of attraction, repulsion and x-axis, X is the coordinates of the starting point, Xsum is the coordinate vector of the target and the obstacle, which is a (n+1)*2 matrix
for i=1:n+1%n is the number of obstacles
deltaXi=Xsum(i,1)-X(1)
deltaYi=Xsum(i,2)-X(2)
ri=sqrt(deltaXi^2+deltaYi^2)
if deltaXi>0
theta=asin(deltaXi/ri)
else
theta=pi-asin(deltaXi/ri)
end
if i==1%said it was the target
angle=theta
else
angle=pi+theta
end 
Y(i)=angle%Save each angle in the Y vector, the first element is the angle with the target, followed by the angle with the obstacle
end
