clear
clc
Xo=[0 0];%starting position
k=15;%Calculate the gain factor needed for gravity
m=8;%Calculating the gain coefficient of repulsion
Po=5;% obstacle to robot distance (No repulsion if robot is far from this)
n=32;%Number of obstacles
l=0.1;%step size
J=600;%loop iterations
%If the expected goal cannot be achieved, it may also be related to the inappropriate setting of the initial gain coefficient and Po.
%end
%Give obstacle and goal information
Xsum=[20 20;8 4;9 4;10 4;11 4;12 4;8 8;9 8;10 8;11 8;12 8;8 5;8 6;8 7;12 5;12 6;12 7;8 12;9 12;10 12;11 12;12 12;8 16;9 16;10 16;11 16;12 16;8 13;8 14;8 15;12 13;12 14;12 15];%这个向量是(n+1)*2维，其中[10 10]是目标位置，剩下的都是障碍的位置。
Xj=Xo;%j=1 cycle initial, assign the starting coordinates of the car to Xj
%***************main loop begins******************
for j=1:J%cycle starts
Goal(j,1)=Xj(1);%Goal is to save the coordinates of each point that the car has passed. Just start by putting the starting point into this vector
Goal(j,2)=Xj(2);
%Call the calculation angle module
Theta=compute_angle(Xj,Xsum,n);%Theta is the calculated angle between the car and the obstacle, and the target and the X-axis. The angle is uniformly specified as the counterclockwise direction, which can be calculated with this module.
%Call the calculation gravity module
Angle=Theta(1);%Theta（1）is the angle between the car and the target, and the target is the gravitational force on the car.
angle_at=Theta(1);%For subsequent calculations, the component of the repulsive force in the direction of gravity is assigned to angle_at
[Fatx,Faty]=compute_Attract(Xj,Xsum,k,Angle);%Calculate the two component values ​​of the target's gravitational force on the car in the x and y directions.
for i=1:n
angle_re(i)=Theta(i+1);%The angle used to calculate the repulsive force is a vector, because there are n obstacles, there are n angles
end

%Call the calculation repulsion module
[Yrerxx,Yreryy]=compute_repulsion(Xj,Xsum,m,angle_re,n,Po);%Calculate the component array of the repulsive force in the x, y direction.
% Calculate the resultant force and direction. There is a problem. It should be a number. The size of the resultant force should be a unique number in each j cycle, not an array. All the components of the repulsive force should be added together, and all the components of the attractive force should be added together.
Fsumyj=Faty+Yreryy;%Resultant force in the y direction
Fsumxj=Fatx+Yrerxx;%The resultant force in the x direction
Position_angle(j)=atan(Fsumyj/Fsumxj);%The angle vector between the resultant force and the x-axis direction
%Calculate the next step of the car
if Fsumyj < 0 && Fsumxj <0
   Xnext(1)=Xj(1)-l*cos(Position_angle(j));
   Xnext(2)=Xj(2)-l*sin(Position_angle(j));
else
   Xnext(1)=Xj(1)+l*cos(Position_angle(j));
   Xnext(2)=Xj(2)+l*sin(Position_angle(j));
end
%Save every position of the car in a vector
Xj=Xnext;
%judge
if ((Xj(1)-Xsum(1,1))>0)&((Xj(2)-Xsum(1,2))>0)%Should it count as arrival when it's exactly equal, or just close enough? Now program in exact equal time.。
%K=j%Record the number of iterations to reach the goal.
break;
end
end
K=j;
Goal(K,1)=Xsum(1,1);%Assign the last point of the path vector as the target
Goal(K,2)=Xsum(1,2);

%***********************************Draw obstacles, starting points, goals, waypoints*************************
%draw the path
X=Goal(:,1);
Y=Goal(:,2);
%The path vector Goal is a two-dimensional array, and X and Y are the collection of x and y elements of the array respectively, which are two one-dimensional arrays.
x=[8 9 10 11 12 8 9 10 11 12 8 8 8 12 12 12 8 9 10 11 12 8 9 10 11 12 8 8 8 12 12 12 ];%x-coordinate of the obstacle
y=[4 4 4 4 4 8 8 8 8 8 5 6 7 5 6 7 12 12 12 12 12 16 16 16 16 16 13 14 15 13 14 15];
plot(x,y,'o',Xsum(1,1),Xsum(1,2),'v',0,0,'ms',X,Y,'.r');


