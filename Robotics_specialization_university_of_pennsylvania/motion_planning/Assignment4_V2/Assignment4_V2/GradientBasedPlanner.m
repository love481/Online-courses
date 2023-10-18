function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
route =start_coords ;
[dim_x,dim_y]=size(f);
new_RPoint=start_coords;
i=0;
while ~(norm(new_RPoint-end_coords)<2||i==max_its),
    
    %below is the code to find x coordinate and y coordinate respectively.
    t=(1:dim_x)==ceil(new_RPoint(1))&(1:dim_y)'==ceil(new_RPoint(2));
    [x,y] = find(t);
    new_grad=[gx(x,y) gy(x,y)];
    new_RPoint=new_RPoint+new_grad/norm(new_grad);
      route=[route;new_RPoint]; 
      i=i+1;
end
route=double(route);
% *******************************************************************
end
