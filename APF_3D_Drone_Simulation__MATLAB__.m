 close all; %close everything
 clf; %clears current figure window
 xmax = 10;
 ymax = xmax;
 zmax = xmax;
 n_obs = 20;
 robotPose = [0,0,0];
 Obs_Krep = rand(n_obs,1)* (0.6-0.4) + 0.4;
 shapeOfObs = randi([3,10],1,n_obs);
 obs = rand(n_obs,3)*28;
 colorObs = [0.3,0.5,0.8];
 rObs = rand(n_obs,1)*(2) ;
 hold on
 for i = 1:n_obs
     plot3D_shape(obs(i,:), rObs(i), shapeOfObs(i), colorObs)
 end
 hold off
 %--------------------------------------------------------%

 %Moving Obstacles
n_DynamicObs = 2;
DynamicObs_Krep = rand(n_DynamicObs,1)* (0.7-0.4) + 0.4;
DynamicObs = rand(n_DynamicObs,3)*28;
rDynamicObs = rand(n_DynamicObs, 1)*2;
colorDynamicObs = [0.8,0.5,0.8];
hold on
 for i = 1:n_DynamicObs
     plot3D_shape(DynamicObs(i,:), rDynamicObs(i), 1000, colorDynamicObs)
 end
 hold off
 %Goal 1 near to BookShelf 4
 Goal_Katt = 0.1;
 nGoal = 3;
 Goal = rand(nGoal,3)*28;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create Repulsive and attractive potential field forces %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i = 1:xmax;
j = 1:ymax;
k = 1:zmax;
[I, J, K] = meshgrid(i, j, k);
ACCUMULATOR = []; %empty matrix for z-axis plot

for i=1:xmax 
    for j=1:ymax 
         for k=1:zmax   
             pos = [i j k]; % scaled to fit 300by170 matrix
             accumulator = 0;    %accumulator
             %Obstacles with exponential curve while goal is by vector
             for v = 1:n_obs
                 accumulator = accumulator + exp(-norm(pos-obs(v,:))/Obs_Krep(v));
             end
             for v = 1:n_DynamicObs
                 accumulator = accumulator + exp(-norm(pos-DynamicObs(v,:))/DynamicObs_Krep(v));
             end
             for v = 1:nGoal
                 accumulator = accumulator + Goal_Katt*(norm(Goal(v,:)-pos));
             end
             ACCUMULATOR(i, j, k) = accumulator; %store in matrix
         end
    end
end
[px,py, pz] = gradient(ACCUMULATOR);
quiver3(I,J,K,-px,-py,-pz,'r')
figure
for i = zmax:-1:1
surf(ACCUMULATOR(:,:,i),'FaceAlpha',0.5,'edgeAlpha',0)
pause(0.01)
hold on
end
