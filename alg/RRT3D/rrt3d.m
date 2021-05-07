%% 绘制障碍物(以球为例，主要是方便计算)
%x0=100; y0=100; z0=100;%球心
circleCenter = [100,100,100;50,50,50;100,40,60;150,100,100;60,130,50];
r=[50;20;20;15;15];%半径
cylinderMatrix = [10,100;40,40];%圆柱体中心坐标
cylinderRMatrix = [10;20];%圆柱体半径
cylinderHMatrix = [200;150];%圆柱体的高
%下面开始绘制
figure(1);
[x,y,z]=sphere;
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end
for i = 1:size(cylinderMatrix)   %绘制圆柱体障碍物
    [x,y,z] = cylinder(cylinderRMatrix(i));%创建以(0,0)为圆心，高度为[0,1]，半径为R的圆柱
    mesh(x + cylinderMatrix(i,1),y + cylinderMatrix(i,2),z*cylinderHMatrix(i));
    hold on;
end

axis equal
%%将横轴纵轴的定标系数设成相同值
%% 参数
source=[10 10 10];
goal=[150 150 150];
stepsize = 10;
threshold = 10;
maxFailedAttempts = 10000;
display = true;
searchSize = [250 250 250];      %探索空间六面体
%% 绘制起点和终点
hold on;%保持图形保持功能
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
tic;  % tic-toc: Functions for Elapsed Time
RRTree = double([source -1]);
failedAttempts = 0;
pathFound = false;

%% 循环
while failedAttempts <= maxFailedAttempts  
    %%随机点生成
    if rand < 0.5
        sample = rand(1,3) .* searchSize;   
    else
        sample = goal; 
    end
    %%随机生成树中距离随机点上最近的一个点
    [A, I] = min( distanceCost3(RRTree(:,1:3),sample) ,[],1); 
    closestNode = RRTree(I(1),1:3);
    %% 获得随机生成树的生长方向
    movingVec = [sample(1)-closestNode(1),sample(2)-closestNode(2),sample(3)-closestNode(3)];
    movingVec = movingVec/sqrt(sum(movingVec.^2));  %单位化
    newPoint = closestNode + stepsize * movingVec;
    if ~checkPath3(closestNode, newPoint, circleCenter,r,cylinderMatrix,cylinderRMatrix,cylinderHMatrix) % 判断新的点是否可达
        failedAttempts = failedAttempts + 1;
        continue;
    end
    
    if distanceCost3(newPoint,goal) < threshold, pathFound = true; break; end % 到达终点
    [A, I2] = min( distanceCost3(RRTree(:,1:3),newPoint) ,[],1); 
    if distanceCost3(newPoint,RRTree(I2(1),1:3)) < threshold, failedAttempts = failedAttempts + 1; continue; end 
    
    RRTree = [RRTree; newPoint I(1)]; % 添加新的点到随机生成树中
    failedAttempts = 0;
    if display, plot3([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'LineWidth',1);pause(0.05);end
    pause(0.05);
end

if display && pathFound, plot3([closestNode(1);goal(1)],[closestNode(2);goal(2)],[closestNode(3);goal(3)]); end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end
%% 从随机生成树的终点开始回溯找到最短路径
path = goal;
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:3); path];
    prev = RRTree(prev,4);
end

pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + distanceCost3(path(i,1:3),path(i+1,1:3)); end % 计算路径长度
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
figure(2)
[x,y,z]=sphere;
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end
for i = 1:size(cylinderMatrix)  %绘制圆柱体障碍物
    [x,y,z] = cylinder(cylinderRMatrix(i));%创建以(0,0)为圆心，高度为[0,1]，半径为R的圆柱
    mesh(x + cylinderMatrix(i,1),y + cylinderMatrix(i,2),z*cylinderHMatrix(i));
    hold on;
end


axis equal
hold on;
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
plot3(path(:,1),path(:,2),path(:,3),'LineWidth',2,'color','r');
