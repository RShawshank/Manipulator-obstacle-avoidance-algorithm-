%% 参数设置
map=im2bw(imread('3.bmp')); 
source=[10 10]; 
goal=[490 490]; 
stepsize = 20;  % RRT的每次生长的步长
threshold = 20; % 节点距离阈值，小于该距离即可认为相同
maxFailedAttempts = 10000;% 最大尝试次数
display = true; % RRT算法的开关

if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display,imshow(map);
    %rectangle('position',[1 1 size(map)-1],'edgecolor','r'); 
end
hold on;
plot(source(1),source(2),'o');
plot(goal(1),goal(2),'o');
tic;  % tic-toc: Functions for Elapsed Time
% RRT算法的主体部分
RRTree = double([source -1]); 
failedAttempts = 0;
counter = 0;
pathFound = false;

while failedAttempts <= maxFailedAttempts  
    %依据概率随机生成点
    if rand < 0.5
        sample = rand(1,2) .* size(map);   % 随机数，随机给出点
    else
        sample = goal; % 给出终点
    end
	% 找到随机生成树距离该随机点的最短的节点
    [A, I] = min( distanceCost(RRTree(:,1:2),sample) ,[],1); 
    closestNode = RRTree(I(1),1:2);
    % 计算生成的点的位置
    thedis = atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(thedis)  cos(thedis)]));
    % 判断拓展树是否可行，即是否在map里面
    if ~checkPath(closestNode(1:2), newPoint, map) 
        failedAttempts = failedAttempts + 1;
        continue;
    end
	%到达终点
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end 
	%判断新节点是否在生成树上
    [A, I2] = min( distanceCost(RRTree(:,1:2),newPoint) ,[],1);
    if distanceCost(newPoint,RRTree(I2(1),1:2)) < threshold, failedAttempts = failedAttempts + 1; continue; end 
	 % 添加节点
    RRTree = [RRTree; newPoint I(1)];
    failedAttempts = 0;
    % 绘制路径
    if display, 
        plot([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)],'LineWidth',1);
        pause(0.05);%调整显示速度
    end  
end

if display && pathFound, line([closestNode(2);goal(2)],[closestNode(1);goal(1)]); counter = counter+1;M(counter) = getframe; end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end

%% 检索路径
path = [goal];
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:2); path];
    prev = RRTree(prev,3);
end

pathLength = 0;
% 计算路径长度
for i=1:length(path)-1, pathLength = pathLength + distanceCost(path(i,1:2),path(i+1,1:2)); end 
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
imshow(map);
%rectangle('position',[1 1 size(map)-1],'edgecolor','k');
plot(path(:,2),path(:,1),'LineWidth',2,'color','r');