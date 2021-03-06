map=im2bw(imread('1.png'));%将灰度图像转换为二值图像
goal=[400 400];
stepsize=20; 
disTh=20; 
maxFailedAttempts = 10000;
display=true;

if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);
    %rectangle('position',[1 1 size(map)-1],'edgecolor','k'); %画边框
end
hold on;
plot(source(1),source(2),'o');
plot(goal(1),goal(2),'o');
tic;
% 第一条随机生成树从起点出发
RRTree1=double([source -1]);
% 第二条随机生成树从终点出发
RRTree2=double([goal -1]); 
% 拓展失败标志——true
tree1ExpansionFail=false; 
tree2ExpansionFail=false;
while ~tree1ExpansionFail || ~tree2ExpansionFail 
    % 从起点到终点进行拓展
    if ~tree1ExpansionFail 
        [RRTree1,pathFound,tree1ExpansionFail]=rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map); 
        if ~tree1ExpansionFail && isempty(pathFound) && display
            plot([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'LineWidth',1);
        pause(0.05);
        end
    end
    % 从终点到起点进行拓展
    if ~tree2ExpansionFail 
        [RRTree2,pathFound,tree2ExpansionFail]=rrtExtend(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map);
        if ~isempty(pathFound), pathFound(3:4)=pathFound(4:-1:3); end 
        if ~tree2ExpansionFail && isempty(pathFound) && display
            plot([RRTree2(end,2);RRTree2(RRTree2(end,3),2)],[RRTree2(end,1);RRTree2(RRTree2(end,3),1)],'LineWidth',1);
           pause(0.05);
        end
    end
    % 路径可达
    if ~isempty(pathFound) 
         if display
             plot([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'LineWidth',1);
           pause(0.05);
         end
        % 获得完整路径
        path=[pathFound(1,1:2)]; 
        prev=pathFound(1,3); 
        % 添加第一颗树的节点
        while prev>0
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); 
        % 添加第二棵树的节点
        while prev>0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        break;
    end
end
if display 
    disp('click/press any key');
    waitforbuttonpress; 
end
if size(pathFound,1)<=0, error('no path found. maximum attempts reached'); end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 
imshow(map);
%rectangle('position',[1 1 size(map)-1],'edgecolor','k');%画边框
plot(path(:,2),path(:,1),'LineWidth',2,'color','r');
