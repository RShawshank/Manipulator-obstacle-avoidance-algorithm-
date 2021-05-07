function [RRTree1,pathFound,extendFail]=rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map)
%如果找到路径，则返回连接两棵树的新节点
pathFound=[]; 
failedAttempts=0;
while failedAttempts<=maxFailedAttempts
    %随机点生成
    if rand < 0.5, 
        sample=rand(1,2) .* size(map); 
    else
        sample=goal; 
    end
    [A, I]=min( distanceCost(RRTree1(:,1:2),sample) ,[],1); 
    closestNode = RRTree1(I(1),:);%随机生成树中距离随机点上最近的一个点
    thedis=atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2))); 
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(thedis)  cos(thedis)]));%得到新生成点
    if ~checkPath(closestNode(1:2), newPoint, map) %判断是否在地图上
        failedAttempts=failedAttempts+1;
        continue;
    end
    % 对第二棵随机生成树进行同样的操作
    [A, I2]=min( distanceCost(RRTree2(:,1:2),newPoint) ,[],1); 
    %如果两棵树随机生成树连接在一起
    if distanceCost(RRTree2(I2(1),1:2),newPoint)<disTh, 
        pathFound=[newPoint I(1) I2(1)];extendFail=false;break; 
    end 
    [A, I3]=min( distanceCost(RRTree1(:,1:2),newPoint) ,[],1); 
    if distanceCost(newPoint,RRTree1(I3(1),1:2))<disTh, failedAttempts=failedAttempts+1;continue; end 
    RRTree1=[RRTree1;newPoint I(1)];extendFail=false;break;
end