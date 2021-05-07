function pathLength = AllCost(RRTree,x_near)
%计算从根节点到x_near的路径长度
path=[x_near(1:2)];
prev=x_near(3);%父节点
pathLength=0;
if prev<2
    pathLength=pathLength+distanceCost(x_near,RRTree(1,1:2));
else
    while prev>0
        path=[RRTree(prev,1:2);path];
        prev=RRTree(prev,3);
    end
    for i=1:length(path)-1
        pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2));
    end

end