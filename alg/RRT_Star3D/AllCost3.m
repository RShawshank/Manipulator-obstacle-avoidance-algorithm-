function pathLength = AllCost3(RRTree,x_near)
%计算从根节点到x_near的路径长度
path=[x_near(1:3)];
prev=x_near(4);%父节点
pathLength=0;
if prev<2
    pathLength=pathLength+distanceCost3(x_near,RRTree(1,1:3));
else
    while prev>0
        path=[RRTree(prev,1:3);path];
        prev=RRTree(prev,4);
    end
    for i=1:length(path)-1
        pathLength=pathLength+distanceCost3(path(i,1:3),path(i+1,1:3));
    end

end