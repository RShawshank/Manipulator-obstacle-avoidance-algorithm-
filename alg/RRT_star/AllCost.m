function pathLength = AllCost(RRTree,x_near)
%����Ӹ��ڵ㵽x_near��·������
path=[x_near(1:2)];
prev=x_near(3);%���ڵ�
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