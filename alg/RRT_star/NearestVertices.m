function x_near_list = NearestVertices(x_new,RRTree)
% NearestVertices 
% 返回树RRTree中距离采样点距离r以内的点
% 记录其坐标，在树中的坐标，父节点。
[num,~]=size(RRTree);
gama =500;
r = gama*sqrt(log(num)/num);
dis = distanceCost(RRTree(:,1:2),x_new);
index = find(dis<=r);
[n,~] = size(index);
if n==0
    x_near_list=[];
else
    %zeros(m,n)：生成m×n全零阵。
    x_near_list=zeros(n,4);
    for i=1:n
        x_near_list(i,:)=[RRTree(index(i),1:3),index(i)];
    end
end
end

