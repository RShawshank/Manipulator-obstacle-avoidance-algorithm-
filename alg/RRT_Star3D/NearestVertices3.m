function x_near_list = NearestVertices3(x_new,RRTree)
% ������RRTree�о�����������r���ڵĵ�
% ��¼�����꣬�����е����꣬���ڵ㡣
[num,~]=size(RRTree);
gama =500;
r = gama*sqrt(log(num)/num);
dis = distanceCost3(RRTree(:,1:3),x_new);
index = find(dis<=r);
[n,~] = size(index);
if n==0
    x_near_list=[];
else
    %zeros(m,n)������m��nȫ����
    x_near_list=zeros(n,5);
    for i=1:n
        x_near_list(i,:)=[RRTree(index(i),1:4),index(i)];
    end
end
end

