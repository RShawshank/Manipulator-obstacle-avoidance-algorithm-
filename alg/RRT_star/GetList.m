function sorted_list = GetList(newPoint,x_near_list,RRTree)
[n,~]=size(x_near_list);
sortlist=cell(1,3,n);
%���μ���Ǳ�ڸ��ڵ����x_new�ڵ�Ĵ���
for i=1:n
    order=zeros(2);
    order(1,:)=x_near_list(i,1:2);
    order(2,:)=newPoint;
    %order=Steer(x_near_list(i,1:2),newPoint,2);
    cost = AllCost(RRTree,x_near_list(i,:))+distanceCost(newPoint,x_near_list(i,1:2));
    sortlist{1,1,i} = x_near_list(i,:);%��ʾǱ�ڸ��ڵ�
    sortlist{1,2,i} = cost;%��ʾ�ܴ���
    sortlist{1,3,i} = order;%��ʾ����   
end
%�����ܴ��۽�������
[~,~,n]=size(sortlist);
sorted_list = cell(1,3,n);
costTemp=zeros(n,1);
for i=1:n
    costTemp(i)=sortlist{1,2,i};
end
[~,index]=sort(costTemp);
for i=1:n
    for j=1:3
        sorted_list{1,j,i}=sortlist{1,j,index(i)}; 
    end
end

end



