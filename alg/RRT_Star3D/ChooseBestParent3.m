function x_min = ChooseBestParent3(Ls,circleCenter,r)
x_min=[];
[~,~,n]=size(Ls);
for i=1:n
    temp=Ls{1,3,i};%��ȡ����
    if checkPath3(double(int32(temp(1,:))),double(int32(temp(end,:))),circleCenter,r) %��������ײ
        x_min=Ls{1,1,i};
        break;
    end
end
end


