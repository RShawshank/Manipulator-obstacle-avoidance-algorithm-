function x_min = ChooseBestParent(Ls,map)
x_min=[];
[~,~,n]=size(Ls);
for i=1:n
    temp=Ls{1,3,i};%��ȡ����
    if checkPath(double(int32(temp(1,:))),double(int32(temp(end,:))),map) %��������ײ
        x_min=Ls{1,1,i};
        break;
    end
end

end

