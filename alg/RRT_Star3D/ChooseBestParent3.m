function x_min = ChooseBestParent3(Ls,circleCenter,r)
x_min=[];
[~,~,n]=size(Ls);
for i=1:n
    temp=Ls{1,3,i};%获取连线
    if checkPath3(double(int32(temp(1,:))),double(int32(temp(end,:))),circleCenter,r) %连线无碰撞
        x_min=Ls{1,1,i};
        break;
    end
end
end


