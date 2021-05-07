function x_min = ChooseBestParent(Ls,map)
x_min=[];
[~,~,n]=size(Ls);
for i=1:n
    temp=Ls{1,3,i};%获取连线
    if checkPath(double(int32(temp(1,:))),double(int32(temp(end,:))),map) %连线无碰撞
        x_min=Ls{1,1,i};
        break;
    end
end

end

