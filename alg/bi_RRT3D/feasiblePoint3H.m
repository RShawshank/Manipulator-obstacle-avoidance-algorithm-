%% feasiblePoint3H.m
%% 判断点是否在圆柱体内
function feasible=feasiblePoint3H(point,cylinderMatrix,cylinderRMatrix,cylinderHMatrix)
feasible=true;
for i = 1:size(cylinderMatrix)
    if(abs(point(1)-cylinderMatrix(i,1))<cylinderRMatrix(i)&&abs(point(2)-cylinderMatrix(i,2))<cylinderRMatrix(i)&&point(3)<cylinderHMatrix(i))
        feasible = false;break;
    end
end
end
