function RRTree_new=RewireVertices3(x_min,x_new,Ls,RRTree,circleCenter,r)
[~,~,n]=size(Ls);
for i=1:n
    x_near=Ls{1,1,i};
    dis=AllCost3(RRTree,x_min);
    if dis+distanceCost3(x_min(1:3),x_new)+distanceCost3(x_new,x_near(1:3))<AllCost3(RRTree,x_near)
        if checkPath3(x_near(1:3),x_new,circleCenter,r)
            RRTree(x_near(5),4)=length(RRTree);
        end
    end
end
RRTree_new=RRTree;
end

