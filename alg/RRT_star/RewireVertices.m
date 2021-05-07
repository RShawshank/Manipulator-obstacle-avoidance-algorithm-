function RRTree_new=RewireVertices(x_min,x_new,Ls,RRTree,map)
[~,~,n]=size(Ls);
for i=1:n
    x_near=Ls{1,1,i};
    dis=AllCost(RRTree,x_min);
    if dis+distanceCost(x_min(1:2),x_new)+distanceCost(x_new,x_near(1:2))<AllCost(RRTree,x_near)
        if checkPath(x_near(1:2),x_new,map)
            RRTree(x_near(4),3)=length(RRTree);
        end
    end
end
RRTree_new=RRTree;
end

