function [RRTree1,pathFound,extendFail,closestNode,newPoint]=rrtExtend3(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,threshold,circleCenter,r,searchSize)
pathFound=[]; %if path found, returns new node connecting the two trees, index of the nodes in the two trees connected
failedAttempts=0;
while failedAttempts<=maxFailedAttempts
    if rand < 0.5
        sample=rand(1,3) .* searchSize; % random sample
    else
        sample=goal; % sample taken as goal to bias tree generation to goal
    end
    [A, I] = min( distanceCost3(RRTree1(:,1:3),sample) ,[],1);  % find closest as per the function in the metric node to the sample
    closestNode = RRTree1(I(1),1:3);
    movingVec = [sample(1)-closestNode(1),sample(2)-closestNode(2),sample(3)-closestNode(3)];
    movingVec = movingVec/sqrt(sum(movingVec.^2));  %µ¥Î»»¯
    newPoint = closestNode + stepsize * movingVec;
    if ~checkPath3(closestNode, newPoint, circleCenter,r) % if extension of closest node in tree to the new point is feasible
        failedAttempts = failedAttempts + 1;
        continue;
    end
    [A, I2] = min( distanceCost3(RRTree2(:,1:3),newPoint) ,[],1); % find closest in the second tree
    if distanceCost3(RRTree2(I2(1),1:3),newPoint)<threshold % if both trees are connected
        pathFound=[newPoint I(1) I2(1)];extendFail=false;break; 
    end 
    [A, I3]=min( distanceCost3(RRTree1(:,1:3),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost3(newPoint,RRTree1(I3(1),1:3))<threshold, failedAttempts=failedAttempts+1;continue; end 
    RRTree1=[RRTree1;newPoint I(1)];extendFail=false;break; % add node
end