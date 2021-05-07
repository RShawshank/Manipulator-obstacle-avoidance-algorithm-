function [RRTree1,pathFound,extendFail]=rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map)
%����ҵ�·�����򷵻��������������½ڵ�
pathFound=[]; 
failedAttempts=0;
while failedAttempts<=maxFailedAttempts
    %���������
    if rand < 0.5, 
        sample=rand(1,2) .* size(map); 
    else
        sample=goal; 
    end
    [A, I]=min( distanceCost(RRTree1(:,1:2),sample) ,[],1); 
    closestNode = RRTree1(I(1),:);%����������о���������������һ����
    thedis=atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2))); 
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(thedis)  cos(thedis)]));%�õ������ɵ�
    if ~checkPath(closestNode(1:2), newPoint, map) %�ж��Ƿ��ڵ�ͼ��
        failedAttempts=failedAttempts+1;
        continue;
    end
    % �Եڶ����������������ͬ���Ĳ���
    [A, I2]=min( distanceCost(RRTree2(:,1:2),newPoint) ,[],1); 
    %������������������������һ��
    if distanceCost(RRTree2(I2(1),1:2),newPoint)<disTh, 
        pathFound=[newPoint I(1) I2(1)];extendFail=false;break; 
    end 
    [A, I3]=min( distanceCost(RRTree1(:,1:2),newPoint) ,[],1); 
    if distanceCost(newPoint,RRTree1(I3(1),1:2))<disTh, failedAttempts=failedAttempts+1;continue; end 
    RRTree1=[RRTree1;newPoint I(1)];extendFail=false;break;
end