%% ��������
map=im2bw(imread('3.bmp')); 
source=[10 10]; 
goal=[490 490]; 
stepsize = 20;  % RRT��ÿ�������Ĳ���
threshold = 20; % �ڵ������ֵ��С�ڸþ��뼴����Ϊ��ͬ
maxFailedAttempts = 10000;% ����Դ���
display = true; % RRT�㷨�Ŀ���

if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display,imshow(map);
    %rectangle('position',[1 1 size(map)-1],'edgecolor','r'); 
end
hold on;
plot(source(1),source(2),'o');
plot(goal(1),goal(2),'o');
tic;  % tic-toc: Functions for Elapsed Time
% RRT�㷨�����岿��
RRTree = double([source -1]); 
failedAttempts = 0;
counter = 0;
pathFound = false;

while failedAttempts <= maxFailedAttempts  
    %���ݸ���������ɵ�
    if rand < 0.5
        sample = rand(1,2) .* size(map);   % ����������������
    else
        sample = goal; % �����յ�
    end
	% �ҵ�����������������������̵Ľڵ�
    [A, I] = min( distanceCost(RRTree(:,1:2),sample) ,[],1); 
    closestNode = RRTree(I(1),1:2);
    % �������ɵĵ��λ��
    thedis = atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(thedis)  cos(thedis)]));
    % �ж���չ���Ƿ���У����Ƿ���map����
    if ~checkPath(closestNode(1:2), newPoint, map) 
        failedAttempts = failedAttempts + 1;
        continue;
    end
	%�����յ�
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end 
	%�ж��½ڵ��Ƿ�����������
    [A, I2] = min( distanceCost(RRTree(:,1:2),newPoint) ,[],1);
    if distanceCost(newPoint,RRTree(I2(1),1:2)) < threshold, failedAttempts = failedAttempts + 1; continue; end 
	 % ��ӽڵ�
    RRTree = [RRTree; newPoint I(1)];
    failedAttempts = 0;
    % ����·��
    if display, 
        plot([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)],'LineWidth',1);
        pause(0.05);%������ʾ�ٶ�
    end  
end

if display && pathFound, line([closestNode(2);goal(2)],[closestNode(1);goal(1)]); counter = counter+1;M(counter) = getframe; end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end

%% ����·��
path = [goal];
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:2); path];
    prev = RRTree(prev,3);
end

pathLength = 0;
% ����·������
for i=1:length(path)-1, pathLength = pathLength + distanceCost(path(i,1:2),path(i+1,1:2)); end 
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
imshow(map);
%rectangle('position',[1 1 size(map)-1],'edgecolor','k');
plot(path(:,2),path(:,1),'LineWidth',2,'color','r');