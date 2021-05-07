%% ��������
map=im2bw(imread('1.bmp')); 
source=[10 10]; %�������
goal=[490 490]; %�յ�����
stepsize = 10;  % RRT��ÿ�������Ĳ���
threshold = 10; % �ڵ������ֵ��С�ڸþ��뼴����Ϊ��ͬ
maxFailedAttempts = 10000;% ����Դ���
display = true; % RRT�㷨�Ŀ���
%%�ж������յ�����λ���Ƿ����ϰ���
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display,imshow(map);%���Ƶ�ͼ
end
hold on;
%���
plot(source(2),source(1),'o');
plot(goal(2),goal(1),'o');
tic;  % tic-toc: Functions for Elapsed Time
% RRT�㷨�����岿��
RRTree = double([source -1]);%��������� 
failedAttempts = 0;
pathFound = false;

while failedAttempts <= maxFailedAttempts  
    %���ݸ���������ɵ�
    if rand < 0.5
        x_rand = rand(1,2) .* size(map);   % ����������������
    else
        x_rand = goal; % �����յ�
    end
	% �ҵ�����������������������̵Ľڵ�
    % A �Ǿ���x_rand����Ľڵ���룬I����������
    [A, I] = min( distanceCost(RRTree(:,1:2),x_rand) ,[],1); 
    x_nearest = RRTree(I(1),1:2);
    % �������ɵĵ��λ��
    thedis = atan2(x_rand(1)-x_nearest(1),x_rand(2)-x_nearest(2));  
    newPoint = double(int32(x_nearest(1:2) + stepsize * [sin(thedis)  cos(thedis)]));
    % �ж���չ���Ƿ���У����Ƿ���map����
    if ~checkPath(x_nearest(1:2), newPoint, map) 
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
    plot([x_nearest(2);newPoint(2)],[x_nearest(1);newPoint(1)],'LineWidth',1);
    pause(0.05);%������ʾ�ٶ�
    end

end

if display && pathFound, 
    plot([x_nearest(2);goal(2)],[x_nearest(1);goal(1)],'LineWidth',1);
        pause(0.05);%������ʾ�ٶ�
end
if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end

%% ��������������յ㿪ʼ�����ҵ����·��
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
plot(path(:,2),path(:,1),'LineWidth',2,'color','r');