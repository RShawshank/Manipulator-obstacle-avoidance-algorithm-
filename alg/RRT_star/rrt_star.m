%% ��������
map=im2bw(imread('10.bmp')); 
source=[100 1000]; %�������
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
i = 1;
pathFound = false;
lowestCost=1000;pl=zeros(1,maxFailedAttempts);
while i <= maxFailedAttempts  
    pl(i)=lowestCost;
    i=i+1;
    
    %���ݸ���������ɵ�
    if rand < 0.5
        x_rand = rand(1,2) .* size(map);   % ����������������
    else
        x_rand = goal; % �����յ�
    end
	% �ҵ�����������������������̵Ľڵ�
    [~, I] = min( distanceCost(RRTree(:,1:2),x_rand) ,[],1); 
    x_nearest = RRTree(I(1),1:2);
    % �������ɵĵ��λ��
    thedis = atan2(x_rand(1)-x_nearest(1),x_rand(2)-x_nearest(2));  
    newPoint = double(int32(x_nearest(1:2) + stepsize * [sin(thedis)  cos(thedis)]));
    % �ж���չ���Ƿ���У����Ƿ���map����
    if ~checkPath(x_nearest(1:2), newPoint, map) 
        i = i + 1;
        continue;
    end
	%�����յ�
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end 
	%���²����Ľڵ� newPoint �����Զ���İ뾶��Χ��Ѱ�ҡ����ڡ�,����Ϊ�滻newPoint���ڵ�ı�ѡ
    x_near_list = NearestVertices(newPoint,RRTree);
    %��û���ڽ��Ľڵ�ʱ��ȡ������о�����̵Ľڵ�
    if  size(x_near_list)==[0,0]
        [~, I1]=min(distanceCost(RRTree(:,1:2),newPoint) ,[],1); 
        x_near_list = [RRTree(I1,1:3),I1];
    end
    %���μ��㡰���ڡ��ڵ㵽����·�����ۼ��� newPoint ��ÿ�������ڡ���·������
     sortlist=GetList(newPoint,x_near_list,RRTree);
     %�� x_near_list ѡ��x_min��Ϊ���ڵ�
    x_min=ChooseBestParent(sortlist,map);
    if ~size(x_min)==[0 0] 
        %�����µĽڵ�
        RRTree=[RRTree;[newPoint(1:2),x_min(4)]];
       %��������
        RRTree=RewireVertices(x_min,newPoint,sortlist,RRTree,map);
        % ����·��
        if display
            plot([x_min(2);newPoint(2)],[x_min(1);newPoint(1)],'LineWidth',1);pause(0.05);
        end
        if AllCost(RRTree,RRTree(end,:))+distanceCost(RRTree(end,1:2),goal)<lowestCost && distanceCost(newPoint,goal)<thedis
            lowestCost=AllCost(RRTree,RRTree(end,:))+distanceCost(RRTree(end,1:2),goal);
            tree1=RRTree;
            pl(i)=lowestCost;
        end
    else
        continue;
    end
end

if display && pathFound,plot([x_min(2);goal(2)],[x_min(1);goal(1)],'LineWidth',1);pause(0.05);%������ʾ�ٶ�
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