%% �����ϰ���(����Ϊ������Ҫ�Ƿ������)
%x0=100; y0=100; z0=100;%����
circleCenter = [100,100,100;50,50,50;100,40,60;150,100,100;60,130,50];
r=[50;20;20;15;15];%�뾶
%���濪ʼ����
figure(1);
[x,y,z]=sphere;
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end
axis equal
%%����������Ķ���ϵ�������ֵͬ
%% ����
source=[10 10 10];
goal=[150 150 150];
stepsize = 10;
threshold = 10;
maxFailedAttempts = 10000;
display = true;
searchSize = [250 250 250];      %̽���ռ�������
%% ���������յ�
hold on;%����ͼ�α��ֹ���
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
tic;  % tic-toc: Functions for Elapsed Time
RRTree = double([source -1]);
pathFound = false;
i = 1;
lowestCost=1000;pl=zeros(1,maxFailedAttempts);
%% ѭ��
while i <= maxFailedAttempts  
    pl(i)=lowestCost;
    i=i+1;
    %%���������
    if rand < 0.5
        sample = rand(1,3) .* searchSize;   
    else
        sample = goal; 
    end
    %%����������о���������������һ����
    [A, I] = min( distanceCost3(RRTree(:,1:3),sample) ,[],1); 
    closestNode = RRTree(I(1),1:3);
    %% ����������������������
    movingVec = [sample(1)-closestNode(1),sample(2)-closestNode(2),sample(3)-closestNode(3)];
    movingVec = movingVec/sqrt(sum(movingVec.^2));  %��λ��
    newPoint = closestNode + stepsize * movingVec;
    if ~checkPath3(closestNode, newPoint, circleCenter,r) % �ж��µĵ��Ƿ�ɴ�
        i = i + 1;
        continue;
    end
    %�����յ�
    if distanceCost3(newPoint,goal) < threshold, pathFound = true; break; end % �����յ�
   %���²����Ľڵ� newPoint �����Զ���İ뾶��Χ��Ѱ�ҡ����ڡ�,����Ϊ�滻newPoint���ڵ�ı�ѡ
     x_near_list = NearestVertices3(newPoint,RRTree);
     %��û���ڽ��Ľڵ�ʱ��ȡ������о�����̵Ľڵ�
    if  size(x_near_list)==[0,0]
        [~, I1]=min(distanceCost3(RRTree(:,1:3),newPoint) ,[],1); 
        x_near_list = [RRTree(I1,1:4),I1];
    end
    %���μ��㡰���ڡ��ڵ㵽����·�����ۼ��� newPoint ��ÿ�������ڡ���·������
     sortlist=GetList3(newPoint,x_near_list,RRTree);
     %�� x_near_list ѡ��x_min��Ϊ���ڵ�
    x_min=ChooseBestParent3(sortlist,circleCenter,r);
    if ~size(x_min)==[0 0] 
        %�����µĽڵ�
        RRTree=[RRTree;[newPoint(1:3),x_min(5)]];
       %��������
        RRTree=RewireVertices3(x_min,newPoint,sortlist,RRTree,circleCenter,r);
        % ����·��
        if display
            plot3([x_min(1);newPoint(1)],[x_min(2);newPoint(2)],[x_min(3);newPoint(3)],'LineWidth',1);pause(0.05);
        end
        if AllCost3(RRTree,RRTree(end,:))+distanceCost3(RRTree(end,1:3),goal)<lowestCost && distanceCost3(newPoint,goal)<thedis
            lowestCost=AllCost3(RRTree,RRTree(end,:))+distanceCost3(RRTree(end,1:3),goal);
            tree1=RRTree;
            pl(i)=lowestCost;
        end
    else
        continue;
    end
end

if display && pathFound, plot3([x_min(1);goal(1)],[x_min(2);goal(2)],[x_min(3);goal(3)]); end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end
%% ��������������յ㿪ʼ�����ҵ����·��
path = goal;
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:3); path];
    prev = RRTree(prev,4);
end

pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + distanceCost3(path(i,1:3),path(i+1,1:3)); end % ����·������
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
figure(2)
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end
axis equal
hold on;
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
plot3(path(:,1),path(:,2),path(:,3),'LineWidth',2,'color','r');
