%% �����ϰ���(����Ϊ������Ҫ�Ƿ������)
%x0=100; y0=100; z0=100;%����
circleCenter = [100,100,100;50,50,50;100,40,60;150,100,100;60,130,50];
r=[50;20;20;15;15];%�뾶
cylinderMatrix = [10,100;40,40];%Բ������������
cylinderRMatrix = [10;20];%Բ����뾶
cylinderHMatrix = [200;150];%Բ����ĸ�
%���濪ʼ����
figure(1);
[x,y,z]=sphere;
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end
for i = 1:size(cylinderMatrix)   %����Բ�����ϰ���
    [x,y,z] = cylinder(cylinderRMatrix(i));%������(0,0)ΪԲ�ģ��߶�Ϊ[0,1]���뾶ΪR��Բ��
    mesh(x + cylinderMatrix(i,1),y + cylinderMatrix(i,2),z*cylinderHMatrix(i));
    hold on;
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
searchSize = [200 200 200];      %̽���ռ�������
%�ռ�������ܴ�Ļ����ᵼ���������ķ�������һ��
%% ���������յ�
hold on;%����ͼ�α��ֹ���
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
tic;  % tic-toc: Functions for Elapsed Time
RRTree1 = double([source -1]);
RRTree2=double([goal -1]);
failedAttempts = 0;
tree1ExpansionFail=false;
tree2ExpansionFail=false;
%% ѭ��
while ~tree1ExpansionFail || ~tree2ExpansionFail
    if ~tree1ExpansionFail 
        [RRTree1,pathFound,tree1ExpansionFail,closestNode,newPoint]=rrtExtend3(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,threshold,circleCenter,r,searchSize,cylinderMatrix,cylinderRMatrix,cylinderHMatrix);
        if ~tree1ExpansionFail && isempty(pathFound) && display
           %%%%
           plot3([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'LineWidth',1);end
    pause(0.05);
    end
    if ~tree2ExpansionFail 
        [RRTree2,pathFound,tree2ExpansionFail,closestNode,newPoint]=rrtExtend3(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,threshold,circleCenter,r,searchSize,cylinderMatrix,cylinderRMatrix,cylinderHMatrix);
        if ~isempty(pathFound), pathFound(4:5)=pathFound(5:-1:4); end 
        if ~tree2ExpansionFail && isempty(pathFound) && display
           %%%%
            plot3([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'LineWidth',1);end
    pause(0.05);
    end
%% retrieve path from parent information
    if ~isempty(pathFound) % path found
        if display
            plot3([RRTree1(pathFound(1,4),1);RRTree2(pathFound(1,5),1)],[RRTree1(pathFound(1,4),2);RRTree2(pathFound(1,5),2)],[RRTree1(pathFound(1,4),3);RRTree2(pathFound(1,5),3)],'LineWidth',1);
        end
         path=[pathFound(1,1:3)]; 
        prev=pathFound(1,4);
        % ��ӵ�һ�����Ľڵ�
        while prev>0
           path = [RRTree1(prev,1:3); path];
           prev = RRTree1(prev,4);
        end
        prev=pathFound(1,5); 
        % ��ӵڶ������Ľڵ�
        while prev>0
            path=[path;RRTree2(prev,1:3)];
            prev=RRTree2(prev,4);
        end
        break
     end
end
if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end
pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + distanceCost3(path(i,1:3),path(i+1,1:3)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
figure(2)
[x,y,z]=sphere;
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end
for i = 1:size(cylinderMatrix)  %����Բ�����ϰ���
    [x,y,z] = cylinder(cylinderRMatrix(i));%������(0,0)ΪԲ�ģ��߶�Ϊ[0,1]���뾶ΪR��Բ��
    mesh(x + cylinderMatrix(i,1),y + cylinderMatrix(i,2),z*cylinderHMatrix(i));
    hold on;
end
axis equal
hold on;
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
plot3(path(:,1),path(:,2),path(:,3),'LineWidth',2,'color','r');
