map=im2bw(imread('map1.bmp'));
source=[10 10]; 
goal=[490 490];
stepsize=20; 
disTh=20; 
maxFailedAttempts = 10000;
display=true;

if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end
hold on;
plot(source(1),source(2),'o');
plot(goal(1),goal(2),'o');
tic;
% ��һ�������������������
RRTree1=double([source -1]);
% �ڶ���������������յ����
RRTree2=double([goal -1]); 
counter=0;
% ��չʧ�ܱ�־����true
tree1ExpansionFail=false; 
tree2ExpansionFail=false;
while ~tree1ExpansionFail || ~tree2ExpansionFail 
    % ����㵽�յ������չ
    if ~tree1ExpansionFail 
        [RRTree1,pathFound,tree1ExpansionFail]=rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map); 
        if ~tree1ExpansionFail && isempty(pathFound) && display
            line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','g');
            counter=counter+1;M(counter)=getframe;
        end
    end
    % ���յ㵽��������չ
    if ~tree2ExpansionFail 
        [RRTree2,pathFound,tree2ExpansionFail]=rrtExtend(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map);
        if ~isempty(pathFound), pathFound(3:4)=pathFound(4:-1:3); end 
        if ~tree2ExpansionFail && isempty(pathFound) && display
            line([RRTree2(end,2);RRTree2(RRTree2(end,3),2)],[RRTree2(end,1);RRTree2(RRTree2(end,3),1)],'color','r');
            counter=counter+1;M(counter)=getframe;
        end
    end
    % ·���ɴ�
    if ~isempty(pathFound) 
         if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green');
            counter=counter+1;M(counter)=getframe;
         end
        % �������·��
        path=[pathFound(1,1:2)]; 
        prev=pathFound(1,3); 
        % ���ӵ�һ�����Ľڵ�
        while prev>0
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); 
        % ���ӵڶ������Ľڵ�
        while prev>0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        break;
    end
end
if display 
    disp('click/press any key');
    waitforbuttonpress; 
end
if size(pathFound,1)<=0, error('no path found. maximum attempts reached'); end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));