% A* algorithmʵ��
clc; clear; close all;

%% ������ȡ������
circleCenter = [8,8,8;5,5,5;8,4,6;10,10,10;6,13,5];
r=[5;2;2;1;1];%�뾶

%���濪ʼ����
figure(1);
[x,y,z]=sphere;
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end

axis equal;
%% ����
start=[1 1 1];%%����������Ķ���ϵ�������ֵͬ
goal=[10 10 10];
%searchSize = [250 250 250];  %̽���ռ�������
Alldirec = [[1,0,0];[0,1,0];[0,0,1];[-1,0,0];[0,-1,0];[0,0,-1];...
            [1,1,0];[1,0,1];[0,1,1];[-1,-1,0];[-1,0,-1];[0,-1,-1];...
            [1,-1,0];[-1,1,0];[1,0,-1];[-1,0,1];[0,1,-1];[0,-1,1];...
            [1,1,1];[-1,-1,-1];[1,-1,-1];[-1,1,-1];[-1,-1,1];[1,1,-1];...
            [1,-1,1];[-1,1,1]];
threshold = 10;%����ܶ�
stop = threshold*1.5;
g = [start, 0; goal, inf]; % ÿһ��ǰ������Ϊ�����꣬���ĸ���Ϊ·����ɢinfΪ�������+��
Path = [];
Parent = [];
Open = [start, g(findIndex(g,start),4) + getDist(start,goal)];

    
%% ���������յ�
hold on;%����ͼ�α��ֹ���
scatter3(start(1),start(2),start(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
tic;  % tic-toc: Functions for Elapsed Time

%% ��ѭ��
while ~isempty(Open)
    [xi, index] = findMin(Open);
    Open(index,:) = [];
    if getDist(xi, goal) < stop
        break;
    end
    children = getChildren(xi, Alldirec, threshold, circleCenter, r);
    scatter3(children(:,1),children(:,2),children(:,3),10,'filled','o');
    drawnow;
    [n,~] = size(children);
    for i = 1:n
        child = children(i,:);
        if findIndex(g, child) == 0   % child����g
            g = [g; child, inf];
        end
        a = g(findIndex(g, xi),4) + getDist(xi,child);
        if a < g(findIndex(g, child),4)
            g(findIndex(g, child),4) = a;
            Parent = setParent(Parent, child,xi);
            Open = setOpen(Open, child, a, goal);
        end
    end  
end
lastPoint = xi;
%% ���ݹ켣
x = lastPoint;
Path = x;
[n,~] = size(Parent);
while any(x ~= start)
    for i = 1:n
        if Parent(i,1:3) == x
            Path = [Parent(i,4:6); Path];
            break;
        end
    end
    x = Parent(i,4:6);
end
plot3([Path(:,1);goal(1)],[Path(:,2);goal(2)],[Path(:,3);goal(3)],'LineWidth',3,'color','r');
%% ����켣����
pathLength = 0;
[n,~] = size(Path);
for i = 1:n-1
    pathLength = pathLength + getDist(Path(i,:),Path(i+1,:));
end
pathLength = pathLength + getDist(Path(end,:),goal);
fprintf('·���ĳ���Ϊ:%f',pathLength);
%% ����
function children = getChildren(pos, Alldirec, step,circleCenter,circleR)
allchild = [];
[n,~] = size(Alldirec);
for i = 1:n
    direc = Alldirec(i,:);
    child = pos + direc * step;
    if ~checkCol(child, circleCenter,circleR)
        continue;
    end
    allchild = [allchild; child];
end
children = allchild;
end
%��ײ���
function flag = checkCol(pos, circleCenter,circleR)
[numberOfSphere, ~] = size(circleCenter);
flag = true;
for i = 1:numberOfSphere
    if getDist(pos, circleCenter(i,:)) <= circleR(i)
        flag = false;
        break;
    end
end

if pos(3) <= 0 flag = false; end
end
%���ø��ڵ�
function Par = setParent(Parent, xj, xi)
[n,~] = size(Parent);
if n == 0
    Par = [xj, xi];
else
    for i = 1:n
        if Parent(i,1:3) == xj
            Parent(i,4:6) = xi;
            Par = Parent;
            break;
        end
        if i == n
            Par = [Parent; xj, xi];
        end
    end
end
end

function Ope = setOpen(Open, child, a, goal)
[n,~] = size(Open);
if n == 0
    Ope = [child, a + getDist(child, goal)];
else
    for i = 1:n
        if Open(i,1:3) == child
            Open(i,4) = a + getDist(child, goal);
            Ope = Open;
        end
        if i == n
            Ope = [Open; child, a + getDist(child, goal)];
        end
    end
end
end
function index = findIndex(g, pos)
[n,~] = size(g);
index = 0;    % ��ʾû���ҵ�����
for i = 1:n
    if g(i,1:3) == pos
        index = i;   % ����Ϊi
        break;
    end
end
end

function d = getDist(x,y)
d = sqrt(sum((x - y).^2));
end

function [pos, index] = findMin(Open)
[~,index] = min(Open(:,4));
pos = Open(index,1:3);
end
