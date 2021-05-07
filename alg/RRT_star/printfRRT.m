function printfRRT(RRTree)
[n,~]=size(RRTree);
if n>=2
for i=2:n
    a=RRTree(i,1:2);
    b=RRTree(RRTree(i,3),1:2);
    plot([a(2);b(2)],[a(1);b(1)],'LineWidth',1); 
    pause(0.05);%调整显示速度
    hold on;
end
end

end