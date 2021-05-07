function h=distanceCost3(a,b)         %% distanceCost.m
h = sqrt((a(:,1)-b(:,1)).^2 + (a(:,2)-b(:,2)).^2 +(a(:,3)-b(:,3)).^2);
end
