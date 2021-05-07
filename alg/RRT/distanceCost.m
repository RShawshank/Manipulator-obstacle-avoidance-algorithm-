%% distanceCost.m
function h=distanceCost(a,b)
	
	
	
h = sqrt((a(:,1)-b(:,1)).^2 + (a(:,2)-b(:,2)).^2 );