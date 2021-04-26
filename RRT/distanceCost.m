%% distanceCost.m
function h=distanceCost(a,b)
	h = sqrt(sum((a-b).^2, 2));
	
	
