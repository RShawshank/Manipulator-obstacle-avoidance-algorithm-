function h=distanceCost3(a,b)         %% distanceCost.m
	h = sqrt(sum((a-b).^2, 2));
end
