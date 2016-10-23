%% sample_normal: function description
function [outputs] = sample_normal(b_square)
	
	accumlate = 0;

	for i = 1:12
		rand_num = (rand-0.5) * 2 * sqrt(b_square); 
		accumlate = accumlate + rand_num;
	end


	outputs = 0.5*accumlate;
