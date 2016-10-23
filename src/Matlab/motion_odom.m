clc;
clear;
close all;

filename = 'odom_only_data1.txt';
delimiterIn = ' ';
data = importdata(filename,delimiterIn,0);


% linear_vel = zeros(size(data,1)-1,1);
% angular_vel = zeros(size(data,1)-1,1);

% linear_vel, angular_vel, delta_t
vel_state = zeros(size(data,1)-1,3);

% parameters
a1 = .0005;
a2 = .0005;
a3 = .03;
a4 = .03;

num_samples = 1000;

cell_sample = cell(size(data,1), num_samples);

x_est = zeros(size(data,1),1);
y_est = zeros(size(data,1),1);
theta_est = zeros(size(data,1),1);

x_est(1,1) = data(1,1);
y_est(1,1) = data(1,2);
theta_est(1,1) = data(1,3);

for i = 2:size(data,1)
% for i = 2:47
	
	x_bar = data(i-1,1);
	y_bar = data(i-1,2);
	theta_bar = data(i-1,3);

	xp_bar = data(i,1);
	yp_bar = data(i,2);
	thetap_bar = data(i,3);

	d_rot_1 = atan2(yp_bar-y_bar, xp_bar-x_bar) - theta_bar;
	d_trans = sqrt((x_bar-xp_bar)^2 + (y_bar-yp_bar)^2);
	d_rot_2 = thetap_bar - theta_bar - d_rot_1;


	if d_trans <= 0.01 && d_rot_1+d_rot_2 <= 0.01
		% disp('skip one iteration');
		x_est(i) = x_est(i-1);
		y_est(i) = y_est(i-1);
		theta_est(i) = theta_est(i-1);
		continue;
	end

	xp_sum = 0;
	yp_sum = 0;
	thetap_sum = 0;

	for j = 1 : num_samples

		d_rot_1_hat = d_rot_1 - sample_normal(a1*d_rot_1*d_rot_1 + a2*d_trans*d_trans);
		d_trans_hat = d_trans - sample_normal(a3*d_trans*d_trans + a4*d_rot_1*d_rot_1 + a4*d_rot_2*d_rot_2);
		d_rot_2_hat = d_rot_2 - sample_normal(a2*d_trans*d_trans + a1*d_rot_2*d_rot_2);

		xp = x_est(i-1,1) + d_trans_hat*cos(theta_est(i-1,1)+d_rot_1_hat);
		yp = y_est(i-1,1) + d_trans_hat*sin(theta_est(i-1,1)+d_rot_1_hat);
		thetap = theta_est(i-1,1) + d_rot_1_hat + d_rot_2_hat;

		cell_sample{i,j} = [xp yp thetap];

		xp_sum = xp_sum + xp;
		yp_sum = yp_sum + yp;
		thetap_sum = thetap_sum + thetap;

	end

	x_est(i) = xp_sum/num_samples;
	y_est(i) = yp_sum/num_samples;
	theta_est(i) = thetap_sum/num_samples;

end

figure();
plot(x_est/100,y_est/100);
axis([-8 12 -9 1]);


% odom_state = zeros(size(data,1),4);

% odom_state(1,:) = data(1,:);


% for i = 2:size(odom_state,1)

% 	% x_bar
% 	odom_state(i,1) = odom_state(i-1,1) + vel_state(i-1,1)*cos(odom_state(i-1,3))*vel_state(i-1,3);

% 	% y_bar
% 	odom_state(i,2) = odom_state(i-1,2) + vel_state(i-1,2)*sin(odom_state(i-1,3))*vel_state(i-1,3);

% 	% theta_bar
% 	odom_state(i,3) = odom_state(i-1,3) + vel_state(i-1,3)*vel_state(i-1,3);

% 	% time
% 	odom_state(i,4) = odom_state(i-1,4) + vel_state(i-1,3);

% end

% figure();
% plot(odom_state(1:800,1)/100, odom_state(1:800,2)/100);
% axis([-8 12 -9 1]);

% % figure();
% % plot(data(1:end,1)/100, data(1:end,2)/100);
% % axis([-8 12 -9 1]);

% figure();
% plot(vel_state(:,1));

% acummulate = 0;

% for i = 1:1000
% 	acummulate = acummulate + sample_normal(4);
% end

% acummulate/1000







