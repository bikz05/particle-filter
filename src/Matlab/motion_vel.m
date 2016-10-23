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
a1 = 1;
a2 = 1;
a3 = 1;
a4 = 1;
a5 = 1;
a6 = 1;


for i = 2:size(data,1)
% for i = 46
	
	x = data(i-1,1);
	y = data(i-1,2);
	theta = data(i-1,3);

	x_p = data(i,1);
	y_p = data(i,2);
	theta_p = data(i,3);

	dt = data(i,4) - data(i-1,4);

	if (y-y_p == 0) && (x-x_p == 0) 
		mu = 0;
	else
		mu = 0.5 * ( (x-x_p)*cos(theta) + (y-y_p)*sin(theta) ) / ( (y-y_p)*cos(theta) - (x-x_p)*sin(theta) ); 
	end

	x_star = 0.5*(x+x_p) + mu*(y-y_p);
	y_star = 0.5*(y+y_p) + mu*(x_p-x);

	r_star = sqrt((x-x_star)^2+(y-y_star)^2);

	d_theta = atan2(y_p-y_star,x_p-x_star) - atan2(y-y_star,x-x_star);

	v_hat = d_theta*r_star/dt;

	omega_hat = d_theta/dt;

	vel_state(i-1,1) = v_hat;

	vel_state(i-1,2) = omega_hat;

	vel_state(i-1,3) = dt;

	gamma_hat = (theta_p-theta)/dt - omega_hat;

end

odom_state = zeros(size(data,1),4);

odom_state(1,:) = data(1,:);


for i = 2:size(odom_state,1)

	% x
	odom_state(i,1) = odom_state(i-1,1) + vel_state(i-1,1)*cos(odom_state(i-1,3))*vel_state(i-1,3);

	% y
	odom_state(i,2) = odom_state(i-1,2) + vel_state(i-1,2)*sin(odom_state(i-1,3))*vel_state(i-1,3);

	% theta
	odom_state(i,3) = odom_state(i-1,3) + vel_state(i-1,3)*vel_state(i-1,3);

	% time
	odom_state(i,4) = odom_state(i-1,4) + vel_state(i-1,3);

end

% figure();
% plot(odom_state(1:800,1)/100, odom_state(1:800,2)/100);
% axis([-8 12 -9 1]);

figure();
plot(data(1:end,1)/100, data(1:end,2)/100);
axis([-8 12 -9 1]);

figure();
plot(vel_state(:,1));

% acummulate = 0;

% for i = 1:1000
% 	acummulate = acummulate + sample_normal(4);
% end

% acummulate/1000







