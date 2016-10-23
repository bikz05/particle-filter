clc
clear
start = 0;
final = 1000;
n = 10000;
z_sample = linspace(start,final,n);


z_pred = 500; %predict measurement

%tuning parameters
w_hit = 1;
w_short = 1;
w_max = 0.01;
w_rand = 1;
lambda_short = 0.005;
sigma_hit = 30;
z_max = 980; %max measurement dist

%p_hit
p_hit = zeros(1,n);
eta_hit = 1/((2*pi*(sigma_hit^2))^0.5); 

for i = 1:length(z_sample)
    if z_sample(i) >= 0 && z_sample(i) <= z_max
        p_hit(i)  = eta_hit.*exp(-1/2 .* ((z_sample(i) - z_pred)/(sigma_hit)).^2);
    else
        p_hit(i) = 0;
    end
end

%p_short
p_short = zeros(1,n);
short_eta = 1/(1 - exp(-lambda_short*z_pred));
for i = 1:length(z_sample)
    if z_sample(i) >= 0 && z_sample(i) <= z_pred
        p_short(i) = short_eta.*lambda_short.*exp(-lambda_short.*z_sample(i));
    else
        p_short(i) = 0;
    end
end

%p_max
p_max = zeros(1,n);
for i = 1:length(z_sample)
    if z_sample(i) >= z_max
        p_max(i) = 1;
    else
        p_max(i) = 0;
    end
end
%p_rand
p_rand = zeros(1,n);
for i = 1:length(z_sample)
    if z_sample(i)>=0 && z_sample(i) < z_max 
        p_rand(i) = 1/z_max;
    else
        p_rand(i) = 0;
    end
end

figure(1); hold on;
% plot(z_sample,p_hit,'r-');
plot(z_sample,w_hit.*p_hit + w_short.*p_short + w_max.*p_max + w_rand.*p_rand,'r-');






