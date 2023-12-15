function [s,sd,sdd,t] = traj_gen(start_pos,final_pos,t_step)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%------------------------------Set up--------------------------------------

%R-Parameters
ar_max = 1000  ; 
vr_max = 600; 
vr_avg = 0.5 * vr_max;

%Theta-Parameters
alpha_max = 1000;
omega_max = 600;
omega_avg = 0.5 * omega_max;

%Z-Parameters
az_max = 1000;
vz_max = 600;
vz_avg = 0.5 * vz_max;

%---------------------Initial Parameters Calculations----------------------

a_max = [ar_max,alpha_max,az_max];
v_avg = [vr_avg,omega_avg,vz_avg];

%Distance Calculation
distance_r      = final_pos(1) - start_pos(1);
distance_theta  = final_pos(2) - start_pos(2);
distance_z      = final_pos(3) - start_pos(3);

%Negative Constant
%for further calculation
if distance_r < 0
    ncr = -1;
else
    ncr = 1;
end

if distance_theta < 0
    nctheta = -1;
else
    nctheta = 1;
end

if distance_z < 0
    ncz = -1;
else
    ncz = 1;
end

nc = [ncr, nctheta, ncz];

distance_r      = abs(distance_r);
distance_theta  = abs(distance_theta);
distance_z      = abs(distance_z);
disp(distance_r)
disp(distance_z)
disp(distance_theta)

distance = [distance_r,distance_theta,distance_z];


%Maximum Interval Estimation
Max_interval = [distance_r/vr_avg, distance_theta/omega_avg, distance_z/vz_avg];
t_max = max(Max_interval);
disp("t_max")
disp(t_max)
control_ind = find(Max_interval==t_max);

%Boundary Interval Calculation
a = a_max(control_ind);
v = v_avg(control_ind);
disp(a)
disp(v)
if a*t_max >= 4*v
    t_bound = (a*t_max - sqrt((a*t_max)^2 - 4*a*v*t_max))/(2*a);
else
    t_max = 2*sqrt(distance(control_ind)/a);
    t_bound = t_max/2;
end
disp("t_max_new")
disp(t_max)
disp("t_bound")
disp(t_bound)
%Controlled Accelaration Optimization
for i = 1:3
    if i ~= control_ind
        a_max(i) = distance(i)/(t_bound*(t_max - t_bound));
    end
    a_max(i) = nc(i) * a_max(i);
end


%-----------------------Trajectory Generator-------------------------------
%t Generate
%t_step = t_max/(nstep - 1);
nstep = ceil(t_max/t_step);
disp(t_step)
disp(nstep)
disp(t_max - t_bound)
t = 0:t_step:t_max;
if nstep < numel(t)
    nstep = nstep + 1;
end

%Profile Generate
sdd = zeros(nstep,3);
sd = zeros(nstep,3);
s = zeros(nstep,3);

for i = 1:3
    for j = 1:nstep
        switch t_bound==t_max/2
            case 0
                if t(j) <= t_bound
                    sdd(j,i) = a_max(i);
                    sd(j,i) = a_max(i)* t(j);
                    s(j,i) = start_pos(i) + 0.5*a_max(i)*(t(j))^2;
                elseif t_bound < t(j) && t(j) < t_max - t_bound
                    sdd(j,i) = 0;
                    sd(j,i) = a_max(i)*t_bound;
                    s(j,i) = start_pos(i) + 0.5*a_max(i)*t_bound^2 + a_max(i)*t_bound*(t(j)-t_bound);
                elseif t(j) >= t_max - t_bound
                    sdd(j,i) = -a_max(i);
                    sd(j,i) = -a_max(i)*t(j) + a_max(i)*t_max;
                    s(j,i) = start_pos(i) + a_max(i)*(-0.5*t_max^2 + t_max*t_bound - t_bound^2 - 0.5*(t(j))^2 + t_max*t(j));
                end 
       
            case 1
                if t(j) <= t_bound
                    sdd(j,i) = a_max(i);
                    sd(j,i) = a_max(i)* t(j);
                    s(j,i) = start_pos(i) + 0.5*a_max(i)*(t(j))^2;
                else
                    sdd(j,i) = -a_max(i);
                    sd(j,i) = -a_max(i)*t(j) + a_max(i)*t_max ;
                    s(j,i) = start_pos(i) + a_max(i)*(-0.5*t_max^2 + t_max*t_bound - t_bound^2 - 0.5*(t(j))^2 + t_max*t(j));
                end
        end
    end

end

s(end,1) = final_pos(1);
s(end,2) = final_pos(2);
s(end,3) = final_pos(3);
disp(size(s(:,2)))
disp(s(end,2))
disp(size(t'))
%-------------------------------Displays-----------------------------------

figure('Name','R','NumberTitle','off');
tiledlayout(3,1)
nexttile
plot(t',s(:,1));
title('s')
nexttile
plot(t',sd(:,1))
title('sd')
nexttile
plot(t',sdd(:,1))
title('sdd')

figure('Name','Theta','NumberTitle','off');
tiledlayout(3,1)
nexttile
hold on
plot(t',s(:,2))
plot([t_bound t_bound],[0 800],'k--')
plot([(t_max-t_bound) (t_max-t_bound)],[0 800],'k--')
hold off
title('position')
xlabel('time(s)')
ylabel('position')


nexttile
hold on
plot(t',sd(:,2))
plot([t_bound t_bound],[0 800],'k--')
plot([(t_max-t_bound) (t_max-t_bound)],[0 800],'k--')
hold off
title('velocity')
xlabel('time(s)')
ylabel('velocity')
nexttile
hold on
plot(t',sdd(:,2))
plot([t_bound t_bound],[-1000 1000],'k--')
plot([(t_max-t_bound) (t_max-t_bound)],[-1000 1000],'k--')
hold off
title('acceleration')
xlabel('time(s)')
ylabel('acceleration')

figure('Name','Z','NumberTitle','off');
tiledlayout(3,1)
nexttile
plot(t',s(:,3))
title('s')
nexttile
plot(t',sd(:,3))
title('sd')
nexttile
plot(t',sdd(:,3))
title('sdd')

figure('Name','Trajectory','NumberTitle','off');
% for i = 1:nstep
%     x = s(:,1).*cos(s(:,2).*pi./180);
%     y = s(:,1).*sin(s(:,2).*pi./180);
%     z = s(:,3);
%     plot3(x,y,z,'.')
% end
for i = 1:nstep
    x = 1*cos(s(:,2).*pi./180);
    y = 1*sin(s(:,2).*pi./180);
    z = s(:,3);
    plot3(x,y,z,'.')
end
axis equal 
end

