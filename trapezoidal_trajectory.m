%{
*
 NAME           : trapezoidal_trajectory.m
 AUTHOR         : Paweekorn Buasakorn
 DATE           : May 13th 2023
 MODIFIED BY    : -
 DESCRIPTION    : The following code is intended for beginners seeking to grasp 
                  the concept of synchronized trajectory, achieved through 
                  the implementation of a trapezoidal trajectory.
                  to use this function you can copy this command to the
                  Command Window >> trapezoidal_trajectory(0,100,400,500)
*
%}

function [position, velocity, acceleration, time] = trapezoidal_trajectory(initial_position, target_position, max_velocity, max_acceleration)

s = abs(target_position - initial_position); % Calculated Distance
dir = sign(target_position - initial_position); % Define Direcction
k = (max_velocity^2)/max_acceleration
% Define pattern of trapezoidal_trajectory
if s > (max_velocity^2)/max_acceleration
    % Calculate the time required for acceleration and deceleration for
    time_accel = max_velocity / max_acceleration;
    time_total = 2 * time_accel + (s -(max_velocity^2)/max_acceleration) / max_velocity;
else
    % Calculate the time required for acceleration and deceleration
    time_accel = sqrt(s/max_acceleration);
    time_total = 2 * time_accel;
end

% Generate the time array
time = 0:0.01:time_total;

% Initialize the position, velocity, and acceleration arrays
position = zeros(1, length(time));
velocity = zeros(1, length(time));
acceleration = zeros(1, length(time));

% Calculate the position, velocity, and acceleration for each time step
for i = 1:length(time)
    t = time(i);
    if t <= time_accel
        % Acceleration phase
        velocity(i) = max_acceleration * t * dir;
        position(i) = initial_position + dir * 0.5 * max_acceleration * t^2;
        acceleration(i) = max_acceleration * dir;
        max_velocity = velocity(i);
    elseif t < (time_total - time_accel)
        % Constant velocity phase
        velocity(i) = max_velocity;
        position(i) = initial_position + 0.5 * dir * max_acceleration * time_accel^2 + max_velocity * (t - time_accel);
        acceleration(i) = 0;
    elseif t >= (time_total - time_accel)
        % Deceleration phase
        velocity(i) = max_velocity - max_acceleration * dir * (t - (time_total - time_accel));
        position(i) = initial_position + 0.5 * dir * max_acceleration * time_accel^2 + max_velocity * (time_total  -2 * time_accel) - 0.5 * dir * max_acceleration * (t - (time_total - time_accel))^2 + max_velocity*(t-(time_total - time_accel));
        acceleration(i) = -max_acceleration * dir;
    end
end

subplot(3,1,1)
plot(time,position)
xlabel('time (sec)')
ylabel('pos (mm)')

subplot(3,1,2)
plot(time,velocity)
xlabel('time (sec)')
ylabel('velo (mm/s)')

subplot(3,1,3)
plot(time,acceleration)
xlabel('time (sec)')
ylabel('accel (mm/s^2)')
end