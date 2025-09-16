clc; clear; close all;

data = readmatrix("data_logg_speedy_old.csv");

% Assuming columns: timestamp | voltage | angular_speed
time = data(:,1);
voltage = data(:,2) * 12.0;
angular_speed = data(:,3);

%% Estimate sample time
dt = mean(diff(time)); % approximate sampling interval
Ts = dt;               % sample time for discrete-time identification

%% Plot raw data
figure;
subplot(2,1,1);
plot(time, voltage); 
xlabel('Time [s]'); ylabel('Voltage [V]');
title('Input Voltage');

subplot(2,1,2);
plot(time, angular_speed); 
xlabel('Time [s]'); ylabel('Angular Speed [rad/s]');
title('Output Angular Speed');

%% System Identification
% Create an iddata object
data_id = iddata(angular_speed, voltage, Ts);

% Estimate a transfer function
% Here we assume a second-order system; adjust '2' if you think it's different
sys_tf = tfest(data_id, 2, 0);

%% Display results
disp('Estimated Transfer Function:');
sys_tf

%% Plot Bode and Step Response
figure;
bode(sys_tf);
title('Bode Plot of Estimated TF');

figure;
step(sys_tf);
title('Step Response of Estimated TF');