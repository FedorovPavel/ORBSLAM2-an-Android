clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');

% -------------------------------------------------------------------------
% Select dataset (comment in/out)

countSample = 200;
filePathA = 'meDataSet/A_0.txt'; 
filePathAL = 'meDataSet/AL_78_0.txt';
filePathG = 'meDataSet/G_78_0.txt';
filePathM = 'meDataSet/M_0.txt';
filePathT = 'meDataSet/example.txt';
startTime = 0;
stopTime = 26;
samplePeriod = 0.005;

fid = fopen(filePathAL, 'r');
formatSpec = '%f,%f,%f';
size = [3 Inf];
accel = fscanf(fid, formatSpec, size);
fclose(fid);
accel = accel';
accXR = accel(:, 1);
accYR = accel(:, 2);
accZR = accel(:, 3);

fid = fopen(filePathG, 'r');
[gyro, count] = fscanf(fid, formatSpec, size);
fclose(fid);
gyro = gyro';
gyrXR = gyro(:,1);
gyrYR = gyro(:,2);
gyrZR = gyro(:,3);
time = zeros(length(gyrXR), 1);
time(1) = 0;
vel = zeros(length(accXR), 3);
pos = zeros(length(accXR),3);
for i=2:length(time);
    time(i) = time(i-1) + samplePeriod;
end

AHRSalgorithm = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit', 1);
quat = zeros(length(time), 4);
initAHRS = false;
iteration = length(gyrXR) / countSample - 1;
for K = 1: iteration
    accX = zeros(countSample,1);
    accY = zeros(countSample,1);
    accZ = zeros(countSample,1);
    gyrX = zeros(countSample,1);
    gyrY = zeros(countSample,1);
    gyrZ = zeros(countSample,1);
    for i=1:countSample
        accX(i) = accXR(i+(K - 1)*countSample);
        accY(i) = accYR(i+(K - 1)*countSample);
        accZ(i) = accZR(i+(K - 1)*countSample);
        
        gyrX(i) = gyrXR(i+(K - 1)*countSample);
        gyrY(i) = gyrYR(i+(K - 1)*countSample);
        gyrZ(i) = gyrZR(i+(K - 1)*countSample);
    end
    % -------------------------------------------------------------------------
    % Detect stationary periods

    % Compute accelerometer magnitude
    acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

    % HP filter accelerometer data
    filtCutOff = 0.001;
    [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
    acc_magFilt = filtfilt(b, a, acc_mag);

    % Compute absolute value
    acc_magFilt = abs(acc_magFilt);

    % LP filter accelerometer data
    filtCutOff = 5;
    [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
    acc_magFilt = filtfilt(b, a, acc_magFilt);

    % Threshold detection
    stationary = acc_magFilt < 0.05;

    % -------------------------------------------------------------------------
    % Plot data raw sensor data and stationary periods

    % figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
    % ax(1) = subplot(2,1,1);
    %     hold on;
    %     plot(time, gyrX, 'r');
    %     plot(time, gyrY, 'g');
    %     plot(time, gyrZ, 'b');
    %     title('Gyroscope');
    %     xlabel('Time (s)');
    %     ylabel('Angular velocity (^\circ/s)');
    %     legend('X', 'Y', 'Z');
    %     hold off;
    % ax(2) = subplot(2,1,2);
    %     hold on;
    %     plot(time, accX, 'r');
    %     plot(time, accY, 'g');
    %     plot(time, accZ, 'b');
    %     plot(time, acc_magFilt, ':k');
    %     plot(time, stationary, 'k', 'LineWidth', 2);
    %     title('Accelerometer');
    %     xlabel('Time (s)');
    %     ylabel('Acceleration (g)');
    %     legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
    %     hold off;
    % linkaxes(ax,'x');

    % -------------------------------------------------------------------------
    % Compute orientation

    
    

    % Initial convergence
    if (initAHRS == false)
        AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX) mean(accY) mean(accZ)]);
    end

    % For all data
    for t = 1:length(accX)
        if(stationary(t))
            AHRSalgorithm.Kp = 0.5;
        else
            AHRSalgorithm.Kp = 0;
        end
        AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
        quat(t + (K - 1)*countSample,:) = AHRSalgorithm.Quaternion;
    end

    quatR = zeros(countSample, 4);
    for i=1:countSample
        quatR(i) = quat(i+(K-1)*countSample);
    end
    % -------------------------------------------------------------------------
    % Compute translational accelerations
    

    % Rotate body accelerations to Earth frame
    acc = quaternRotate([accX accY accZ], quaternConj(quatR));

    % % Remove gravity from measurements
    % acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     % unnecessary due to velocity integral drift compensation

    % Convert acceleration measurements to m/s/s
    %acc = acc * 9.81;

% % Plot translational accelerations
% figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerations');
% hold on;
% plot(time, acc(:,1), 'r');
% plot(time, acc(:,2), 'g');
% plot(time, acc(:,3), 'b');
% title('Acceleration');
% xlabel('Time (s)');
% ylabel('Acceleration (m/s/s)');
% legend('X', 'Y', 'Z');
% hold off;

% -------------------------------------------------------------------------
% Compute translational velocities

%acc(:,3) = acc(:,3) - 9.81;

% Integrate acceleration to yield velocity
%llll = size(acc);
    
    if (initAHRS == false)
        for t = 2:countSample
            vel(t + (K-1)*countSample,:) = vel(t-1 + (K-1)*countSample,:) + acc(t,:) * samplePeriod;
            if(stationary(t) == 1)
                vel(t + (K-1)*countSample,:) = [0 0 0];     % force zero velocity when foot stationary
            end
        end
    else 
        for t = 1:countSample
            vel(t + (K-1)*countSample,:) = vel(t-1 + (K-1)*countSample,:) + acc(t,:) * samplePeriod;
            if(stationary(t) == 1)
                vel(t + (K-1)*countSample,:) = [0 0 0];     % force zero velocity when foot stationary
            end
        end
    end


% Compute integral drift during non-stationary periods
%velDrift = zeros(length(accX),3);
% stationaryStart = find([0; diff(stationary)] == -1);
% stationaryEnd = find([0; diff(stationary)] == 1);
% for i = 1:numel(stationaryEnd)
%     driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
%     enum = 1:(stationaryEnd(i) - stationaryStart(i));
%     drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
%     velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
% end

% Remove integral drift
%vel = vel - velDrift;

% Plot translational velocity
% figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
% hold on;
% plot(time, vel(:,1), 'r');
% plot(time, vel(:,2), 'g');
% plot(time, vel(:,3), 'b');
% title('Velocity');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% legend('X', 'Y', 'Z');
% hold off;

% -------------------------------------------------------------------------
% Compute translational position

% Integrate velocity to yield position

    if (initAHRS == false)
        for t = 2:countSample
            pos(t + (K-1)*countSample,:) = pos(t-1 + (K-1)*countSample,:) + vel(t + (K-1)*countSample,:) * samplePeriod;    % integrate velocity to yield position
        end
    else 
        for t = 1:countSample
            pos(t + (K-1)*countSample,:) = pos(t-1 + (K-1)*countSample,:) + vel(t + (K-1)*countSample,:) * samplePeriod;    % integrate velocity to yield position
        end
    end
    initAHRS = true;
end
% Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot(time, pos(:,1), 'r');
plot(time, pos(:,2), 'g');
plot(time, pos(:,3), 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Plot 3D foot trajectory

% % Remove stationary periods from data to plot
% posPlot = pos(find(~stationary), :);
% quatPlot = quat(find(~stationary), :);
posPlot = pos;
quatPlot = quat;

% Extend final sample to delay end of animation
extraTime = 20;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];

% Create 6 DOF animation
SamplePlotFreq = 4;
Spin = 120;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
