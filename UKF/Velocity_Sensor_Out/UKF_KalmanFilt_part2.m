clear; % Clear variables
datasetNum = 1; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);
%% How do you want to do this version
camLinVel = proj2Data.linearVel;
camAngVel = proj2Data.angVel;
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time step in real time

%% Calculate Kalmann Filter
for i = 1:length(sampledTime)
    %% FILL IN THE FOR LOOP
    dt = sampledData(i).t - prevTime;
    [covarEst, uEst] = pred_step(uPrev, covarPrev,sampledData(i).omg,sampledData(i).acc,dt);
    z_t = [camLinVel';camAngVel'];
    update = upd_step(z_t(:,i), covarEst, uEst);
    savedStates(:,i) = update(:,1);
    covarPrev = update(:,2:16);
    uPrev = update(:,1);
    prevTime = sampledData(i).t;
    
end

plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);
