%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
x = zeros(2,length(sampledData));
for n = 2:length(sampledData)

    %% Initalize Loop load images
    
    c_w = corner(sampledData(n-1).img);

    %% Initalize the tracker to the last frame.
    
    pointTracker = vision.PointTracker;
    initialize(pointTracker,  c_w,  sampledData(n-1).img);
    
    %% Find the location of the next points;
    videoFrame = sampledData(n).img;
    [c_w_tracked,validity] = pointTracker(videoFrame);
    %% Calculate velocity
    % Use a for loop
    delta_t = sampledData(n).t - sampledData(n-1).t;
    displace = c_w_tracked(1,:) - c_w(1,:);
    
    xy_dot = displace/delta_t;
    x(:,n) = xy_dot';
    %% Calculate Height

%     Pose = estimatePose(sampledData,n);
%     Z_robot_height = Pose(3,1);
%     Z = zeros(length(xy_dot),1);
%     R = Pose(:,3:5);
%     for i = 1:length(xy_dot)
%         Position = R * [c_w_tracked(i,:),0]';
%         Z(i) = Position(3);
%     end
%     %% RANSAC    
%     % Write your own RANSAC implementation in the file velocityRANSAC
%     Vel = velocityRANSAC(xy_dot,c_w_tracked, Z_robot_height ,R ,0.9);
%     
%     %% Thereshold outputs into a range.
%     % Not necessary
%     
%     %% Fix the linear velocity
%     % Change the frame of the computed velocity to world frame
%     
%     %% ADD SOME LOW PASS FILTER CODE
%     % Not neceessary but recommended 
%     %estimatedV(:,n) = Vel;
%     
%     %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
%     estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
%     % Structure of the Vector Vel should be as follows:
end

%plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
