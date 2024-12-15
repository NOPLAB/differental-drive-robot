clear;
clc;

% 画像を読み込み
img = imread("github.jpg");
% imshow(img);

points = img2points(img);

path = points2path(points, 0.01);

path = path';

robotInitialLocation = path(1,:);
robotGoal = path(end,:);

initialOrientation = 3.14;

robotCurrentPose = [robotInitialLocation initialOrientation]';

r = 0.025
l = 0.146

robot = differentialDriveKinematics("TrackWidth", l, "VehicleInputs", "WheelSpeeds");
robot.WheelRadius = r;

% figure
% plot(path(:,1), path(:,2),'k--d')
% xlim([0 5])
% ylim([0 5])

controller = controllerPurePursuit;

controller.Waypoints = path;
controller.DesiredLinearVelocity = 3.0; % 5.0
controller.MaxAngularVelocity = inf;
controller.LookaheadDistance = 0.05;

goalRadius = 0.005;
distanceToGoal = norm(robotInitialLocation - robotGoal);

filter = zeros(1, 1);

% Initialize the simulation loop
sampleTime = 0.02;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

res_w1 = [];
res_w2 = [];

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega, next] = controller(robotCurrentPose);

    w1 = (2*v - L * omega) / 2;
    w2 = (2*v + L * omega) / 2;

    res_w1 = [res_w1 w1];
    res_w2 = [res_w2 w2];

    diff = [next(1) - robotCurrentPose(1), next(2) - robotCurrentPose(2)];
    w = (atan2(diff(2), diff(1)) - (mod(robotCurrentPose(3) + pi, 2*pi) - pi)) / sampleTime;

    newSpeed = mean([filter(2:end), 10.0 / abs(w)]);
    filter = [filter(2:end), newSpeed];

    controller.DesiredLinearVelocity = newSpeed;
    newSpeed

    % disp([v, omega, w1 w2])

    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [w1 w2]);    
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 2])
    ylim([0 2])

    waitfor(vizRate);
end

% CSVに書き出すデータを準備
output_data = table(res_w2.' * r, res_w1.' * r);

% ファイル名を指定
output_filename = 'Data.csv';

% CSVに書き出し
writetable(output_data, output_filename);