% Define an empty scenario.
scenario = drivingScenario;
scenario.SampleTime = 0.01;
lastUpdateTime = -inf;

% Add all road segments
roadCenters = [40.09 20.14 0;
    42.8 -0.7 0;
    39.89 -20.14 0;
    14.7 -20.14 0;
    12.4 0.1 0;
    14.7 20.14 0;
    40.09 20.14 0];
road(scenario, roadCenters, 'Name', 'Road');

car = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [42.2 10.3 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [42.2 10.3 0;
    42.7 -1 0;
    41.1 -16 0;
    34.5 -26 0;
    23.2 -25.6 0;
    16 -21.9 0;
    11.6 -6.5 0;
    12.1 10.2 0;
    14.9 21.1 0;
    24.1 27.3 0;
    35 25.6 0;
    41.3 19.2 0;
    42.3 8.3 0];
trajectory(car, waypoints, 10);

% Add the non-ego actors
egoCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [27.5 -0.2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');

sensors = cell(1,1);
sensors{1} = visionDetectionGenerator('SensorIndex', 7, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], 'Height', 1.5);

% Register actor profiles with the sensors.
profiles = actorProfiles(scenario);
for m = 1:numel(sensors)
    if isa(sensors{m},'drivingRadarDataGenerator')
        sensors{m}.Profiles = profiles;
    else
        sensors{m}.ActorProfiles = profiles;
    end
end
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationThreshold', [4 5]);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

% Create the display and return a handle to the bird's-eye plot
BEP = createDemoDisplay(egoCar, sensors);

predictedPositions = [];
while advance(scenario) && ishghandle(BEP.Parent)
    % Get the scenario time
    time = scenario.SimulationTime;

    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoCar);

    % Simulate the sensors
    detectionClusters = {};
    isValidTime = false(1,8);
    for i = 1:numel(sensors)
        [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            for j = 1:numValidDets
                % Vision detections do not report SNR. The tracker requires
                % that they have the same object attributes as the radar
                % detections. This adds the SNR object attribute to vision
                % detections and sets it to a NaN.
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end

                % Remove the Z-component of measured position and velocity
                % from the Measurement and MeasurementNoise fields
                sensorDets{j}.Measurement = sensorDets{j}.Measurement([1 2 4 5]);
                sensorDets{j}.MeasurementNoise = sensorDets{j}.MeasurementNoise([1 2 4 5],[1 2 4 5]);
            end
            detectionClusters = [detectionClusters; sensorDets]; %#ok<AGROW>
        end
    end

    % Update the tracker if there are new detections
    if any(isValidTime)
        if isa(sensors{1},'drivingRadarDataGenerator')
            vehicleLength = sensors{1}.Profiles.Length;
        else
            vehicleLength = sensors{1}.ActorProfiles.Length;
        end

    confirmedTracks = updateTracks(tracker, detectionClusters, time);

    updateBEP(BEP, egoCar, detectionClusters, confirmedTracks, positionSelector, velocitySelector);

    if ~isempty(confirmedTracks)
        positions = getTrackPositions(confirmedTracks, positionSelector);
        velocities = getTrackVelocities(confirmedTracks, velocitySelector);
    
        carPredictedPos = positions(1,:);
        carVelocity = velocities(1,:);

            % === CẢI TIẾN: Chặn đầu thông minh ===
    leadTime = 4;  % Dự đoán xa hơn 4 giây
    targetIntercept = carPredictedPos + carVelocity * leadTime;

    fprintf('Target intercept at time %.2f: x = %.2f, y = %.2f\n', time, targetIntercept(1), targetIntercept(2));


    % === Cập nhật trajectory nếu đủ điều kiện ===
    if isempty(lastUpdateTime)
        lastUpdateTime = -inf;
    end

    % Cập nhật mỗi 0.5s để tránh quá tải
    if time - lastUpdateTime > 0.5
        egoPos = egoCar.Position;
        distance = norm(targetIntercept - egoPos(1:2));
        if distance > 1.0
            newTrajectory = [egoPos(1:2); targetIntercept];
            newTrajectory(:,3) = 0;
            trajectory(egoCar, newTrajectory, 2.5);  % đi tới intercept trong 2.5 giây
            lastUpdateTime = time;
        end
    end

    end
    end
end

disp('All predicted positions:');
disp(predictedPositions);