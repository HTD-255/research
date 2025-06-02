% Define an empty scenario.
scenario = drivingScenario;
scenario.SampleTime = 0.01;

roadCenters = [0 0; 50 0; 100 0; 250 20; 500 40];
mainRoad = road(scenario, roadCenters, 'lanes',lanespec(2));
barrier(scenario,mainRoad);


% Create the ego vehicle that travels at 25 m/s along the road.  Place the
% vehicle on the right lane by subtracting off half a lane width (1.8 m)
% from the centerline of the road.
egoCar = vehicle(scenario, 'ClassID', 1);
trajectory(egoCar, roadCenters(2:end,:) - [0 1.8], 25); % On right lane

% Add a car in front of the ego vehicle
leadCar = vehicle(scenario, 'ClassID', 1);
trajectory(leadCar, [70 0; roadCenters(3:end,:)] - [0 1.8], 25); % On right lane

% Add a car that travels at 35 m/s along the road and passes the ego vehicle
passingCar = vehicle(scenario, 'ClassID', 1);
waypoints = [0 -1.8; 50 1.8; 100 1.8; 250 21.8; 400 32.2; 500 38.2];
trajectory(passingCar, waypoints, 35);

% Add a car behind the ego vehicle
chaseCar = vehicle(scenario, 'ClassID', 1);
trajectory(chaseCar, [25 0; roadCenters(2:end,:)] - [0 1.8], 25); % On right lane

sensors = cell(8,1);
% Front-facing long-range radar sensor at the center of the front bumper of the car.
sensors{1} = drivingRadarDataGenerator('SensorIndex', 1, 'RangeLimits', [0 174], ...
    'MountingLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0, 0.2], 'FieldOfView', [20, 5]);

% Rear-facing long-range radar sensor at the center of the rear bumper of the car.
sensors{2} = drivingRadarDataGenerator('SensorIndex', 2, 'MountingAngles', [180 0 0], ...
    'MountingLocation', [-egoCar.RearOverhang, 0, 0.2], 'RangeLimits', [0 30], 'FieldOfView', [20, 5]);

% Rear-left-facing short-range radar sensor at the left rear wheel well of the car.
sensors{3} = drivingRadarDataGenerator('SensorIndex', 3, 'MountingAngles', [120 0 0], ...
    'MountingLocation', [0, egoCar.Width/2, 0.2], 'RangeLimits', [0 30], 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Rear-right-facing short-range radar sensor at the right rear wheel well of the car.
sensors{4} = drivingRadarDataGenerator('SensorIndex', 4, 'MountingAngles', [-120 0 0], ...
    'MountingLocation', [0, -egoCar.Width/2, 0.2], 'RangeLimits', [0 30], 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Front-left-facing short-range radar sensor at the left front wheel well of the car.
sensors{5} = drivingRadarDataGenerator('SensorIndex', 5, 'MountingAngles', [60 0 0], ...
    'MountingLocation', [egoCar.Wheelbase, egoCar.Width/2, 0.2], 'RangeLimits', [0 30], ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Front-right-facing short-range radar sensor at the right front wheel well of the car.
sensors{6} = drivingRadarDataGenerator('SensorIndex', 6, 'MountingAngles', [-60 0 0], ...
    'MountingLocation', [egoCar.Wheelbase, -egoCar.Width/2, 0.2], 'RangeLimits', [0 30], ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Front-facing camera located at front windshield.
sensors{7} = visionDetectionGenerator('SensorIndex', 7, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], 'Height', 1.1);

% Rear-facing camera located at rear windshield.
sensors{8} = visionDetectionGenerator('SensorIndex', 8, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.2*egoCar.Wheelbase 0], 'Height', 1.1, 'Yaw', 180);

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

toSnap = true;
while advance(scenario) && ishghandle(BEP.Parent)
    % Get the scenario time
    time = scenario.SimulationTime;

    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoCar);

    % Simulate the sensors
    detectionClusters = {};
    isValidTime = false(1,8);
    for i = 1:8
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

        % Update bird's-eye plot
        updateBEP(BEP, egoCar, detectionClusters, confirmedTracks, positionSelector, velocitySelector);
    end

    % Snap a figure for the document when the car passes the ego vehicle
    if ta(1).Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
    end
end
displayEndOfDemoMessage(mfilename);