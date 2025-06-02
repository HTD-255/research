% check3_updated_with_tracking_recovery.m

scenario = drivingScenario;
scenario.SampleTime = 0.1;
lastUpdateTime = -inf;
rng(1);

lastTrackState = [];  % lưu lại trạng thái track gần nhất nếu mất tạm thời

roadCenters = [40.4 2.2 0;
    -2.9 8 0;
    -41.3 3.7 0;
    -40.8 -65.9 0;
    -1 -70.3 0;
    43.3 -64.5 0;
    40.4 2.2 0];
road(scenario, roadCenters, 'Name', 'Road');

car = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [42.2 10.3 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');

waypoints = [-18.5 7.6 0;
    7.8 7.6 0;
    25.8 7.1 0;
    43.3 0.7 0;
    56.5 -11.9 0;
    63.7 -30.4 0;
    58.4 -52.3 0;
    46.7 -63 0;
    25.8 -71.3 0;
    6.8 -70.8 0;
    -11.7 -71.3 0;
    -26.7 -69.8 0;
    -43.3 -64.9 0;
    -57.4 -53.7 0;
    -63.7 -32.8 0;
    -59.3 -10.9 0;
    -45.7 1.7 0;
    -30.1 7.1 0;
    -18.5 8 0];

numLaps = 2;
waypoints2 = repmat(waypoints(1:end-1,:), numLaps, 1);
waypoints2(end+1,:) = waypoints(end,:);
trajectory(car, waypoints2, 2);

egoCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-16.7 -30.4 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');

sensors = { visionDetectionGenerator('SensorIndex', 7, ...
    'FalsePositivesPerImage', 0, ...
    'DetectionProbability', 1.0, ...
    'SensorLocation', [0.75*egoCar.Wheelbase 0], ...
    'Height', 2.5, ...
    'MaxRange', 80, ...
    'DetectorOutput', 'Objects only') };
sensors{1}.ActorProfiles = actorProfiles(scenario);

tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationThreshold', [4 5]);

positionSelector = [1 0 0 0; 0 0 1 0];
velocitySelector = [0 1 0 0; 0 0 0 1];

BEP = createDemoDisplay(egoCar, sensors);

ppController = controllerPurePursuit;
ppController.DesiredLinearVelocity = 8;
ppController.MaxAngularVelocity = 2;
ppController.LookaheadDistance = 6;

usePurePursuit = true;
ppInitialized = false;

nlobj = nlmpc(4, 4, 2);
nlobj.Ts = 0.1;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 2;
nlobj.Model.StateFcn = @(x,u) vehicleKinematics(x,u,nlobj.Ts);
nlobj.Model.IsContinuousTime = false;
nlobj.MV(1).Min = -6;
nlobj.MV(1).Max = 10;
nlobj.MV(2).Min = -0.5;
nlobj.MV(2).Max = 0.5;
nlobj.States(1).Min = 10; nlobj.States(1).Max = 45;
nlobj.States(2).Min = -30; nlobj.States(2).Max = 30;
nlobj.States(3).Min = -pi; nlobj.States(3).Max = pi;
nlobj.States(4).Min = 0; nlobj.States(4).Max = 15;
nlobj.Weights.OutputVariables = [1 1 0 0];
nlobj.Weights.ManipulatedVariables = [0.2 0.2];
nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1];

x_mpc = [egoCar.Position(1:2), deg2rad(egoCar.Yaw), 0]';
u0 = [0; 0];

if ~exist('lastIntercept', 'var')
    lastIntercept = x_mpc(1:2);
end
while advance(scenario) && ishghandle(BEP.Parent)
    time = scenario.SimulationTime;
    ta = targetPoses(egoCar);

    detectionClusters = {};
    isValidTime = false(1,8);
    for i = 1:numel(sensors)
        [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            for j = 1:numValidDets
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end
                sensorDets{j}.Measurement = sensorDets{j}.Measurement([1 2 4 5]);
                sensorDets{j}.MeasurementNoise = ...
                    sensorDets{j}.MeasurementNoise([1 2 4 5], [1 2 4 5]);
            end
            detectionClusters = [detectionClusters; sensorDets];
        end
    end

    confirmedTracks = updateTracks(tracker, detectionClusters, time);
    updateBEP(BEP, egoCar, detectionClusters, confirmedTracks, positionSelector, velocitySelector);

    % === CHỌN TRACK GẦN NHẤT ===
    egoPos = x_mpc(1:2)';
    selectedTrack = [];
    minDist = inf;

    for i = 1:numel(confirmedTracks)
        t = confirmedTracks(i);
        dist = norm(t.State(1:2) - egoPos);
        if dist < minDist
            minDist = dist;
            selectedTrack = t;
        end
    end

    % === LẤY THÔNG TIN TỪ targetPoses ===
    if ~isempty(ta)
        targetPos = ta(1).Position(1:2);
        targetVel = ta(1).Velocity(1:2);
        v = norm(targetVel);
        theta = atan2(targetVel(2), targetVel(1));
        targetRealState = [targetPos(:); v; theta];
    else
        targetRealState = [];
    end

    % === KIỂM TRA target có nằm trong FOV ===
    if ~isempty(targetRealState)
        dir = targetRealState(1:2) - x_mpc(1:2);
        headingVec = [cos(x_mpc(3)); sin(x_mpc(3))];
        phi = acos(dot(headingVec, dir) / (norm(headingVec)*norm(dir) + 1e-6));
        isInFOV = phi < deg2rad(80);
    else
        isInFOV = false;
    end

    % === KIỂM TRA UKF ĐÃ HỘI TỤ CHƯA ===
    if ~isempty(selectedTrack) && selectedTrack.Age > 15
        useInterception = true;
        ukfState = selectedTrack.State;
    else
        useInterception = false;
    end

    % === ĐIỀU KHIỂN EGO ===
    hasSensorDetection = ~isempty(detectionClusters);

    if ~hasSensorDetection
        u = [0; 0];  % Giai đoạn 1: chưa thấy qua sensor
    elseif ~useInterception
        % GIAI ĐOẠN 2: theo targetPoses
        targetIntercept = targetRealState(1:2);
        dir = targetIntercept - x_mpc(1:2);
        angleTo = atan2(dir(2), dir(1));
        headingError = atan2(sin(angleTo - x_mpc(3)), cos(angleTo - x_mpc(3)));
        desiredSpeed = min(12, 6 + 0.6 * norm(dir));
        a = (desiredSpeed - x_mpc(4)) / nlobj.Ts;
        delta = max(min(headingError, 0.3), -0.3);
        u = [a; delta];
    else
        % GIAI ĐOẠN 3: đánh chặn dựa trên track.State (UKF)
        dt = 0.1;
        N = 10;
        pos = ukfState(1:2);
        v = ukfState(3);
        theta = ukfState(4);
        futurePred = zeros(2,N);
        for k = 1:N
            pos = pos + dt * v * [cos(theta); sin(theta)];
            futurePred(:,k) = pos;
        end
        targetIntercept = futurePred(:,end);

        dir = targetIntercept - x_mpc(1:2);
        angleTo = atan2(dir(2), dir(1));
        headingError = atan2(sin(angleTo - x_mpc(3)), cos(angleTo - x_mpc(3)));
        desiredSpeed = min(12, 6 + 0.6 * norm(dir));
        a = (desiredSpeed - x_mpc(4)) / nlobj.Ts;
        delta = max(min(headingError, 0.3), -0.3);
        u = [a; delta];

    end

    % === CẬP NHẬT TRẠNG THÁI EGO ===
    x_mpc = vehicleKinematics(x_mpc, u, nlobj.Ts);
    egoCar.Position = [x_mpc(1), x_mpc(2), 0];
    egoCar.Yaw = rad2deg(x_mpc(3));
    egoCar.Velocity = [cos(x_mpc(3)), sin(x_mpc(3)), 0] * x_mpc(4);
end