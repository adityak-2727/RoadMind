classdef CarlaSession < handle
% CarlaSession - holds one live CARLA Python-adapter connection/session
% for the whole MATLAB process. A handle class so the single shared
% instance (see getCarlaSession.m's persistent accessor) keeps state
% across separate calls to carlaConnect/carlaSpawnEgoVehicle/
% carlaGetEgoState/carlaApplyControl/carlaDisconnect - each its own
% top-level file/function, per this project's one-function-per-file
% convention - since MATLAB functions in different files cannot share a
% plain `persistent` variable with each other, but they CAN all retrieve
% the same handle-class instance.
%
% This class only ever calls into carla/python/carla_adapter.py via
% MATLAB's built-in Python integration (py.*) - it never talks to CARLA
% directly. Nothing else in the project should construct this class
% directly either; use the free functions in carla/matlab/ (carlaConnect,
% carlaSpawnEgoVehicle, carlaGetEgoState, carlaApplyControl,
% carlaDisconnect), which are the documented Task 3 interface.

    properties (Access = private)
        PyAdapter = [] % the py.carla_adapter.CarlaAdapter instance, once connected
    end

    properties (SetAccess = private)
        Connected = false
    end

    methods
        function connect(obj, cfg)
            if obj.Connected
                return; % already connected - idempotent, Task 3's "handle connection failures cleanly"
            end

            pythonDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'python');
            if count(py.sys.path, pythonDir) == 0
                insert(py.sys.path, int32(0), pythonDir);
            end

            adapterModule = py.importlib.import_module('carla_adapter');
            obj.PyAdapter = adapterModule.CarlaAdapter(pyargs('host', cfg.host, 'port', int32(cfg.port), 'timeout', cfg.timeoutSeconds));
            try
                obj.PyAdapter.connect();
            catch causeErr
                obj.PyAdapter = [];
                error('CarlaSession:connectionFailed', 'Could not connect to CARLA at %s:%d - %s', ...
                    cfg.host, cfg.port, causeErr.message);
            end
            obj.Connected = true;
        end

        function reloaded = loadMap(obj, mapName)
            % Phase 11.6: loads the named CARLA map if it isn't already
            % active - see carla_adapter.py's load_map() docstring for
            % why this exists (connect() alone never loads a specific
            % map). Returns true if a reload actually happened.
            obj.assertConnected();
            reloaded = logical(obj.PyAdapter.load_map(mapName));
        end

        function actorId = spawnEgoVehicle(obj, cfg)
            obj.assertConnected();
            spawnIdx0Based = int32(cfg.spawnPointIndex - 1); % project is 1-based, CARLA/Python side is 0-based
            actorId = double(obj.PyAdapter.spawn_ego_vehicle(pyargs('blueprint_id', cfg.egoBlueprint, 'spawn_point_index', spawnIdx0Based)));
        end

        function rawState = getRawState(obj)
            obj.assertConnected();
            pyState = obj.PyAdapter.get_vehicle_state();
            rawState = CarlaSession.pyDictToStruct(pyState);
        end

        function applied = applyControl(obj, steer, throttle, brake)
            obj.assertConnected();
            pyResult = obj.PyAdapter.apply_control(pyargs('steer', steer, 'throttle', throttle, 'brake', brake));
            applied = CarlaSession.pyDictToStruct(pyResult);
        end

        function sensorId = attachCamera(obj, camCfg)
            obj.assertConnected();
            sensorId = double(obj.PyAdapter.attach_camera(pyargs( ...
                'width', int32(camCfg.width), 'height', int32(camCfg.height), 'fov', camCfg.fov, ...
                'mount_x', camCfg.mountX, 'mount_y', camCfg.mountY, 'mount_z', camCfg.mountZ, ...
                'mount_pitch', camCfg.mountPitch, 'mount_yaw', camCfg.mountYaw, 'mount_roll', camCfg.mountRoll)));
        end

        function sensorId = attachLidar(obj, lidarCfg)
            obj.assertConnected();
            sensorId = double(obj.PyAdapter.attach_lidar(pyargs( ...
                'channels', int32(lidarCfg.channels), 'range_m', lidarCfg.range, ...
                'points_per_second', int32(lidarCfg.pointsPerSecond), 'rotation_frequency', lidarCfg.rotationFrequency, ...
                'upper_fov', lidarCfg.upperFov, 'lower_fov', lidarCfg.lowerFov, ...
                'mount_x', lidarCfg.mountX, 'mount_y', lidarCfg.mountY, 'mount_z', lidarCfg.mountZ, ...
                'mount_pitch', lidarCfg.mountPitch, 'mount_yaw', lidarCfg.mountYaw, 'mount_roll', lidarCfg.mountRoll)));
        end

        function sensorId = attachRadar(obj, radarCfg)
            obj.assertConnected();
            sensorId = double(obj.PyAdapter.attach_radar(pyargs( ...
                'horizontal_fov', radarCfg.horizontalFov, 'vertical_fov', radarCfg.verticalFov, ...
                'range_m', radarCfg.range, 'points_per_second', int32(radarCfg.pointsPerSecond), ...
                'mount_x', radarCfg.mountX, 'mount_y', radarCfg.mountY, 'mount_z', radarCfg.mountZ, ...
                'mount_pitch', radarCfg.mountPitch, 'mount_yaw', radarCfg.mountYaw, 'mount_roll', radarCfg.mountRoll)));
        end

        function frame = getCameraFrame(obj)
            % Returns [] if no frame has arrived yet (sensor just attached
            % / world hasn't ticked since) - callers must handle this,
            % never assume a frame is immediately available.
            obj.assertConnected();
            pyResult = obj.PyAdapter.get_camera_frame();
            if isa(pyResult, 'py.NoneType')
                frame = [];
                return;
            end
            s = CarlaSession.pyDictToStruct(pyResult);
            frame.width = s.width;
            frame.height = s.height;
            frame.fov = s.fov;
            frame.frame = s.frame;
            frame.timestamp = s.timestamp_s;
            rgbBytes = uint8(pyResult{'rgb_bytes'});
            frame.image = permute(reshape(rgbBytes, [3, frame.width, frame.height]), [3, 2, 1]);
        end

        function points = getLidarPoints(obj)
            obj.assertConnected();
            pyResult = obj.PyAdapter.get_lidar_points();
            if isa(pyResult, 'py.NoneType')
                points = [];
                return;
            end
            s = CarlaSession.pyDictToStruct(pyResult);
            pointBytes = uint8(pyResult{'points_bytes'});
            flat = typecast(pointBytes, 'single');
            points.xyzi = reshape(double(flat), [4, s.num_points])'; % Nx4: [x, y, z, intensity], sensor-local CARLA frame
            points.numPoints = s.num_points;
            points.frame = s.frame;
            points.timestamp = s.timestamp_s;
        end

        function detections = getRadarDetections(obj)
            obj.assertConnected();
            pyResult = obj.PyAdapter.get_radar_detections();
            if isa(pyResult, 'py.NoneType')
                detections = [];
                return;
            end
            s = CarlaSession.pyDictToStruct(pyResult);
            detBytes = uint8(pyResult{'detections_bytes'});
            flat = typecast(detBytes, 'single');
            detections.raw = reshape(double(flat), [4, s.num_detections])'; % Nx4: [depth_m, azimuth_rad, altitude_rad, velocity_mps]
            detections.numDetections = s.num_detections;
            detections.frame = s.frame;
            detections.timestamp = s.timestamp_s;
        end

        function objects = getNearbyActorObjects(obj, rangeM)
            % Simulator-grounded actor metadata (CARLA's own ground truth
            % of nearby vehicle/pedestrian actors) - NOT an image-based
            % detector. See carla_adapter.py's get_nearby_actor_objects()
            % docstring.
            obj.assertConnected();
            pyResult = obj.PyAdapter.get_nearby_actor_objects(pyargs('range_m', rangeM));
            s = CarlaSession.pyDictToStruct(pyResult);
            objBytes = uint8(pyResult{'objects_bytes'});
            flat = typecast(objBytes, 'single');
            objects.raw = reshape(double(flat), [13, s.num_objects])'; % Nx13, see carla_adapter.py's field order
            objects.numObjects = s.num_objects;
            objects.classNames = cellfun(@char, cell(s.class_names), 'UniformOutput', false);
            objects.frame = s.frame;
            objects.timestamp = s.timestamp_s;
        end

        function actorId = spawnActorRelativeToEgo(obj, blueprintId, forwardM, rightM, upM, yawOffsetDeg)
            % Spawns a NON-EGO actor at a position relative to the ego
            % vehicle's current transform - Phase 10 coordinate
            % verification / validation scene only. actorId is [] if the
            % spawn failed (e.g. collision at that exact point).
            obj.assertConnected();
            pyResult = obj.PyAdapter.spawn_actor_relative_to_ego(pyargs( ...
                'blueprint_id', blueprintId, 'forward_m', forwardM, 'right_m', rightM, ...
                'up_m', upM, 'yaw_offset_deg', yawOffsetDeg));
            if isa(pyResult, 'py.NoneType')
                actorId = [];
            else
                actorId = double(pyResult);
            end
        end

        function setActorTargetVelocity(obj, actorId, vx, vy, vz)
            obj.assertConnected();
            obj.PyAdapter.set_actor_target_velocity(pyargs('actor_id', int32(actorId), 'vx', vx, 'vy', vy, 'vz', vz));
        end

        function setActorVelocityRelativeToEgo(obj, actorId, forwardMps, rightMps, upMps)
            % Phase 13: sets a non-ego actor's velocity as components
            % along the ego's OWN current forward/right axes (recomputed
            % from the live ego transform), guaranteeing a "closing right"
            % component actually moves the actor toward the ego's path
            % regardless of the road's absolute world heading - see
            % carla_adapter.py's set_actor_velocity_relative_to_ego()
            % docstring for the live bug this replaces.
            obj.assertConnected();
            obj.PyAdapter.set_actor_velocity_relative_to_ego(pyargs( ...
                'actor_id', int32(actorId), 'forward_mps', forwardMps, ...
                'right_mps', rightMps, 'up_mps', upMps));
        end

        function rawState = getActorState(obj, actorId)
            obj.assertConnected();
            pyState = obj.PyAdapter.get_actor_state(pyargs('actor_id', int32(actorId)));
            rawState = CarlaSession.pyDictToStruct(pyState);
        end

        function actorId = spawnEgoVehicleAtTransform(obj, egoBlueprint, x, y, z, yawDeg)
            % Phase 11.5: spawns the ego at an explicit world transform
            % (resolved from the hero scene's actual road waypoints - see
            % config/carlaIndianSceneConfig.m), instead of by CARLA's
            % generic spawn_points() index (spawnEgoVehicle's approach).
            obj.assertConnected();
            actorId = double(obj.PyAdapter.spawn_ego_vehicle_at_transform(pyargs( ...
                'blueprint_id', egoBlueprint, 'x', x, 'y', y, 'z', z, 'yaw_deg', yawDeg)));
        end

        function actorId = spawnActorAtTransform(obj, blueprintId, x, y, z, yawDeg)
            % Phase 11.5: spawns a non-ego actor at an explicit world
            % transform. actorId is [] if the spawn point was occupied.
            obj.assertConnected();
            pyResult = obj.PyAdapter.spawn_actor_at_transform(pyargs( ...
                'blueprint_id', blueprintId, 'x', x, 'y', y, 'z', z, 'yaw_deg', yawDeg));
            if isa(pyResult, 'py.NoneType')
                actorId = [];
            else
                actorId = double(pyResult);
            end
        end

        function frame = captureSnapshot(obj, x, y, z, yawDeg, pitchDeg, width, height, fov)
            % Phase 11.5: one-shot RGB capture from an arbitrary world
            % transform (evidence screenshots), independent of the
            % ego-mounted camera. [] if no frame arrived within the
            % adapter's internal timeout.
            obj.assertConnected();
            pyResult = obj.PyAdapter.capture_snapshot(pyargs( ...
                'x', x, 'y', y, 'z', z, 'yaw_deg', yawDeg, 'pitch_deg', pitchDeg, ...
                'width', int32(width), 'height', int32(height), 'fov', fov));
            if isa(pyResult, 'py.NoneType')
                frame = [];
                return;
            end
            s = CarlaSession.pyDictToStruct(pyResult);
            frame.width = s.width;
            frame.height = s.height;
            rgbBytes = uint8(pyResult{'rgb_bytes'});
            frame.image = permute(reshape(rgbBytes, [3, frame.width, frame.height]), [3, 2, 1]);
        end

        function count = freezeTrafficLights(obj, x, y, rangeM)
            % Phase 11.5: freezes every traffic light within rangeM of
            % (x,y) to a fixed, non-cycling state - infrastructure
            % remains visible but never controls right-of-way. Returns
            % the number of traffic lights frozen.
            obj.assertConnected();
            count = double(obj.PyAdapter.freeze_traffic_lights(pyargs('x', x, 'y', y, 'range_m', rangeM)));
        end

        function destroyOtherActors(obj)
            % Destroys every non-ego actor spawned via
            % spawnActorRelativeToEgo() so far, WITHOUT disconnecting or
            % destroying the ego/sensors. Phase 12: lets a multi-section
            % demo (carlaTrackingPredictionDemo.m) reset the scene to a
            % clean slate between sections instead of accumulating actors
            % (and collision risk for later spawns) across an entire run.
            obj.assertConnected();
            obj.PyAdapter.destroy_other_actors();
        end

        function disconnect(obj)
            if ~obj.Connected
                return;
            end
            try
                obj.PyAdapter.disconnect();
            catch
                % Cleanup must never itself throw and block the caller
                % from finishing - Task 3's "disconnect/cleanup safely".
            end
            obj.PyAdapter = [];
            obj.Connected = false;
        end
    end

    methods (Access = private)
        function assertConnected(obj)
            if ~obj.Connected
                error('CarlaSession:notConnected', 'Call carlaConnect() before using this function.');
            end
        end
    end

    methods (Static, Access = private)
        function s = pyDictToStruct(pyDict)
            s = struct();
            keysList = cell(py.list(pyDict.keys()));
            for i = 1:numel(keysList)
                key = char(keysList{i});
                value = pyDict{keysList{i}};
                s.(key) = CarlaSession.pyValueToMatlab(value);
            end
        end

        function v = pyValueToMatlab(value)
            if isa(value, 'py.NoneType')
                v = [];
            elseif isa(value, 'py.dict')
                v = CarlaSession.pyDictToStruct(value);
            elseif isa(value, 'py.int') || isa(value, 'py.float')
                v = double(value);
            elseif isa(value, 'py.str')
                v = char(value);
            else
                v = value; % pass through unrecognized types rather than guessing a conversion
            end
        end
    end
end
