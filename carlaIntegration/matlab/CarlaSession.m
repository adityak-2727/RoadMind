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
