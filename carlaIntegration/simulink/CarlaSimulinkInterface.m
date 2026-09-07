classdef CarlaSimulinkInterface < matlab.System
    % CarlaSimulinkInterface - Phase 9 Task 4 interface foundation: a
    % MATLAB System object usable as a Simulink block that sends a
    % steering/throttle/brake control command to CARLA and returns the
    % ego vehicle's current state, via the existing carla/matlab/*.m
    % adapter functions - it never talks to CARLA or Python directly
    % itself, keeping CARLA-specific code isolated behind that same
    % adapter boundary (Task 9).
    %
    % A MATLAB System object (matlab.System) is used rather than a plain
    % "MATLAB Function" block because MATLAB Function blocks execute in a
    % code-generation-restricted subset of MATLAB by default, which does
    % not support the py.* Python interop or handle-class calls the
    % carla/matlab/*.m adapter needs; a MATLAB System object supports
    % ordinary interpreted MATLAB execution, which is what CARLA
    % integration requires.
    %
    % This is deliberately NOT the project's vehicle controller and does
    % NOT rebuild it - it is only the data/control interface skeleton
    % Task 4 asks for. The actual driving logic
    % (control/vehicleController.m, the K1/K2 planning pipeline, etc.)
    % stays exactly where it already is and is completely unaffected by
    % this block's existence; nothing in main.m or demo/runDemo.m
    % references this file.
    %
    % Inputs (per Simulink step):
    %   steer, throttle, brake - control command, forwarded to
    %                            carlaApplyControl.m unchanged (see that
    %                            file for the valid ranges)
    % Outputs (per Simulink step):
    %   egoX, egoY, egoYaw, egoVelocity - the four scalar fields of
    %                            carlaGetEgoState.m's returned egoState
    %                            (config/createEgoState.m's schema)
    %
    % Usage inside a Simulink model: add this as a "MATLAB System" block
    % (Simulink library browser, or programmatically via
    % add_block('simulink/User-Defined Functions/MATLAB System', ...)
    % then set its System object class to 'CarlaSimulinkInterface'). See
    % carla/simulink/buildCarlaControlInterfaceModel.m for a
    % programmatically-built minimal model using this block.

    properties (Nontunable)
        CarlaCfg = carlaConfig();
    end

    methods (Access = protected)
        function setupImpl(obj)
            carlaConnect(obj.CarlaCfg);
            carlaSpawnEgoVehicle(obj.CarlaCfg);
        end

        function [egoX, egoY, egoYaw, egoVelocity] = stepImpl(~, steer, throttle, brake)
            carlaApplyControl(steer, throttle, brake);
            egoState = carlaGetEgoState();
            egoX = egoState.x;
            egoY = egoState.y;
            egoYaw = egoState.yaw;
            egoVelocity = egoState.velocity;
        end

        function releaseImpl(~)
            carlaDisconnect();
        end

        function [o1, o2, o3, o4] = getOutputSizeImpl(~)
            o1 = 1; o2 = 1; o3 = 1; o4 = 1;
        end

        function [o1, o2, o3, o4] = getOutputDataTypeImpl(~)
            o1 = 'double'; o2 = 'double'; o3 = 'double'; o4 = 'double';
        end

        function [o1, o2, o3, o4] = isOutputComplexImpl(~)
            o1 = false; o2 = false; o3 = false; o4 = false;
        end

        function [o1, o2, o3, o4] = isOutputFixedSizeImpl(~)
            o1 = true; o2 = true; o3 = true; o4 = true;
        end
    end
end
