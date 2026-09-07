function modelPath = buildCarlaControlInterfaceModel()
% buildCarlaControlInterfaceModel - Phase 9 Task 4: programmatically
% builds (rather than hand-authoring XML/binary) a minimal Simulink model
% establishing the control/data interface skeleton:
%
%   [SteerCmd, ThrottleCmd, BrakeCmd] Inports
%       -> MATLAB System block (CarlaSimulinkInterface)
%       -> [EgoX, EgoY, EgoYaw, EgoVelocity] Outports
%
% This is deliberately NOT the project's vehicle controller - no
% pure-pursuit/bicycle-model logic lives in this model. It only
% establishes that Simulink can host the CARLA control/state interface
% (Task 4), ready for later phases to build real control logic around.
% See carlaIntegration/simulink/CarlaSimulinkInterface.m for what the
% System object block actually does (forwards to the same
% carlaIntegration/matlab/*.m adapter functions used everywhere else).
%
% Run this function to (re)create carlaIntegration/simulink/
% carlaControlInterface.slx. Building it programmatically, rather than
% committing a hand-authored binary .slx, keeps it reproducible and
% diffable in intent (this script) even though the resulting .slx itself
% is still a binary file.
%
% Output:
%   modelPath - full path to the saved .slx file

modelName = 'carlaControlInterface';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

new_system(modelName);
open_system(modelName);

% Simulink runs a plain sim() as fast as the CPU allows by default (no
% wall-clock pacing). CARLA's server advances physics on its own
% real-time clock (asynchronous mode), so an unpaced model calls
% apply_control()+get_vehicle_state() back-to-back with ~0s of real time
% between ticks, starving CARLA of any time to actually move the vehicle
% between reads - confirmed live during Phase 9 verification (velocity
% stayed exactly 0.0000 m/s for 41 consecutive ticks with unpaced
% throttle=0.6, every tick). Pacing at 1x real-time fixes this.
set_param(modelName, 'EnablePacing', 'on');
set_param(modelName, 'PacingRate', 1);

inportNames = {'SteerCmd', 'ThrottleCmd', 'BrakeCmd'};
outportNames = {'EgoX', 'EgoY', 'EgoYaw', 'EgoVelocity'};

for i = 1:numel(inportNames)
    blockPath = [modelName, '/', inportNames{i}];
    add_block('simulink/Sources/In1', blockPath);
    set_param(blockPath, 'Position', [30, 30 + (i - 1) * 80, 60, 60 + (i - 1) * 80]);
    set_param(blockPath, 'Port', num2str(i));
end

interfaceBlockPath = [modelName, '/CarlaInterface'];
add_block('simulink/User-Defined Functions/MATLAB System', interfaceBlockPath);
set_param(interfaceBlockPath, 'Position', [150, 20, 320, 220]);
set_param(interfaceBlockPath, 'System', 'CarlaSimulinkInterface');
% MATLAB System blocks default to attempting code generation, which does
% NOT support py.* Python interop (confirmed live: sim() failed with
% "Function py.sys.path is not supported for code generation" until this
% was set) - CarlaSimulinkInterface needs ordinary interpreted execution.
set_param(interfaceBlockPath, 'SimulateUsing', 'Interpreted Execution');

for i = 1:numel(outportNames)
    blockPath = [modelName, '/', outportNames{i}];
    add_block('simulink/Sinks/Out1', blockPath);
    set_param(blockPath, 'Position', [400, 30 + (i - 1) * 50, 430, 60 + (i - 1) * 50]);
    set_param(blockPath, 'Port', num2str(i));
end

for i = 1:numel(inportNames)
    add_line(modelName, [inportNames{i}, '/1'], ['CarlaInterface/', num2str(i)]);
end
for i = 1:numel(outportNames)
    add_line(modelName, ['CarlaInterface/', num2str(i)], [outportNames{i}, '/1']);
end

modelPath = fullfile(fileparts(mfilename('fullpath')), [modelName, '.slx']);
save_system(modelName, modelPath);
close_system(modelName, 0);

fprintf('Built and saved %s\n', modelPath);

end
