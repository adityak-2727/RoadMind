function modelPath = buildCarlaClosedLoopModel()
% buildCarlaClosedLoopModel - Phase 14: programmatically builds (not
% hand-authored XML/binary) a Simulink model that hosts the REAL CARLA
% closed loop (CarlaClosedLoopBlock.m, which itself only calls
% carlaClosedLoopStep.m - no pipeline stage is reimplemented) as one
% MATLAB System block, with output signals logged to the workspace and a
% Stop Simulation block that ends the run once the ego reaches the post-
% intersection goal - mirrors simulink/buildAutonomyPipelineModel.m's
% established pattern exactly, applied to the live CARLA hero scene
% instead of the five synthetic scenarios.
%
% SimulateUsing is 'Interpreted Execution' for the same documented reason
% as every other MATLAB System block in this project touching CARLA or
% the struct/cell-array-based pipeline: py.* interop and struct/cell
% arrays are not code-generation compatible.
%
% StopTime is left generous (300s of simulated time) since real CARLA
% I/O per tick has no simulated-time relationship to wall-clock duration
% - the model's own Stop Simulation block (wired to GoalReached) is what
% actually ends the run once the maneuver completes, not StopTime, which
% only exists as an upper bound safety net.
%
% Output:
%   modelPath - full path to the saved .slx file

modelName = 'carlaClosedLoopPipeline';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

new_system(modelName);
open_system(modelName);

set_param(modelName, 'StopTime', '300');
set_param(modelName, 'FixedStep', '0.1');
set_param(modelName, 'SolverType', 'Fixed-step');

blockPath = [modelName, '/CarlaClosedLoop'];
add_block('simulink/User-Defined Functions/MATLAB System', blockPath);
set_param(blockPath, 'Position', [150, 20, 340, 340]);
set_param(blockPath, 'System', 'CarlaClosedLoopBlock');
set_param(blockPath, 'SimulateUsing', 'Interpreted Execution');

outNames = {'EgoX', 'EgoY', 'EgoYaw', 'EgoVelocity', 'DecisionSeverity', 'MinTTC', ...
    'IsColliding', 'DistToGoal', 'GoalReached', 'SteeringDeg', 'ThrottleCmd', ...
    'BrakeCmd', 'FeasibleCandidateCount', 'TickCount', 'RealCollisionCount'};
workspaceVarNames = {'carlaEgoX', 'carlaEgoY', 'carlaEgoYaw', 'carlaEgoVelocity', ...
    'carlaDecisionSeverity', 'carlaMinTTC', 'carlaIsColliding', 'carlaDistToGoal', ...
    'carlaGoalReached', 'carlaSteeringDeg', 'carlaThrottleCmd', 'carlaBrakeCmd', ...
    'carlaFeasibleCandidateCount', 'carlaTickCount', 'carlaRealCollisionCount'};

for i = 1:numel(outNames)
    sinkPath = [modelName, '/', outNames{i}];
    add_block('simulink/Sinks/To Workspace', sinkPath);
    yPos = 10 + (i - 1) * 26;
    set_param(sinkPath, 'Position', [420, yPos, 520, yPos + 14]);
    set_param(sinkPath, 'VariableName', workspaceVarNames{i}, 'SaveFormat', 'Array');
    add_line(modelName, ['CarlaClosedLoop/', num2str(i)], [outNames{i}, '/1']);
end

% Stop Simulation once the ego reaches the post-intersection goal (output
% port 9, GoalReached, 0/1) - mirrors buildAutonomyPipelineModel.m's own
% StopWhenGoalReached block and main.m's `break`.
stopPath = [modelName, '/StopWhenGoalReached'];
add_block('simulink/Sinks/Stop Simulation', stopPath);
set_param(stopPath, 'Position', [420, 400, 460, 420]);
add_line(modelName, 'CarlaClosedLoop/9', 'StopWhenGoalReached/1');

modelPath = fullfile(fileparts(mfilename('fullpath')), [modelName, '.slx']);
save_system(modelName, modelPath);
close_system(modelName, 0);

fprintf('Built and saved %s\n', modelPath);

end
