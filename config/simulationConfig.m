function cfg = simulationConfig()
% simulationConfig - top-level simulation timing/loop parameters (stub, Phase 0).

cfg = struct( ...
    'dt',              0.1, ...   % [s] simulation timestep
    'numSteps',        0, ...     % total steps for current scenario (set by scenario loader)
    'realTimePlot',    false, ...  % whether to render live plots during the loop
    'logToFile',       true, ...  % write per-step logs to results/logs
    'randomSeed',      42 ...     % for reproducible stochastic elements (traffic, noise)
);

end
